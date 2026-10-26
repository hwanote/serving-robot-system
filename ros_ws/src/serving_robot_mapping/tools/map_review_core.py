#!/usr/bin/env python3
"""Review Nav2 raw maps without modifying the original.

Usage: python3 r16_148_wall_editor.py prepare
       python3 r16_148_wall_editor.py inspect WALLS.json
       python3 r16_148_wall_editor.py apply WALLS.json [--allow-free]
"""
import argparse
import base64
import hashlib
import html
import json
import math
import re
import struct
import zlib
from pathlib import Path

DIR = Path('/mnt/c/dev/finalpj/mapping-run-logs')
SRC = DIR / 'R16-147_official_raw_map.pgm'
YAML = DIR / 'R16-147_official_raw_map.yaml'
STEM = DIR / 'R16-148_reviewed_map'


def pgm_read(path):
    raw = path.read_bytes()
    pos = 0

    def token():
        nonlocal pos
        while pos < len(raw):
            if raw[pos] in b' \t\r\n':
                pos += 1
            elif raw[pos] == 35:
                end = raw.find(b'\n', pos)
                if end < 0:
                    raise ValueError('PGM comment header truncated')
                pos = end + 1
            else:
                break
        start = pos
        while pos < len(raw) and raw[pos] not in b' \t\r\n':
            pos += 1
        value = raw[start:pos]
        if pos < len(raw):
            pos += 1
        return value

    if token() != b'P5':
        raise ValueError('Only binary P5 PGM is supported')
    w, h, maximum = map(int, (token(), token(), token()))
    if not (3 <= w <= 4096 and 3 <= h <= 4096 and maximum == 255):
        raise ValueError('Unsupported map dimensions/bit depth')
    pixels = raw[pos:]
    if len(pixels) != w * h:
        raise ValueError(f'PGM pixel count mismatch: {len(pixels)} vs {w*h}')
    return w, h, bytearray(pixels)


def metadata():
    contents = YAML.read_text()
    if not re.search(r'^\s*mode:\s*["\']?raw["\']?\s*$', contents, re.M):
        raise ValueError('Source YAML must use mode: raw')
    match = re.search(r'^\s*resolution:\s*([\d.]+)', contents, re.M)
    if not match:
        raise ValueError('Resolution missing')
    resolution = float(match.group(1))
    if not (math.isfinite(resolution) and 0 < resolution <= 0.3):
        raise ValueError('Invalid map resolution')
    return contents, resolution


def sha():
    return hashlib.sha256(SRC.read_bytes()).hexdigest()


def png_preview(w, h, pixels):
    color = bytearray()
    for val in pixels:
        if val <= 25:
            color.extend((239, 242, 244))
        elif 65 <= val <= 100:
            color.extend((42, 50, 61))
        elif val >= 101:
            color.extend((162, 176, 191))
        else:
            color.extend((215, 192, 137))
    stream = b''.join(b'\0' + color[y*w*3:(y+1)*w*3] for y in range(h))

    def chunk(tag, data):
        return struct.pack('!I', len(data)) + tag + data + struct.pack('!I', zlib.crc32(tag + data) & 0xffffffff)

    return (b'\x89PNG\r\n\x1a\n'
            + chunk(b'IHDR', struct.pack('!2I5B', w, h, 8, 2, 0, 0, 0))
            + chunk(b'IDAT', zlib.compress(stream)) + chunk(b'IEND', b''))


def prepare(w, h, pixels):
    img = base64.b64encode(png_preview(w, h, pixels)).decode('ascii')
    script = r'''<!doctype html><html lang="ko"><meta charset="utf-8"><title>R16-148 지도 벽 검수</title>
<style>body{font:16px sans-serif;background:#151c25;color:#f5f6fa;margin:25px}button{margin:5px;padding:10px 14px;background:#455d82;color:#fff;border:0;border-radius:6px;cursor:pointer}canvas{image-rendering:pixelated;width:800px;max-width:95vw;touch-action:none;border:2px solid #cbd5e1;cursor:crosshair}small{color:#c2cdda}#msg{padding:12px;background:#273444;margin:12px 0}</style>
<h2>R16-148 · 지도 검수 및 벽 연결</h2>
<p>기존 어두운 벽의 <b>짧게 끊어진 구간</b>에만 선을 그리세요. 출입구·통로는 막지 마세요. 회색 미확인 공간을 통째로 칠하지 마세요.</p>
<p><small>밝은 회색: 자유 공간 · 어두운색: 장애물 · 청회색: 미확인 · 분홍색: 제안한 벽</small></p>
<canvas id="canvas" width="__W__" height="__H__"></canvas>
<div><button id="undo">마지막 선 취소</button><button id="clear">선 모두 지우기</button><button id="save">JSON 저장 (다른 이름으로)</button></div>
<div id="msg">그리기: 시작점을 누른 뒤 끝점에서 놓으세요. 각 선은 최대 1m로 검증됩니다.</div>
<script>
const W=__W__,H=__H__,sourceHash='__HASH__';
const img=new Image();img.src='data:image/png;base64,__IMAGE__';
const canvas=document.getElementById('canvas'),ctx=canvas.getContext('2d');
let lines=[],start=null,hover=null;
function xy(e){const r=canvas.getBoundingClientRect();return [Math.max(0,Math.min(W-1,Math.floor((e.clientX-r.left)*W/r.width))),Math.max(0,Math.min(H-1,Math.floor((e.clientY-r.top)*H/r.height)))];}
function draw(){ctx.imageSmoothingEnabled=false;ctx.clearRect(0,0,W,H);if(img.complete)ctx.drawImage(img,0,0);
ctx.lineWidth=1.5;ctx.strokeStyle='#f5369a';for(const a of lines){ctx.beginPath();ctx.moveTo(a[0]+.5,a[1]+.5);ctx.lineTo(a[2]+.5,a[3]+.5);ctx.stroke();}
if(start&&hover){ctx.strokeStyle='#22d3ee';ctx.beginPath();ctx.moveTo(start[0]+.5,start[1]+.5);ctx.lineTo(hover[0]+.5,hover[1]+.5);ctx.stroke();}
document.getElementById('msg').textContent=`벽 선 ${lines.length}개. 원본 지도는 변경되지 않습니다.`;}
img.onload=draw;
canvas.addEventListener('pointerdown',e=>{start=xy(e);hover=start;canvas.setPointerCapture(e.pointerId);draw();});
canvas.addEventListener('pointermove',e=>{if(start){hover=xy(e);draw();}});
canvas.addEventListener('pointerup',e=>{if(!start)return;const end=xy(e);if(end[0]!==start[0]||end[1]!==start[1])lines.push([...start,...end]);start=null;hover=null;draw();});
document.getElementById('undo').onclick=()=>{lines.pop();draw();};
document.getElementById('clear').onclick=()=>{if(confirm('표시한 모든 선을 지울까요?')){lines=[];draw();}};
document.getElementById('save').onclick=async()=>{const obj={format:'R16-148-review-v1',source_sha256:sourceHash,width:W,height:H,segments:lines.map(a=>({x1:a[0],y1:a[1],x2:a[2],y2:a[3]}))};const text=JSON.stringify(obj,null,2);if(window.showSaveFilePicker){try{const handle=await showSaveFilePicker({suggestedName:'R16-148_wall_segments.json',types:[{description:'JSON',accept:{'application/json':['.json']}}]});const file=await handle.createWritable();await file.write(text);await file.close();return;}catch(e){if(e.name==='AbortError')return;}}const a=document.createElement('a');a.href=URL.createObjectURL(new Blob([text],{type:'application/json'}));a.download='R16-148_wall_segments.json';a.click();URL.revokeObjectURL(a.href);};
</script></html>'''
    script = script.replace('__W__', str(w)).replace('__H__', str(h)).replace('__HASH__', sha()).replace('__IMAGE__', img)
    dest = DIR / 'R16-148_wall_editor.html'
    dest.write_text(script, encoding='utf-8')
    print('PASS: Browser editor:', dest)
    print('Open in Windows browser: C:\\dev\\finalpj\\mapping-run-logs\\R16-148_wall_editor.html')
    print('Draw only confirmed short wall gaps; save R16-148_wall_segments.json in the same folder.')


def line_cells(x1,y1,x2,y2):
    dx=abs(x2-x1);dy=-abs(y2-y1);sx=1 if x1<x2 else -1;sy=1 if y1<y2 else -1
    err=dx+dy
    while True:
        yield x1,y1
        if x1==x2 and y1==y2:break
        e2=2*err
        if e2>=dy:err+=dy;x1+=sx
        if e2<=dx:err+=dx;y1+=sy



def validate_connected_wall_lines(records, anchored):
    """Validate connected strokes as wall-anchored chains."""
    tolerance = math.sqrt(2) + 1e-9
    parent = list(range(len(records)))

    def find(index):
        while parent[index] != index:
            parent[index] = parent[parent[index]]
            index = parent[index]
        return index

    def linked(a, b):
        return any(
            math.hypot(
                point[0] - other[0],
                point[1] - other[1],
            ) <= tolerance
            for point in a[1:]
            for other in b[1:]
        )

    for i in range(len(records)):
        for j in range(i + 1, len(records)):
            if linked(records[i], records[j]):
                parent[find(j)] = find(i)

    groups = {}

    for index, record in enumerate(records):
        groups.setdefault(find(index), []).append(record)

    failures = []

    for group in sorted(
        groups.values(),
        key=lambda entries: min(r[0] for r in entries),
    ):
        ids = [record[0] for record in group]
        terminals = []

        for record in group:
            for point in record[1:]:
                connected = any(
                    other[0] != record[0]
                    and math.hypot(
                        point[0] - end[0],
                        point[1] - end[1],
                    ) <= tolerance
                    for other in group
                    for end in other[1:]
                )

                if not connected:
                    terminals.append(point)

        anchors = [
            anchored(*point) for point in terminals
        ]

        label = ",".join(map(str, ids))

        details = ", ".join(
            f"{point}={'WALL' if ok else 'UNANCHORED'}"
            for point, ok in zip(terminals, anchors)
        )

        print(
            f"Wall chain [{label}]: {details}",
            flush=True,
        )

        if len(terminals) != 2 or not all(anchors):
            failures.append(label)

    if failures:
        raise ValueError(
            "Unanchored or branched wall chains: "
            + ", ".join(failures)
            + ". Both outer endpoints must connect to "
            "previously measured walls. "
            "No map changes made."
        )

def inspect_or_apply(action, segments_path, allow_free, w,h,pixels,res, yaml):
    doc=json.loads(Path(segments_path).read_text(encoding='utf-8'))
    if not (doc.get('format')=='R16-148-review-v1' and doc.get('source_sha256')==sha() and doc.get('width')==w and doc.get('height')==h):
        raise ValueError('Editor JSON does not match source map (hash/dimensions)')
    lines=doc.get('segments')
    if not isinstance(lines,list) or not 1<=len(lines)<=20:
        raise ValueError('Require 1..20 short wall segments')
    changes=set()
    records=[]
    anchor_r=max(1,math.ceil(0.20/res))
    def anchored(x,y):
        return any(65<=pixels[yy*w+xx]<=100 for yy in range(max(0,y-anchor_r),min(h,y+anchor_r+1)) for xx in range(max(0,x-anchor_r),min(w,x+anchor_r+1)) if math.hypot(xx-x,yy-y)*res<=0.20+1e-9)
    for i, seg in enumerate(lines,1):
        pts=[seg.get(k) for k in ('x1','y1','x2','y2')]
        if not all(type(x) is int for x in pts):
            raise ValueError(f'Segment {i}: coordinates must be integers')
        x1,y1,x2,y2=pts
        if not all(0<=x<w for x in (x1,x2)) or not all(0<=y<h for y in (y1,y2)):
            raise ValueError(f'Segment {i}: outside map')
        length=math.hypot(x2-x1,y2-y1)*res
        if length<res*.8 or length>1.0:
            raise ValueError(f'Segment {i}: length {length:.2f}m outside (0,1.0]m')
        records.append((i, (x1, y1), (x2, y2)))
        changes.update(y*w+x for x,y in line_cells(*pts))
    validate_connected_wall_lines(records, anchored)
    modified=[idx for idx in changes if pixels[idx] != 100]
    free=[idx for idx in modified if pixels[idx]<=25]
    unknown=[idx for idx in modified if pixels[idx]>=101]
    uncertain=[idx for idx in modified if 25<pixels[idx]<101]
    print(f'Segments={len(lines)}, new wall cells={len(modified)}, free overwritten={len(free)}, unknown replaced={len(unknown)}, uncertain replaced={len(uncertain)}')
    if len(modified)>100:
        raise ValueError('Changes exceed 100 cells; split into small verified corrections')
    if action=='inspect':
        print('INSPECT ONLY; original and output files unchanged.')
        if free:print('CAUTION: Crossing known free cells. Verify these really are walls!')
        return
    if free and not allow_free:
        raise ValueError('Crosses known free space; add --allow-free ONLY after reviewing each line')
    edited=bytearray(pixels)
    for index in changes:edited[index]=100
    img_path=STEM.with_suffix('.pgm')
    yaml_path=STEM.with_suffix('.yaml')
    if img_path.exists() or yaml_path.exists():
        raise ValueError('Reviewed output already exists; refusing to overwrite')
    img_path.write_bytes(f'P5\n{w} {h}\n255\n'.encode()+edited)
    output_yaml=re.sub(r'^\s*image:\s*.*$',f'image: {img_path.name}',yaml,count=1,flags=re.M)
    if output_yaml==yaml:raise ValueError('YAML image field not found')
    yaml_path.write_text(output_yaml)
    report={'label':'OPERATOR_REVIEWED_MAP_NOT_AUTONOMOUS_COMPLETION','source_sha256':sha(),'segments':lines,'new_wall_cells':len(modified),'free_cells_changed':len(free),'unknown_cells_changed':len(unknown),'source':str(SRC),'output':str(yaml_path)}
    (DIR/'R16-148_review_provenance.json').write_text(json.dumps(report,ensure_ascii=False,indent=2)+'\n')
    print('PASS: Reviewed Map:',yaml_path)
    print('PASS: Raw Map untouched:',SRC)
    print('NOTE: Operator edits do not mean MAP_COMPLETE.')


def main():
    parser=argparse.ArgumentParser()
    parser.add_argument('action',choices=('prepare','inspect','apply'))
    parser.add_argument('segments',nargs='?')
    parser.add_argument('--allow-free',action='store_true')
    args=parser.parse_args()
    w,h,pixels=pgm_read(SRC)
    yaml,res=metadata()
    if args.action=='prepare':prepare(w,h,pixels)
    else:
        if not args.segments:parser.error('Specify path to saved JSON')
        inspect_or_apply(args.action,args.segments,args.allow_free,w,h,pixels,res,yaml)

if __name__=='__main__':main()
