#!/usr/bin/env python3
"""Rough post-correction of an exploration (raw) map: seal the holes the SLAM grid left open.

Why: the autonomous mapping run saves a raw map (0 = free, 100 = occupied, 255 = unknown).
Walls can be missing at the grid edge or where the scanner never saw, so free cells touch
unknown cells. This script closes those leaks in a *separate* copy and marks every cell it
touched, so a person can review and hand-edit the result. It does NOT make the map "verified
complete": the strict mapping check passes trivially after sealing. Look at the review image.

    python3 scripts/seal_map_edges.py seal   RAW.yaml
    python3 scripts/seal_map_edges.py seal   RAW.yaml --shift-origin -3.2 -2.4
    python3 scripts/seal_map_edges.py review CORRECTED.yaml        # after hand edits

seal   -> <stem>_corrected.{pgm,yaml}   map for Nav2, in the usual trinary look (black wall = 0,
                                         white free = 254, grey unknown = 205, like maps/stores/*).
                                         Edit this one by hand in any image editor: dark = wall,
                                         bright = free, mid grey = unknown (anti-aliasing is fine)
          <stem>_autofix_baseline.pgm   the map exactly as the script left it (same encoding)
          <stem>_autofix_mask.pgm       255 = cell added by the script
          <stem>_autofix_report.json    segments (map coordinates), counts
          <stem>_review.png             red = auto-added, blue = changed by hand later,
                                         black = wall, white = free, grey = unknown
review -> rewrite only <stem>_review.png from the current corrected map

Input may be "mode: raw" or "mode: trinary".

--shift-origin DX DY adds (DX, DY) metres to the map origin of the corrected yaml (pixels do not
change). SLAM maps start at the robot's start pose, so the saved origin is relative to it; with
the Gazebo spawn pose (-3.2, -2.4) the shifted map lines up with the world frame that
config/common uses. The report keeps both origins. Only a pure shift (yaw 0 start) is supported.

The raw file is never modified. Needs numpy and PyYAML only (no ROS).
"""
import argparse
import json
import os
import struct
import sys
import zlib

import numpy as np
import yaml

FREE, OCC, UNK = 0, 100, 255
SCALE = 6
COLORS = {
    'free': (255, 255, 255), 'occ': (0, 0, 0), 'unk': (170, 170, 170),
    'auto': (225, 40, 40), 'manual': (30, 90, 230),
}
_N4 = ((1, 0), (-1, 0), (0, 1), (0, -1))


def read_pgm(path):
    with open(path, 'rb') as f:
        data = f.read()
    tokens, pos = [], 0
    while len(tokens) < 4:
        while data[pos:pos + 1].isspace():
            pos += 1
        if data[pos:pos + 1] == b'#':
            while data[pos:pos + 1] != b'\n':
                pos += 1
            continue
        start = pos
        while not data[pos:pos + 1].isspace():
            pos += 1
        tokens.append(data[start:pos])
    pos += 1
    if tokens[0] != b'P5' or int(tokens[3]) != 255:
        raise ValueError('%s: only binary P5 maxval 255 is supported' % path)
    w, h = int(tokens[1]), int(tokens[2])
    return np.frombuffer(data, np.uint8, w * h, pos).reshape(h, w).copy()


def write_pgm(path, a):
    with open(path, 'wb') as f:
        f.write(b'P5\n%d %d\n255\n' % (a.shape[1], a.shape[0]))
        f.write(np.ascontiguousarray(a, np.uint8).tobytes())


def write_png(path, rgb):
    h, w, _ = rgb.shape
    raw = b''.join(b'\x00' + rgb[r].tobytes() for r in range(h))

    def chunk(tag, body):
        c = struct.pack('>I', len(body)) + tag + body
        return c + struct.pack('>I', zlib.crc32(tag + body) & 0xffffffff)
    with open(path, 'wb') as f:
        f.write(b'\x89PNG\r\n\x1a\n' + chunk(b'IHDR', struct.pack('>IIBBBBB', w, h, 8, 2, 0, 0, 0))
                + chunk(b'IDAT', zlib.compress(raw, 9)) + chunk(b'IEND', b''))


TRI_OCC, TRI_FREE, TRI_UNK = 0, 254, 205       # pixels of a trinary map (as in maps/stores/*)
OUT_OCC_T, OUT_FREE_T = 0.65, 0.196


def decode_trinary(pix, negate=0, occ_t=OUT_OCC_T, free_t=OUT_FREE_T):
    """Pixels -> cell codes with the map_server rule (so hand-edited grey shades still work)."""
    p = pix.astype(np.float64) / 255.0 if negate else (255 - pix.astype(np.float64)) / 255.0
    g = np.full(pix.shape, UNK, np.uint8)
    g[p > occ_t] = OCC
    g[p < free_t] = FREE
    return g


def encode_trinary(grid):
    pix = np.full(grid.shape, TRI_UNK, np.uint8)
    pix[grid == OCC] = TRI_OCC
    pix[grid == FREE] = TRI_FREE
    return pix


def load_map(yaml_path):
    """(meta, grid with FREE/OCC/UNK codes) from a raw or trinary map."""
    with open(yaml_path, encoding='utf-8') as f:
        meta = yaml.safe_load(f)
    mode = str(meta.get('mode', 'trinary'))
    img = os.path.join(os.path.dirname(os.path.abspath(yaml_path)), meta['image'])
    pix = read_pgm(img)
    if mode == 'raw':
        bad = set(np.unique(pix).tolist()) - {FREE, OCC, UNK}
        if bad:
            raise ValueError('unexpected pixel values %s (raw map expects 0/100/255)' % sorted(bad))
        return meta, pix
    if mode == 'trinary':
        return meta, decode_trinary(pix, int(meta.get('negate', 0)),
                                    float(meta.get('occupied_thresh', OUT_OCC_T)),
                                    float(meta.get('free_thresh', OUT_FREE_T)))
    raise ValueError('only "mode: raw" or "mode: trinary" maps are supported, got %r' % mode)


def seal(grid, pad=1):
    """Pad with unknown, then turn every unknown cell next to a free cell into a wall.

    Returns (sealed grid, mask of added cells). A free cell on the old border gets its wall
    in the padding."""
    g = np.pad(grid, pad, constant_values=UNK)
    free = g == FREE
    near_free = np.zeros_like(free)
    near_free[1:, :] |= free[:-1, :]
    near_free[:-1, :] |= free[1:, :]
    near_free[:, 1:] |= free[:, :-1]
    near_free[:, :-1] |= free[:, 1:]
    mask = near_free & (g == UNK)
    out = g.copy()
    out[mask] = OCC
    return out, mask


def leaks(grid):
    """Free cells that touch unknown (or the grid edge) in 4 directions."""
    g = np.pad(grid, 1, constant_values=UNK)
    free = g == FREE
    unk = g == UNK
    touch = np.zeros_like(free)
    touch[1:, :] |= unk[:-1, :]
    touch[:-1, :] |= unk[1:, :]
    touch[:, 1:] |= unk[:, :-1]
    touch[:, :-1] |= unk[:, 1:]
    return int((free & touch).sum())


def segments(mask, res, ox, oy):
    """8-connected groups of added cells with their extent in map coordinates."""
    h, w = mask.shape
    seen = np.zeros_like(mask)
    out = []
    for r0, c0 in zip(*np.nonzero(mask)):
        if seen[r0, c0]:
            continue
        stack, cells = [(r0, c0)], []
        seen[r0, c0] = True
        while stack:
            r, c = stack.pop()
            cells.append((r, c))
            for dr in (-1, 0, 1):
                for dc in (-1, 0, 1):
                    rr, cc = r + dr, c + dc
                    if 0 <= rr < h and 0 <= cc < w and mask[rr, cc] and not seen[rr, cc]:
                        seen[rr, cc] = True
                        stack.append((rr, cc))
        rs = [p[0] for p in cells]
        cs = [p[1] for p in cells]
        out.append({
            'cells': len(cells),
            'x_m': [round(float(ox + min(cs) * res), 3), round(float(ox + (max(cs) + 1) * res), 3)],
            'y_m': [round(float(oy + (h - 1 - max(rs)) * res), 3), round(float(oy + (h - min(rs)) * res), 3)],
        })
    out.sort(key=lambda s: -s['cells'])
    for i, s in enumerate(out, 1):
        s['id'] = i
    return out


def review_image(corrected, baseline, mask):
    rgb = np.empty(corrected.shape + (3,), np.uint8)
    rgb[:] = COLORS['unk']
    rgb[corrected == FREE] = COLORS['free']
    rgb[corrected == OCC] = COLORS['occ']
    rgb[mask & (corrected == baseline)] = COLORS['auto']
    rgb[corrected != baseline] = COLORS['manual']
    return np.repeat(np.repeat(rgb, SCALE, axis=0), SCALE, axis=1)


def _paths(yaml_path, suffix):
    stem = os.path.splitext(os.path.abspath(yaml_path))[0]
    if stem.endswith('_corrected'):
        stem = stem[:-len('_corrected')]
    return {k: stem + v for k, v in (
        ('yaml', '_corrected.yaml'), ('pgm', '_corrected.pgm'), ('base', '_autofix_baseline.pgm'),
        ('mask', '_autofix_mask.pgm'), ('report', '_autofix_report.json'), ('png', '_review.png'))}


def cmd_seal(args):
    meta, grid = load_map(args.map_yaml)
    res = float(meta['resolution'])
    ox, oy, yaw = (float(v) for v in meta['origin'])
    before = leaks(grid)
    pad = 1
    sealed, mask = seal(grid, pad)
    dx, dy = args.shift_origin
    ox2, oy2 = ox - pad * res + dx, oy - pad * res + dy
    p = _paths(args.map_yaml, '')
    new_meta = dict(meta)
    new_meta.update(image=os.path.basename(p['pgm']), origin=[round(ox2, 6), round(oy2, 6), yaw],
                    mode='trinary', negate=0, occupied_thresh=OUT_OCC_T, free_thresh=OUT_FREE_T)
    write_pgm(p['pgm'], encode_trinary(sealed))
    write_pgm(p['base'], encode_trinary(sealed))
    write_pgm(p['mask'], np.where(mask, 255, 0).astype(np.uint8))
    with open(p['yaml'], 'w', encoding='utf-8') as f:
        yaml.safe_dump(new_meta, f, sort_keys=False)
    write_png(p['png'], review_image(sealed, sealed, mask))
    segs = segments(mask, res, ox2, oy2)
    report = {
        'note': 'Auto-corrected copy. NOT a strictly verified map; check the review image.',
        'raw_map': os.path.basename(args.map_yaml),
        'grid_before': list(grid.shape), 'grid_after': list(sealed.shape),
        'origin_before': [ox, oy, yaw], 'origin_after': new_meta['origin'],
        'shift_origin_m': [dx, dy],
        'leaks_before': before, 'leaks_after': leaks(sealed),
        'cells_added': int(mask.sum()), 'segments': segs,
    }
    with open(p['report'], 'w', encoding='utf-8') as f:
        json.dump(report, f, ensure_ascii=False, indent=2)
    print('leak cells (free next to unknown/edge): %d -> %d' % (before, report['leaks_after']))
    print('grid %s -> %s, origin %s -> %s' % (grid.shape, sealed.shape, [ox, oy], new_meta['origin'][:2]))
    print('wall cells added by the script: %d in %d segments' % (int(mask.sum()), len(segs)))
    for s in segs[:15]:
        print('  #%d  %3d cells  x %s  y %s' % (s['id'], s['cells'], s['x_m'], s['y_m']))
    for k in ('yaml', 'pgm', 'mask', 'report', 'png'):
        print('wrote', p[k])
    return 0


def cmd_review(args):
    p = _paths(args.corrected_yaml, '')
    meta, corrected = load_map(args.corrected_yaml)
    baseline, mask = decode_trinary(read_pgm(p['base'])), read_pgm(p['mask']) > 0
    if baseline.shape != corrected.shape:
        raise ValueError('corrected map size changed (%s vs %s); hand edits must keep the grid size'
                         % (corrected.shape, baseline.shape))
    write_png(p['png'], review_image(corrected, baseline, mask))
    changed = int((corrected != baseline).sum())
    print('hand-changed cells: %d, leak cells now: %d' % (changed, leaks(corrected)))
    print('wrote', p['png'])
    return 0


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    sub = ap.add_subparsers(dest='cmd', required=True)
    s = sub.add_parser('seal', help='make the corrected copy from a raw map')
    s.add_argument('map_yaml')
    s.add_argument('--shift-origin', nargs=2, type=float, default=[0.0, 0.0], metavar=('DX', 'DY'),
                   help='metres added to the origin of the corrected map (e.g. -3.2 -2.4)')
    s.set_defaults(fn=cmd_seal)
    r = sub.add_parser('review', help='redraw the review image after hand edits')
    r.add_argument('corrected_yaml')
    r.set_defaults(fn=cmd_review)
    args = ap.parse_args(argv)
    try:
        return args.fn(args)
    except (ValueError, OSError, KeyError) as e:
        print('ERROR:', e, file=sys.stderr)
        return 1


if __name__ == '__main__':
    sys.exit(main())
