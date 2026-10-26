#!/usr/bin/env python3
"""Extend R16-148 review UI with optional geometric suggestions."""

import argparse
import hashlib
import importlib.util
import json
import math
import re
from pathlib import Path

from serving_robot_mapping.map_wall_suggestions import (
    find_wall_gaps,
)


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def load_core():
    path = Path(__file__).with_name("map_review_core.py")
    spec = importlib.util.spec_from_file_location(
        "r16_148_review_core", path
    )
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def image_path(yaml_path):
    text = yaml_path.read_text(encoding="utf-8")
    match = re.search(
        r"(?m)^\s*image\s*:\s*([^\n#]+)",
        text,
    )
    if not match:
        raise ValueError("Map YAML image field missing")

    name = match.group(1).strip().strip("'\"")
    image = Path(name)
    if not image.is_absolute():
        image = yaml_path.parent / image
    return image.resolve()


def geometry(yaml_path):
    text = yaml_path.read_text(encoding="utf-8")
    match = re.search(
        r"(?m)^\s*origin\s*:\s*\[([^\]]+)\]",
        text,
    )
    if not match:
        raise ValueError("Map YAML origin missing")

    values = [
        float(part.strip())
        for part in match.group(1).split(",")
    ]
    if len(values) != 3 or not all(
        math.isfinite(v) for v in values
    ):
        raise ValueError("Invalid map origin")
    return values



NUMBERED_PROPOSAL_UI = r'''
<style id="r16-crisp-label-style">
button.r16-crisp-badge {
  all: unset;
  position: absolute;
  transform: translate(-50%, -50%);
  box-sizing: border-box;
  width: 30px;
  height: 30px;
  display: flex;
  align-items: center;
  justify-content: center;
  border: 3px solid white;
  border-radius: 50%;
  color: white;
  font: 800 16px/1 Arial, sans-serif;
  text-align: center;
  box-shadow: 0 2px 6px #0009;
  cursor: pointer;
  pointer-events: auto;
  z-index: 6;
}
button.r16-crisp-badge:hover,
button.r16-crisp-badge:focus-visible {
  outline: 3px solid #fbbf24;
  outline-offset: 2px;
}
</style>

<script id="r16-176-numbered-wall-review">
(function () {
  "use strict";

  let selected = -1;
  const previousDraw = draw;
  const previousPanel = updateProposalPanel;

  // Put HTML labels ABOVE the low-resolution map canvas.
  const wrapper = document.createElement("div");
  wrapper.style.position = "relative";
  wrapper.style.display = "inline-block";
  wrapper.style.maxWidth = "95vw";
  wrapper.style.verticalAlign = "top";

  canvas.parentNode.insertBefore(wrapper, canvas);
  wrapper.appendChild(canvas);
  canvas.style.display = "block";

  const overlay = document.createElement("div");
  overlay.style.cssText =
    "position:absolute;inset:0;pointer-events:none;z-index:5;";
  wrapper.appendChild(overlay);

  updateProposalPanel = function () {
    previousPanel();

    const root = document.getElementById("auto-items");
    const rows = [...root.children].filter(
      element => element.tagName === "DIV"
    );

    rows.forEach((row, index) => {
      const name = row.querySelector("span");

      if (name) {
        name.textContent =
          `${index + 1}번 · ${name.textContent}`;
      }

      row.style.borderColor =
        selected === index ? "#fbbf24" : "#607086";
      row.style.background =
        selected === index ? "#37465b" : "";

      const locate = document.createElement("button");
      locate.textContent = "위치 보기";
      locate.onclick = () => {
        selected = index;
        draw();
        canvas.scrollIntoView({
          block: "center",
          behavior: "smooth"
        });
      };

      const firstButton = row.querySelector("button");
      if (firstButton) {
        row.insertBefore(locate, firstButton);
      } else {
        row.appendChild(locate);
      }
    });
  };

  function drawLabels() {
    overlay.replaceChildren();

    const positioned = [];
    const cw = canvas.clientWidth;
    const ch = canvas.clientHeight;
    if (!cw || !ch) return;

    for (let index = 0; index < autoSuggestions.length; index++) {
      const proposal = autoSuggestions[index];

      const targetX = canvas.offsetLeft + canvas.clientLeft +
        ((proposal.x1 + proposal.x2 + 1) / 2) * cw / W;
      const targetY = canvas.offsetTop + canvas.clientTop +
        ((proposal.y1 + proposal.y2 + 1) / 2) * ch / H;

      const preferredY = targetY < ch / 2 ? 29 : -29;
      const offsets = [
        [0, preferredY],
        [34, preferredY],
        [-34, preferredY],
        [0, -preferredY],
        [34, -preferredY],
        [-34, -preferredY],
        [58, 0],
        [-58, 0],
        [65, preferredY],
        [-65, preferredY]
      ];

      let chosen = null;
      let bestDistance = -1;

      for (const [ox, oy] of offsets) {
        const x = Math.max(
          17, Math.min(wrapper.clientWidth - 17, targetX + ox)
        );
        const y = Math.max(
          17, Math.min(wrapper.clientHeight - 17, targetY + oy)
        );

        const closest = positioned.length
          ? Math.min(...positioned.map(
              p => Math.hypot(p.x - x, p.y - y)
            ))
          : Infinity;

        if (closest >= 34) {
          chosen = {x, y};
          break;
        }
        if (closest > bestDistance) {
          bestDistance = closest;
          chosen = {x, y};
        }
      }

      if (!chosen) continue;
      positioned.push(chosen);

      // A thin leader visually associates the badge
      // with the exact proposed wall segment.
      const dx = chosen.x - targetX;
      const dy = chosen.y - targetY;
      const distance = Math.hypot(dx, dy);

      const leader = document.createElement("div");
      leader.style.position = "absolute";
      leader.style.left = targetX + "px";
      leader.style.top = targetY + "px";
      leader.style.width = distance + "px";
      leader.style.height = "2px";
      leader.style.background = "#fbbf24";
      leader.style.transformOrigin = "left center";
      leader.style.transform =
        `rotate(${Math.atan2(dy, dx)}rad)`;
      overlay.appendChild(leader);

      const approved = isApplied(proposal);
      const rejected = rejectedSuggestions.has(proposal.id);

      const badge = document.createElement("button");
      badge.type = "button";
      badge.className = "r16-crisp-badge";
      badge.textContent = String(index + 1);
      badge.setAttribute(
        "aria-label",
        `${index + 1}번 벽 연결 제안 위치 선택`
      );
      badge.title = `${index + 1}번 벽 연결 위치`;

      badge.style.left = chosen.x + "px";
      badge.style.top = chosen.y + "px";
      badge.style.background = rejected
        ? "#64748b"
        : approved
          ? "#0f766e"
          : selected === index
            ? "#b45309"
            : "#1d4ed8";

      badge.onclick = event => {
        event.preventDefault();
        event.stopPropagation();
        selected = index;
        draw();
      };

      overlay.appendChild(badge);
    }
  }

  draw = function () {
    previousDraw();
    drawLabels();
  };

  window.addEventListener("resize", drawLabels);

  if (typeof ResizeObserver !== "undefined") {
    new ResizeObserver(drawLabels).observe(canvas);
  }

  img.onload = draw;
  if (img.complete && img.naturalWidth > 0) {
    draw();
  }
})();

</script>'''

def inject_ui(original, suggestions):
    panel = """
<section id="auto-panel" style="
margin:14px 0;padding:12px;background:#273444;
border-radius:8px">
<h3>자동 벽 연결 제안</h3>
<p><small>
주황 점선은 미확인 구간의 추정 벽입니다.
개별 승인 전에는 편집 내용에 반영되지 않습니다.
승인 후에도 실제 구조 검토가 필요합니다.
</small></p>
<div id="auto-items"></div>
</section>
"""
    if original.count('<canvas id="canvas"') != 1:
        raise ValueError("Unsupported R16-148 HTML template")

    original = original.replace(
        '<canvas id="canvas"',
        panel + '<canvas id="canvas"',
        1,
    )

    javascript = r"""
<script>
const autoSuggestions = __PROPOSALS__;
const rejectedSuggestions = new Set();
const previousDraw = draw;

function proposalLine(p) {
  return [p.x1,p.y1,p.x2,p.y2];
}

function sameLine(a,b) {
  return a.every((v,i)=>v===b[i]) ||
    a.every((v,i)=>v===b[3-i]);
}

function isApplied(p) {
  const wanted=proposalLine(p);
  return lines.some(a=>sameLine(a,wanted));
}

function updateProposalPanel() {
  const root=document.getElementById('auto-items');
  root.replaceChildren();

  if(autoSuggestions.length===0){
    const note=document.createElement('p');
    note.textContent='보수적인 조건을 만족하는 자동 제안이 없습니다. 수동 그리기를 사용할 수 있습니다.';
    root.append(note);
    return;
  }

  for(const p of autoSuggestions){
    const row=document.createElement('div');
    row.style.cssText='padding:8px;margin:6px 0;border:1px solid #607086;border-radius:6px';

    const title=document.createElement('span');
    const applied=isApplied(p);
    const rejected=rejectedSuggestions.has(p.id);

    title.textContent=p.id+' · 추정 간격 '+p.gap_m+
      'm · '+(applied?'편집에 반영됨':rejected?'거절됨':'검토 대기');

    const approve=document.createElement('button');
    approve.textContent=applied?'승인 취소':'제안 승인';
    approve.onclick=()=>{
      const wanted=proposalLine(p);
      if(isApplied(p)){
        lines=lines.filter(a=>!sameLine(a,wanted));
      }else{
        if(lines.length>=20){
          alert('검증기는 최대 20개 선분을 허용합니다.');
          return;
        }
        lines.push(wanted);
        rejectedSuggestions.delete(p.id);
      }
      draw();
    };

    const reject=document.createElement('button');
    reject.textContent=rejected?'거절 취소':'거절';
    reject.onclick=()=>{
      const wanted=proposalLine(p);
      if(rejected){
        rejectedSuggestions.delete(p.id);
      }else{
        rejectedSuggestions.add(p.id);
        lines=lines.filter(a=>!sameLine(a,wanted));
      }
      draw();
    };

    row.append(title,approve,reject);
    root.append(row);
  }
}

draw=function(){
  previousDraw();
  ctx.save();

  for(const p of autoSuggestions){
    const applied=isApplied(p);

    if(rejectedSuggestions.has(p.id) && !applied){
      continue;
    }

    ctx.beginPath();
    ctx.lineWidth=applied?2.2:1.7;
    ctx.strokeStyle=applied?'#11c9b0':'#f5a742';
    ctx.setLineDash(applied?[]:[3,2]);
    ctx.moveTo(p.x1+.5,p.y1+.5);
    ctx.lineTo(p.x2+.5,p.y2+.5);
    ctx.stroke();
  }

  ctx.restore();
  updateProposalPanel();
};

img.onload=draw;
if(img.complete && img.naturalWidth>0)draw();
</script>
"""
    javascript = javascript.replace(
        "__PROPOSALS__",
        json.dumps(suggestions, ensure_ascii=False),
    )

    if original.count("</html>") != 1:
        raise ValueError("Unsupported HTML ending")

    return original.replace(
        "</html>", javascript + NUMBERED_PROPOSAL_UI + "</html>", 1
    )


def validate_manifest(output, yaml_path, pgm_path):
    path = output / "map_review_manifest.json"
    manifest = json.loads(
        path.read_text(encoding="utf-8")
    )
    if (
        manifest.get("source_pgm_sha256") != digest(pgm_path)
        or manifest.get("source_yaml_sha256") != digest(yaml_path)
    ):
        raise ValueError(
            "Source map changed after review preparation"
        )
    return manifest


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "action", choices=("prepare", "inspect", "apply")
    )
    parser.add_argument("--map-yaml", required=True)
    parser.add_argument("--output-dir", required=True)
    parser.add_argument("--segments")
    parser.add_argument(
        "--confirm-reviewed", action="store_true"
    )
    args = parser.parse_args()

    yaml_path = Path(args.map_yaml).resolve()
    output = Path(args.output_dir).resolve()
    pgm_path = image_path(yaml_path)

    if not yaml_path.is_file() or not pgm_path.is_file():
        raise ValueError("Map source files not found")

    if args.action != "prepare" and not output.is_dir():
        raise ValueError("Review output directory missing")

    if args.action == "prepare":
        output.mkdir(parents=True, exist_ok=True)

        for name in (
            "map_review.html",
            "map_review_manifest.json",
            "map_wall_suggestions.json",
            "R16-148_wall_editor.html",
        ):
            if (output / name).exists():
                raise ValueError(
                    "Refusing to overwrite " + name
                )

    core = load_core()
    core.DIR = output
    core.SRC = pgm_path
    core.YAML = yaml_path
    core.STEM = output / "reviewed_map"

    width, height, pixels = core.pgm_read(pgm_path)
    _, resolution = core.metadata()

    if args.action == "prepare":
        origin = geometry(yaml_path)
        suggestions = find_wall_gaps(
            pixels, width, height, resolution
        )

        core.prepare(width, height, pixels)
        legacy_html = (
            output / "R16-148_wall_editor.html"
        )
        html = inject_ui(
            legacy_html.read_text(encoding="utf-8"),
            suggestions,
        )
        (output / "map_review.html").write_text(
            html, encoding="utf-8"
        )
        legacy_html.unlink()

        info = {
            "format": "R16-176-wall-suggestions-v1",
            "frame_id": "map",
            "pixel_coordinates": "pgm_top_left",
            "resolution_m": resolution,
            "origin_xy_yaw": origin,
            "width": width,
            "height": height,
            "source_sha256": digest(pgm_path),
            "suggestions": suggestions,
        }
        (output / "map_wall_suggestions.json").write_text(
            json.dumps(info, indent=2, ensure_ascii=False)
            + "\n", encoding="utf-8"
        )

        manifest = {
            "format": "R16-176-map-review-v1",
            "source_pgm": str(pgm_path),
            "source_yaml": str(yaml_path),
            "source_pgm_sha256": digest(pgm_path),
            "source_yaml_sha256": digest(yaml_path),
            "review_state": "PENDING_USER_REVIEW",
            "final_map_approved": False,
            "navigation_map_authorized": False,
        }
        (output / "map_review_manifest.json").write_text(
            json.dumps(manifest, indent=2) + "\n",
            encoding="utf-8",
        )

        print("AUTO_SUGGESTIONS:", len(suggestions))
        print("EDITOR:", output / "map_review.html")
        print("FRONTEND_CONTRACT:",
              output / "map_wall_suggestions.json")
        print("ORIGINAL_MAP_UNCHANGED=True")
        return

    validate_manifest(output, yaml_path, pgm_path)

    if not args.segments:
        parser.error("--segments is required")

    edits_path = Path(args.segments).resolve()
    if not edits_path.is_file():
        raise ValueError("Saved edit JSON missing")

    # Never enable the old --allow-free override.
    if args.action == "inspect":
        core.inspect_or_apply(
            "inspect", edits_path, False,
            width, height, pixels, resolution,
            core.metadata()[0],
        )
        return

    if not args.confirm_reviewed:
        raise ValueError(
            "Explicit --confirm-reviewed required"
        )

    core.inspect_or_apply(
        "apply", edits_path, False,
        width, height, pixels, resolution,
        core.metadata()[0],
    )

    # Reviewed output is still NOT an approved Nav2 map.
    report = {
        "format": "R16-176-reviewed-map-v1",
        "source_pgm_sha256": digest(pgm_path),
        "edit_json_sha256": digest(edits_path),
        "reviewed_pgm": str(
            output / "reviewed_map.pgm"
        ),
        "reviewed_yaml": str(
            output / "reviewed_map.yaml"
        ),
        "review_state": "EDIT_APPLIED_VALIDATION_PENDING",
        "final_map_approved": False,
        "navigation_map_authorized": False,
    }
    (output / "map_review_result.json").write_text(
        json.dumps(report, indent=2) + "\n",
        encoding="utf-8",
    )
    print("REVIEWED_MAP_CREATED=True")
    print("NAVIGATION_MAP_AUTHORIZED=False")


if __name__ == "__main__":
    main()
