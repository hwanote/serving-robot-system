"""Prepare a new SLAM PGM/YAML for the map-review service.

Ported unchanged from the mapping team's delivery (integration/ingest_map.py).

Accepts ROS/Nav2 trinary maps or the team's raw occupancy format.
Never modifies source maps, starts ROS, or authorizes navigation.
"""
import argparse
import hashlib
import json
import math
import os
import shutil
import tempfile
from pathlib import Path

import yaml
from PIL import Image


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def fail(message):
    raise ValueError(message)


def ingest(source_yaml, target):
    source_yaml = Path(source_yaml).resolve()
    target = Path(target).absolute()

    if not source_yaml.is_file():
        fail("Source map YAML not found")
    if target.exists():
        fail("Target draft already exists; refusing overwrite")

    meta = yaml.safe_load(source_yaml.read_text(encoding="utf-8"))
    if not isinstance(meta, dict):
        fail("Invalid map metadata")

    origin = meta.get("origin")
    if not isinstance(origin, list) or len(origin) != 3:
        fail("Invalid map origin")
    if not all(
        isinstance(n, (int, float))
        and not isinstance(n, bool)
        and math.isfinite(n)
        for n in origin
    ):
        fail("Nonfinite map origin")
    if abs(origin[2]) > 1e-9:
        fail("Rotated origin is not yet supported")

    resolution = meta.get("resolution")
    if (
        isinstance(resolution, bool)
        or not isinstance(resolution, (int, float))
        or not math.isfinite(resolution)
        or not 0 < resolution <= 1
    ):
        fail("Invalid resolution")

    if meta.get("negate", 0) != 0:
        fail("Negated maps are not supported")

    image_name = meta.get("image")
    if not isinstance(image_name, str) or not image_name:
        fail("Missing image filename")

    image_path = Path(image_name)
    if not image_path.is_absolute():
        image_path = source_yaml.parent / image_path
    image_path = image_path.resolve()

    if not image_path.is_file():
        fail("Source PGM not found")

    with Image.open(image_path) as image:
        if image.mode != "L":
            fail("Only 8-bit grayscale occupancy maps are supported")
        width, height = image.size
        if not (2 <= width <= 10000 and 2 <= height <= 10000):
            fail("Invalid map dimensions")
        pixels = image.tobytes()

    mode = meta.get("mode", "trinary")
    if mode == "raw":
        if not set(pixels) <= {0, 100, 255}:
            fail("Unsupported raw occupancy encoding")
        raw = pixels
    elif mode == "trinary":
        if not set(pixels) <= {0, 205, 254, 255}:
            fail("Unsupported trinary encoding")
        raw = bytes(
            100 if p == 0 else 255 if p == 205 else 0
            for p in pixels
        )
    else:
        fail("Unsupported map mode")

    free = raw.count(0)
    wall = raw.count(100)
    unknown = raw.count(255)

    if not free or free + wall + unknown != width * height:
        fail("Invalid occupancy counts")

    target.parent.mkdir(parents=True, exist_ok=True)
    pending = Path(tempfile.mkdtemp(
        prefix=".mapping_draft_",
        dir=target.parent,
    ))

    try:
        pgm = pending / "R16-147_official_raw_map.pgm"
        cfg = pending / "R16-147_official_raw_map.yaml"
        manifest = pending / "R16-176_map_draft_manifest.json"
        session = pending / "R16-147_session_result.json"

        Image.frombytes("L", (width, height), raw).save(
            pgm, format="PPM"
        )

        cfg.write_text(yaml.safe_dump({
            "image": pgm.name,
            "mode": "raw",
            "resolution": float(resolution),
            "origin": [float(n) for n in origin],
            "negate": 0,
            "occupied_thresh": 0.65,
            "free_thresh": 0.196,
        }, sort_keys=False), encoding="utf-8")

        manifest.write_text(json.dumps({
            "original_map_files": [
                {"path": pgm.name, "sha256": sha(pgm)},
                {"path": cfg.name, "sha256": sha(cfg)},
            ],
            "source_pgm_sha256": sha(image_path),
            "source_yaml_sha256": sha(source_yaml),
        }, indent=2), encoding="utf-8")

        session.write_text(json.dumps({
            "map_draft_ready": True,
            "navigation_map_authorized": False,
            "map_dimensions": [width, height],
            "free_cells": free,
            "occupied_cells": wall,
            "unknown_cells": unknown,
        }, indent=2), encoding="utf-8")

        shutil.copy2(image_path, pending / "source_map.pgm")
        shutil.copy2(source_yaml, pending / "source_map.yaml")

        os.replace(pending, target)
    finally:
        if pending.exists():
            shutil.rmtree(pending)

    return {
        "draft_dir": str(target),
        "width": width,
        "height": height,
        "free_cells": free,
        "occupied_cells": wall,
        "unknown_cells": unknown,
        "source_sha256": sha(image_path),
        "navigation_authorized": False,
    }


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--map-yaml", required=True)
    parser.add_argument("--draft-dir", required=True)
    args = parser.parse_args()
    print(json.dumps(
        ingest(args.map_yaml, args.draft_dir),
        ensure_ascii=False,
        indent=2,
    ))


if __name__ == "__main__":
    main()
