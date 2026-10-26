#!/usr/bin/env python3
"""R16-177: offline reviewed-map release preflight."""

import argparse
import hashlib
import importlib.util
import json
from pathlib import Path

import yaml

from serving_robot_mapping.map_release_validation import (
    geometric_validation,
    release_preflight_decision,
)


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def load_json(path):
    data = json.loads(path.read_text(encoding="utf-8"))
    if not isinstance(data, dict):
        raise ValueError("Expected JSON object: " + str(path))
    return data


def load_yaml(path):
    data = yaml.safe_load(path.read_text(encoding="utf-8"))
    if not isinstance(data, dict):
        raise ValueError("Expected YAML mapping")
    return data


def linked_image(yaml_path, metadata):
    image = Path(metadata["image"])
    if not image.is_absolute():
        image = yaml_path.parent / image
    return image.resolve()


def verify_equal(actual, expected, message):
    if actual != expected:
        raise ValueError(message)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--review-dir", required=True)
    args = parser.parse_args()

    folder = Path(args.review_dir).resolve()
    manifest = load_json(
        folder / "map_review_manifest.json"
    )
    result = load_json(
        folder / "map_review_result.json"
    )
    edits_path = folder / "R16-148_wall_segments.json"
    edits = load_json(edits_path)

    source_pgm = Path(manifest["source_pgm"]).resolve()
    source_yaml = Path(manifest["source_yaml"]).resolve()
    reviewed_pgm = folder / "reviewed_map.pgm"
    reviewed_yaml = folder / "reviewed_map.yaml"

    for path in (
        source_pgm, source_yaml,
        reviewed_pgm, reviewed_yaml, edits_path
    ):
        if not path.is_file():
            raise ValueError("Missing map artifact: " + str(path))

    verify_equal(
        sha(source_pgm),
        manifest["source_pgm_sha256"],
        "Original PGM integrity failure",
    )
    verify_equal(
        sha(source_yaml),
        manifest["source_yaml_sha256"],
        "Original YAML integrity failure",
    )
    verify_equal(
        sha(source_pgm),
        result["source_pgm_sha256"],
        "Result source hash mismatch",
    )
    verify_equal(
        sha(edits_path),
        result["edit_json_sha256"],
        "Approved edit JSON changed after apply",
    )
    verify_equal(
        edits["source_sha256"],
        sha(source_pgm),
        "Edit JSON source hash mismatch",
    )
    verify_equal(
        edits["format"],
        "R16-148-review-v1",
        "Unexpected edit format",
    )

    for document in (manifest, result):
        verify_equal(
            document["final_map_approved"],
            False,
            "Unexpected prior approval",
        )
        verify_equal(
            document["navigation_map_authorized"],
            False,
            "Unexpected navigation authorization",
        )

    original_meta = load_yaml(source_yaml)
    reviewed_meta = load_yaml(reviewed_yaml)

    verify_equal(
        linked_image(source_yaml, original_meta),
        source_pgm,
        "Original YAML image mismatch",
    )
    verify_equal(
        linked_image(reviewed_yaml, reviewed_meta),
        reviewed_pgm.resolve(),
        "Reviewed YAML image mismatch",
    )

    for key in (
        "mode", "resolution", "origin",
        "negate", "occupied_thresh", "free_thresh"
    ):
        verify_equal(
            original_meta.get(key),
            reviewed_meta.get(key),
            "Map metadata changed: " + key,
        )

    verify_equal(
        original_meta.get("mode"),
        "raw",
        "Map is not in raw PGM mode",
    )

    resolution = float(original_meta["resolution"])

    # Reuse original R16-148 PGM reader.
    core_path = (
        Path(__file__).with_name("map_review_core.py")
    )
    spec = importlib.util.spec_from_file_location(
        "r16_148_validation_reader", core_path
    )
    core = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(core)

    width, height, raw = core.pgm_read(source_pgm)
    rw, rh, edited = core.pgm_read(reviewed_pgm)

    verify_equal(
        (width, height),
        (rw, rh),
        "Map dimensions changed",
    )
    verify_equal(
        (edits["width"], edits["height"]),
        (width, height),
        "Edit JSON dimensions mismatch",
    )

    metrics = geometric_validation(
        width, height,
        raw, edited,
        resolution,
        edits["segments"],
    )
    report = release_preflight_decision(metrics)
    report["source_pgm_sha256"] = sha(source_pgm)
    report["reviewed_pgm_sha256"] = sha(reviewed_pgm)
    report["edit_json_sha256"] = sha(edits_path)
    report["reviewed_map_yaml"] = str(reviewed_yaml)
    report["map_frame"] = "map"
    report["pixel_coordinates"] = "PGM_TOP_LEFT"

    output = folder / "R16-177_release_preflight.json"
    contents = json.dumps(
        report, ensure_ascii=False, indent=2
    ) + "\n"

    if output.exists():
        verify_equal(
            output.read_text(encoding="utf-8"),
            contents,
            "Existing preflight differs; "
            "refusing to overwrite",
        )
        print("REPORT_ALREADY_IDENTICAL=True")
    else:
        with output.open("x", encoding="utf-8") as stream:
            stream.write(contents)

    print("========== R16-177 RELEASE PREFLIGHT ==========")
    for key, value in metrics.items():
        print(f"{key}={value}")

    print("MAP_RELEASE_STATE=",
          report["map_release_state"])

    for reason in report["blocking_requirements"]:
        print("BLOCKER=" + reason)

    print("FINAL_MAP_APPROVED=False")
    print("NAVIGATION_MAP_AUTHORIZED=False")
    print("PREFLIGHT_REPORT=" + str(output))


if __name__ == "__main__":
    main()
