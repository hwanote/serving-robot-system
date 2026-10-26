"""Separate auto exploration results from human map approval."""

import hashlib
import json
import math
from pathlib import Path


def classify_map_draft(report):
    """Classify exploration without authorizing a navigation map."""
    outcome = report.get("explorer_terminal_status")
    saved = report.get("raw_map_saved") is True

    try:
        area = report.get("known_free_m2")
        valid_area = (
            not isinstance(area, bool)
            and math.isfinite(float(area))
            and float(area) >= 10.0
        )
    except (TypeError, ValueError, OverflowError):
        valid_area = False

    auto_verified = (
        saved
        and valid_area
        and outcome == "EXPLORER_REPORTED_MAP_COMPLETE"
        and report.get("strict_verified") is True
        and report.get("map_complete") is True
    )

    limited = (
        saved
        and valid_area
        and outcome == "COVERAGE_LIMITED"
    )

    if auto_verified:
        state = "AUTO_MAP_VERIFIED_PENDING_REVIEW"
    elif limited:
        state = "MAP_DRAFT_READY"
    elif not saved:
        state = "MAP_SAVE_FAILED"
    elif outcome == "TIME_LIMIT_REACHED":
        state = "PARTIAL_MAP_CHECKPOINT"
    elif outcome in (
        "SAFETY_HALTED_UNFINISHED",
        "NAVIGATION_HALTED_UNFINISHED",
    ):
        state = "SAFETY_REVIEW_REQUIRED"
    else:
        state = "AUTO_EXPLORATION_UNFINISHED"

    ready = auto_verified or limited

    return {
        "handoff_state": state,
        "map_draft_ready": ready,
        "manual_review_required": ready,
        "auto_exploration_limited": limited,
        "auto_map_verified": auto_verified,
        "final_map_approved": False,
        "navigation_map_authorized": False,
    }


def write_map_handoff(report, status_path):
    """Write separate provenance without changing original maps."""
    status_path = Path(status_path).resolve()
    output = status_path.parent

    result = classify_map_draft(report)
    originals = []
    valid_files = True

    if report.get("raw_map_saved") is True:
        files = report.get("map_files")

        if (
            not isinstance(files, list)
            or len(files) != 2
        ):
            valid_files = False
        else:
            suffixes = set()

            for filename in files:
                path = Path(filename).resolve()

                if (
                    path.parent != output
                    or not path.is_file()
                ):
                    valid_files = False
                    break

                suffixes.add(path.suffix.lower())

                originals.append({
                    "path": str(path),
                    "sha256": hashlib.sha256(
                        path.read_bytes()
                    ).hexdigest(),
                    "size_bytes": path.stat().st_size,
                })

            if suffixes != {".pgm", ".yaml"}:
                valid_files = False

        if not valid_files:
            result.update({
                "handoff_state": "MAP_ARTIFACT_REVIEW_REQUIRED",
                "map_draft_ready": False,
                "manual_review_required": False,
                "auto_map_verified": False,
                "navigation_map_authorized": False,
            })
            originals = []

    manifest_path = (
        output / "R16-176_map_draft_manifest.json"
    )
    handoff_path = (
        output / "R16-176_map_draft_handoff.json"
    )

    if manifest_path.exists() or handoff_path.exists():
        raise FileExistsError(
            "R16-176 handoff already exists; "
            "refusing to overwrite"
        )

    manifest = {
        "format": "R16-176-map-draft-v1",
        "source_status": str(status_path),
        "exploration_outcome":
            report.get("explorer_terminal_status"),
        "handoff_state": result["handoff_state"],
        "original_map_files": originals,
        "original_map_unchanged": True,
        "editor_contract": {
            "edit_format": "R16-148-review-v1",
            "coordinate_system": "PGM_TOP_LEFT_PIXELS",
            "ui_owner": "frontend",
            "validation_owner": "mapping",
            "source_hash_required": True,
        },
        "unobserved_space_requires_review": True,
        "final_map_approved": False,
        "navigation_map_authorized": False,
    }

    if originals:
        with manifest_path.open(
            "x", encoding="utf-8"
        ) as file:
            json.dump(
                manifest, file,
                indent=2, ensure_ascii=False
            )
            file.write("\n")

        result["map_draft_manifest"] = str(
            manifest_path
        )

    result["source_status"] = str(status_path)

    with handoff_path.open("x", encoding="utf-8") as file:
        json.dump(
            result, file,
            indent=2, ensure_ascii=False
        )
        file.write("\n")

    return result
