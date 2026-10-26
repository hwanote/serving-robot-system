"""R16-176 map draft handoff regression tests."""

import json

import pytest

from serving_robot_mapping.map_draft_policy import (
    classify_map_draft,
    write_map_handoff,
)


def report(outcome="COVERAGE_LIMITED", **kw):
    data = {
        "explorer_terminal_status": outcome,
        "raw_map_saved": True,
        "known_free_m2": 32.0,
        "strict_verified": False,
        "map_complete": False,
    }
    data.update(kw)
    return data


def test_coverage_limited_is_review_draft():
    result = classify_map_draft(report())
    assert result["handoff_state"] == "MAP_DRAFT_READY"
    assert result["map_draft_ready"] is True
    assert result["final_map_approved"] is False


def test_time_limit_is_not_completed_draft():
    result = classify_map_draft(
        report("TIME_LIMIT_REACHED")
    )
    assert result["handoff_state"] == (
        "PARTIAL_MAP_CHECKPOINT"
    )
    assert result["map_draft_ready"] is False


def test_safety_halt_not_misclassified():
    result = classify_map_draft(
        report("SAFETY_HALTED_UNFINISHED")
    )
    assert result["map_draft_ready"] is False
    assert result["navigation_map_authorized"] is False


def test_auto_complete_still_needs_review():
    result = classify_map_draft(report(
        "EXPLORER_REPORTED_MAP_COMPLETE",
        strict_verified=True,
        map_complete=True,
    ))
    assert result["auto_map_verified"] is True
    assert result["final_map_approved"] is False


def test_small_or_unsaved_map_not_ready():
    assert not classify_map_draft(
        report(known_free_m2=0.1)
    )["map_draft_ready"]
    assert not classify_map_draft(
        report(raw_map_saved=False)
    )["map_draft_ready"]


def test_handoff_preserves_source_and_hashes(tmp_path):
    pgm = tmp_path / "raw.pgm"
    yaml = tmp_path / "raw.yaml"
    pgm.write_bytes(b"P5\n3 3\n255\n" + bytes([0] * 9))
    yaml.write_text("image: raw.pgm\n")

    status = tmp_path / "R16-147_session_result.json"
    data = report(
        map_files=[str(pgm), str(yaml)]
    )
    original = json.dumps(data)
    status.write_text(original)

    result = write_map_handoff(data, status)
    assert result["map_draft_ready"] is True
    assert status.read_text() == original

    manifest = json.loads((
        tmp_path / "R16-176_map_draft_manifest.json"
    ).read_text())

    assert len(manifest["original_map_files"]) == 2
    assert manifest["final_map_approved"] is False

    with pytest.raises(FileExistsError):
        write_map_handoff(data, status)


def test_missing_artifacts_fail_closed(tmp_path):
    status = tmp_path / "R16-147_session_result.json"
    data = report(map_files=[
        str(tmp_path / "missing.pgm"),
        str(tmp_path / "missing.yaml"),
    ])
    result = write_map_handoff(data, status)

    assert result["map_draft_ready"] is False
    assert result["handoff_state"] == (
        "MAP_ARTIFACT_REVIEW_REQUIRED"
    )
