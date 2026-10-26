"""R16-177 map release validation safety regressions."""

import pytest

from serving_robot_mapping.map_release_validation import (
    MapReleaseError,
    geometric_validation,
    release_preflight_decision,
)


def example():
    width = height = 30
    raw = [0] * (width * height)

    for x in (10, 11, 12, 13, 14, 17, 18, 19):
        raw[10 * width + x] = 100

    for x in (15, 16):
        raw[10 * width + x] = 205

    edited = list(raw)
    for x in (15, 16):
        edited[10 * width + x] = 100

    segment = {
        "x1": 14, "y1": 10,
        "x2": 17, "y2": 10,
    }
    return width, height, raw, edited, [segment]


def check(data):
    width, height, raw, edited, segments = data
    return geometric_validation(
        width, height, raw, edited, 0.05, segments
    )


def test_exact_approved_unknown_to_wall_pixels():
    metrics = check(example())
    assert metrics[
        "modified_unknown_to_wall_cells"
    ] == 2
    assert metrics["known_free_cells_modified"] == 0


def test_no_free_space_overwrite():
    data = list(example())
    data[3][0] = 100
    with pytest.raises(MapReleaseError):
        check(data)


def test_no_unapproved_map_changes():
    data = list(example())
    data[3][1] = 100
    with pytest.raises(MapReleaseError):
        check(data)


def test_no_missing_approved_edit():
    data = list(example())
    data[3][10 * 30 + 15] = 205
    with pytest.raises(MapReleaseError):
        check(data)


def test_invalid_segment_rejected():
    data = list(example())
    data[4][0]["x1"] = 1000
    with pytest.raises(MapReleaseError):
        check(data)


def test_unknown_is_not_clearance_safe():
    width, height, raw, edited, segments = example()
    metrics = check((
        width, height, raw, edited, segments
    ))
    assert metrics["unknown_treated_as_blocked"]
    assert metrics["clearance_threshold_m"] == 0.48


def test_free_map_edge_is_flagged():
    metrics = check(example())
    assert metrics["free_map_edge_cells"] > 0

    decision = release_preflight_decision(metrics)
    assert (
        "MAP_EXTENT_FREE_EDGE_REVIEW_REQUIRED"
        in decision["blocking_requirements"]
    )


def test_offline_check_cannot_release_map():
    metrics = check(example())
    decision = release_preflight_decision(metrics)

    assert decision["final_map_approved"] is False
    assert decision["navigation_map_authorized"] is False
    assert decision["map_release_state"] == (
        "HOLD_FOR_VALIDATION"
    )


def test_border_wall_edit_requires_review():
    metrics = check(example())
    metrics["edited_wall_cells_on_map_edge"] = 1
    decision = release_preflight_decision(metrics)
    assert (
        "INFERRED_WALL_ON_MAP_BORDER_REVIEW_REQUIRED"
        in decision["blocking_requirements"]
    )
