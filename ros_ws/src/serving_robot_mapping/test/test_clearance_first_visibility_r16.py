"""Clearance-first visibility fallback regression tests."""

import pytest

import serving_robot_mapping.mapping_visibility_fallback as fallback
from serving_robot_mapping.mapping_visibility_audit import (
    boundary_visibility_audit,
)


def make_grid():
    data = [100] * (32 * 32)

    for y in range(2, 30):
        for x in range(2, 30):
            data[y * 32 + x] = 0

    for y in range(12, 18):
        data[y * 32 + 27] = -1

    return data


def audit(**kwargs):
    return boundary_visibility_audit(
        make_grid(), 32, 32, 0.1, 14 * 32 + 10,
        sensor_range_m=4.0,
        clearance_m=0.2,
        stride_cells=2,
        max_candidates=1,
        **kwargs,
    )


def test_filter_applied_before_final_candidate_limit():
    original = audit()

    assert original["candidates"]
    first = original["candidates"][0]
    rejected = {(first[1], first[2])}

    result = audit(
        viewpoint_filter=lambda x, y: (
            (x, y) not in rejected
        )
    )

    assert result["candidates"]
    assert result["candidates"][0] != first


def test_all_rejected_candidates_never_approved():
    result = audit(
        viewpoint_filter=lambda x, y: False
    )

    assert not result["candidates"]
    assert result["scored_viewpoints"] > 0


def test_helper_filters_guard_failures_before_budget(
    monkeypatch,
):
    def fake_audit(*args, **kwargs):
        approve = kwargs["viewpoint_filter"]

        # First position is too close to an occupied cell.
        assert not approve(9, 9)

        # The later, safer position must remain selectable.
        assert approve(12, 12)

        return {
            "status": "VISIBLE_OBSERVATION_LOCATIONS_EXIST",
            "unknown_boundary_cells": 12,
            "visible_boundary_cells": 8,
            "sampled_viewpoints": 20,
            "scored_viewpoints": 2,
            "candidates": [(5, 12, 12)],
        }

    monkeypatch.setattr(
        fallback, "boundary_visibility_audit", fake_audit
    )

    data = [0] * 400
    grid = (data, 20, 20, 0.1, 1.0, -2.0, 0.0)

    def guard(x, y):
        if x < 2.0:
            return False, "GOAL_NEAR_OCCUPIED_CELL"
        return True, "GOAL_CLEAR"

    selected, metrics = fallback.select_visibility_fallback(
        grid=grid,
        robot_xy=(1.55, -1.45),
        visited=[],
        goal_clear=guard,
        max_results=1,
    )

    assert selected == pytest.approx([(2.25, -0.75)])
    assert metrics["guard_rejections"] == 1
    assert metrics["guard_rejection_reasons"] == {
        "GOAL_NEAR_OCCUPIED_CELL": 1
    }


def test_existing_visit_exclusion_is_preserved(
    monkeypatch,
):
    def fake_audit(*args, **kwargs):
        approve = kwargs["viewpoint_filter"]
        assert not approve(9, 9)
        return {
            "status": "NO_VISIBLE_SAFE_OBSERVATION_LOCATION",
            "unknown_boundary_cells": 12,
            "visible_boundary_cells": 8,
            "sampled_viewpoints": 20,
            "candidates": [],
        }

    monkeypatch.setattr(
        fallback, "boundary_visibility_audit", fake_audit
    )

    grid = ([0] * 400, 20, 20, 0.1, 1.0, -2.0, 0.0)

    selected, metrics = fallback.select_visibility_fallback(
        grid=grid,
        robot_xy=(1.55, -1.45),
        visited=[(1.95, -1.05)],
        goal_clear=lambda x, y: (True, "GOAL_CLEAR"),
    )

    assert not selected
    assert metrics["visited_rejections"] == 1
