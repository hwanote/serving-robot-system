"""Regression tests for the visibility fallback adapter."""

import pytest

import serving_robot_mapping.mapping_visibility_fallback as mod
from serving_robot_mapping.mapping_visibility_audit import (
    boundary_visibility_audit,
)


def grid():
    data = [100] * 400

    for y in range(1, 19):
        for x in range(1, 19):
            data[y * 20 + x] = 0

    return data, 20, 20, 0.1, 1.0, -2.0, 0.0


def fake_audit(*args, **kwargs):
    assert kwargs["max_candidates"] == 32
    return {
        "status": "VISIBLE_OBSERVATION_LOCATIONS_EXIST",
        "unknown_boundary_cells": 12,
        "visible_boundary_cells": 8,
        "sampled_viewpoints": 20,
        "candidates": [
            (8, 9, 9),
            (5, 12, 12),
        ],
    }


def test_fallback_transforms_grid_candidates(monkeypatch):
    monkeypatch.setattr(mod, "boundary_visibility_audit", fake_audit)

    points, metrics = mod.select_visibility_fallback(
        grid(),
        (1.55, -1.45),
        [],
        lambda x, y: (True, "CLEAR"),
    )

    assert len(points) == 2
    assert points[0] == pytest.approx((1.95, -1.05))
    assert points[1] == pytest.approx((2.25, -0.75))
    assert metrics["visible_boundary_cells"] == 8


def test_visited_viewpoint_is_excluded(monkeypatch):
    monkeypatch.setattr(mod, "boundary_visibility_audit", fake_audit)

    points, metrics = mod.select_visibility_fallback(
        grid(),
        (1.55, -1.45),
        [(1.95, -1.05)],
        lambda x, y: (True, "CLEAR"),
    )

    assert len(points) == 1
    assert points[0] == pytest.approx((2.25, -0.75))
    assert metrics["visited_rejections"] == 1


def test_clearance_guard_cannot_be_bypassed(monkeypatch):
    monkeypatch.setattr(mod, "boundary_visibility_audit", fake_audit)

    points, metrics = mod.select_visibility_fallback(
        grid(),
        (1.55, -1.45),
        [],
        lambda x, y: (False, "NOT_CLEAR"),
    )

    assert not points
    assert metrics["guard_rejections"] == 2


def test_outside_robot_seed_is_rejected():
    with pytest.raises(ValueError):
        mod.select_visibility_fallback(
            grid(),
            (-20.0, -20.0),
            [],
            lambda x, y: (True, "CLEAR"),
        )


def test_original_audit_supports_larger_candidate_budget():
    data = [100] * (32 * 32)

    for y in range(2, 30):
        for x in range(2, 30):
            data[y * 32 + x] = 0

    for y in range(12, 18):
        data[y * 32 + 27] = -1

    result = boundary_visibility_audit(
        data, 32, 32, 0.1, 14 * 32 + 10,
        sensor_range_m=4.0,
        clearance_m=0.2,
        stride_cells=2,
        max_candidates=32,
    )

    assert result["candidates"]
    assert len(result["candidates"]) <= 32


def test_invalid_candidate_budget_rejected():
    data = [0] * 100

    with pytest.raises(ValueError):
        boundary_visibility_audit(
            data, 10, 10, 0.1, 55,
            max_candidates=0,
        )
