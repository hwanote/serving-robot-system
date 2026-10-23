"""Dense fallback must retain every safety restriction."""

import pytest

import serving_robot_mapping.mapping_visibility_fallback as mod


def grid():
    return ([0] * 400, 20, 20, 0.1, 1.0, -2.0, 0.0)


def result(candidates, unknown=12):
    return {
        "status": "VISIBLE_OBSERVATION_LOCATIONS_EXIST",
        "unknown_boundary_cells": unknown,
        "visible_boundary_cells": 4,
        "sampled_viewpoints": 20,
        "scored_viewpoints": 5,
        "candidates": candidates,
    }


def select(guard=None, visited=()):
    if guard is None:
        guard = lambda x, y: (True, "GOAL_CLEAR")

    return mod.select_visibility_fallback(
        grid=grid(),
        robot_xy=(1.55, -1.45),
        visited=list(visited),
        goal_clear=guard,
        max_results=8,
    )


def test_dense_pass_finds_sparse_missed_candidate(monkeypatch):
    calls = []

    def fake(*args, **kwargs):
        stride = kwargs["stride_cells"]
        calls.append(stride)

        if stride == 4:
            return result([])

        assert stride == 1
        assert kwargs["viewpoint_filter"](12, 12)
        return result([(2, 12, 12)])

    monkeypatch.setattr(
        mod, "boundary_visibility_audit", fake
    )

    points, metrics = select()

    assert calls == [4, 1]
    assert points == pytest.approx([(2.25, -0.75)])
    assert metrics["viewpoint_stride_cells"] == 1


def test_sparse_success_skips_dense_scan(monkeypatch):
    calls = []

    def fake(*args, **kwargs):
        calls.append(kwargs["stride_cells"])
        return result([(3, 12, 12)])

    monkeypatch.setattr(
        mod, "boundary_visibility_audit", fake
    )

    points, metrics = select()

    assert calls == [4]
    assert len(points) == 1
    assert metrics["viewpoint_stride_cells"] == 4


def test_dense_pass_cannot_bypass_clearance(monkeypatch):
    def fake(*args, **kwargs):
        if kwargs["stride_cells"] == 4:
            return result([])

        assert not kwargs["viewpoint_filter"](12, 12)
        return result([])

    monkeypatch.setattr(
        mod, "boundary_visibility_audit", fake
    )

    points, metrics = select(
        lambda x, y: (False, "GOAL_NEAR_OCCUPIED_CELL")
    )

    assert not points
    assert metrics["guard_rejections"] == 1
    assert metrics["guard_rejection_reasons"] == {
        "GOAL_NEAR_OCCUPIED_CELL": 1
    }


def test_dense_pass_excludes_visited_positions(monkeypatch):
    def fake(*args, **kwargs):
        if kwargs["stride_cells"] == 4:
            return result([])

        assert not kwargs["viewpoint_filter"](12, 12)
        return result([])

    monkeypatch.setattr(
        mod, "boundary_visibility_audit", fake
    )

    points, metrics = select(
        visited=[(2.25, -0.75)]
    )

    assert not points
    assert metrics["visited_rejections"] == 1


def test_no_unknown_boundary_skips_dense(monkeypatch):
    calls = []

    def fake(*args, **kwargs):
        calls.append(kwargs["stride_cells"])
        return result([], unknown=0)

    monkeypatch.setattr(
        mod, "boundary_visibility_audit", fake
    )

    points, metrics = select()

    assert not points
    assert calls == [4]
    assert metrics["viewpoint_stride_cells"] == 4
