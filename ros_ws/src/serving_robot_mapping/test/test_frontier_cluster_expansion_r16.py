"""Regression tests for expanded Frontier cluster sampling."""

import math
from types import SimpleNamespace

from serving_robot_mapping.frontier_marker_adapter import (
    frontier_representatives,
)


def points_in_row(count, y):
    return [
        SimpleNamespace(x=0.025 + 0.05 * i, y=y)
        for i in range(count)
    ]


def sample(points, limit):
    return frontier_representatives(
        points,
        resolution=0.05,
        origin_x=0.0,
        origin_y=0.0,
        origin_yaw=0.0,
        limit=limit,
    )


def test_fast_mode_preserves_two_representatives():
    result = sample(points_in_row(35, 0.025), 10)
    assert len(result) == 2


def test_extended_mode_covers_entire_cluster():
    result = sample(points_in_row(35, 0.025), 64)
    assert len(result) == 35
    assert len(set(result)) == 35


def test_extended_mode_finds_new_unvisited_locations():
    points = points_in_row(35, 0.025)
    visited = sample(points, 10)
    expanded = sample(points, 64)

    unvisited = [
        xy for xy in expanded
        if all(
            math.hypot(
                xy[0] - old[0],
                xy[1] - old[1],
            ) > 0.25
            for old in visited
        )
    ]

    assert unvisited


def test_multiple_clusters_are_sampled():
    points = (
        points_in_row(20, 0.025)
        + points_in_row(20, 2.025)
    )

    result = sample(points, 30)

    assert len(result) == 30
    assert len(set(result)) == 30
    assert sum(y < 1 for _, y in result) > 2
    assert sum(y > 1 for _, y in result) > 2


def test_expansion_respects_limit():
    result = sample(points_in_row(100, 0.025), 64)

    assert len(result) == 64
    assert len(set(result)) == 64
