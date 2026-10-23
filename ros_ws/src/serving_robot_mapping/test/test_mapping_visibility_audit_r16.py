"""Visibility-audit regression tests."""

import pytest

from serving_robot_mapping.mapping_visibility_audit import (
    boundary_visibility_audit,
)


def room():
    w = h = 32
    data = [100] * (w * h)

    for y in range(2, 30):
        for x in range(2, 30):
            data[y * w + x] = 0

    return data, w, h


def run(data, seed=14 * 32 + 10):
    return boundary_visibility_audit(
        data, 32, 32, 0.1, seed,
        sensor_range_m=4.0,
        clearance_m=0.2,
        stride_cells=2,
    )


def test_fully_mapped_room_has_no_unknown_boundary():
    data, _, _ = room()
    result = run(data)
    assert result["unknown_boundary_cells"] == 0


def test_open_unknown_patch_has_visible_candidates():
    data, w, _ = room()

    for y in range(12, 18):
        data[y * w + 27] = -1

    result = run(data)
    assert result["unknown_boundary_cells"] > 0
    assert result["visible_boundary_cells"] > 0
    assert result["candidates"]


def test_unknown_behind_wall_is_not_reachable_boundary():
    data, w, _ = room()

    for y in range(12, 18):
        for x in range(12, 18):
            data[y * w + x] = 100

    for y in range(14, 16):
        for x in range(14, 16):
            data[y * w + x] = -1

    result = run(data, seed=8 * w + 8)
    assert result["unknown_boundary_cells"] == 0


def test_invalid_seed_rejected():
    data, _, _ = room()

    with pytest.raises(ValueError):
        run(data, seed=0)


def test_candidates_remain_known_free():
    data, w, _ = room()

    for y in range(12, 18):
        data[y * w + 27] = -1

    result = run(data)

    assert all(
        data[y * w + x] == 0
        for _, x, y in result["candidates"]
    )
