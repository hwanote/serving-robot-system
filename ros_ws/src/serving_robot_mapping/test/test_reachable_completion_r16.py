"""Robot-connected completion verification regression tests."""

import pytest
from serving_robot_mapping.mapping_completion_verifier import (
    verify_reachable_closed_free_region as verify,
)


def enclosed():
    width = height = 12
    data = [100] * (width * height)
    for y in range(2, 10):
        for x in range(2, 10):
            data[y * width + x] = 0
    return data, width, height


def check(data, width=12, height=12, seed=6 * 12 + 6):
    return verify(
        data, width, height, 0.1, seed,
        min_free_area_m2=0.2,
    )


def test_enclosed_connected_free_area_passes():
    data, w, h = enclosed()
    passed, reason, metrics = check(data)
    assert passed
    assert reason == "REACHABLE_CLOSED_FREE_REGION_VERIFIED"
    assert metrics["reachable_free_cells"] == 64


def test_reachable_unknown_prevents_completion():
    data, w, h = enclosed()
    data[6 * w + 7] = -1
    passed, reason, metrics = check(data)
    assert not passed
    assert reason == "REACHABLE_FREE_TOUCHES_UNKNOWN"
    assert metrics["reachable_unknown_boundary_cells"] == 1


def test_disconnected_free_artifact_is_not_coverage():
    data, w, h = enclosed()
    data[0] = 0
    passed, reason, metrics = check(data)
    assert passed
    assert metrics["all_known_free_cells"] == 65
    assert metrics["reachable_free_cells"] == 64


def test_reachable_map_edge_prevents_completion():
    data, w, h = enclosed()
    for y in (1, 0):
        data[y * w + 6] = 0
    passed, reason, metrics = check(data)
    assert not passed
    assert reason == "REACHABLE_FREE_REACHES_MAP_EDGE"


def test_invalid_robot_seed_rejected():
    data, w, h = enclosed()
    passed, reason, _ = check(data, seed=0)
    assert not passed
    assert reason == "ROBOT_NOT_IN_KNOWN_FREE"


def test_tiny_connected_free_region_rejected():
    data = [100] * 144
    data[6 * 12 + 6] = 0
    passed, reason, _ = check(data)
    assert not passed
    assert reason == "INSUFFICIENT_REACHABLE_FREE_AREA"
