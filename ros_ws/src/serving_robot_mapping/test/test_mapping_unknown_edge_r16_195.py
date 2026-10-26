"""R16-195 Unknown, uncertain and map-edge safety regressions."""

import math

import pytest

from serving_robot_mapping.mapping_frontier_clearance import (
    FrontierClearanceGuard,
)
from serving_robot_mapping.mapping_path_risk import (
    MappingPathRiskGuard,
)

W = 60
H = 60
RES = 0.05


def create_guards(data, yaw=0.0, ox=0.0, oy=0.0):
    goal = FrontierClearanceGuard(
        goal_clearance_m=0.52,
        start_clearance_m=0.50,
    )
    path = MappingPathRiskGuard(
        occupied_clearance_m=0.48,
    )
    assert goal.update_map(data, W, H, RES, ox, oy, yaw)
    assert path.update_grid(data, W, H, RES, ox, oy, yaw)
    return goal, path


def test_safe_interior_remains_allowed():
    goal, path = create_guards([0] * (W * H))
    assert goal.goal_clear(1.5, 1.5)[0]
    assert path.check_path(
        [(1.45, 1.5), (1.55, 1.5)]
    ).allowed


def test_goal_and_actual_path_near_edge_blocked():
    goal, path = create_guards([0] * (W * H))
    assert goal.goal_clear(0.30, 1.5) == (
        False, "GOAL_NEAR_MAP_EDGE"
    )
    assert path.check_path(
        [(0.30, 1.5), (0.35, 1.5)]
    ).reason == "PATH_NEAR_MAP_EDGE"


@pytest.mark.parametrize(
    "value,goal_reason,path_reason",
    [
        (-1, "GOAL_NEAR_UNKNOWN_CELL", "PATH_NEAR_UNKNOWN"),
        (50, "GOAL_NEAR_UNCERTAIN_CELL", "PATH_NEAR_UNCERTAIN"),
        (100, "GOAL_NEAR_OCCUPIED_CELL", "PATH_NEAR_OCCUPIED"),
    ],
)
def test_nonfree_neighbor_blocks_envelope(
    value, goal_reason, path_reason,
):
    data = [0] * (W * H)
    data[30 * W + 38] = value
    goal, path = create_guards(data)

    assert goal.goal_clear(1.5, 1.5) == (
        False, goal_reason
    )
    result = path.check_path(
        [(1.45, 1.5), (1.55, 1.5)]
    )
    assert not result.allowed
    assert result.reason == path_reason


def test_unknown_is_not_reported_as_occupied_distance():
    data = [0] * (W * H)
    data[30 * W + 38] = -1
    _, path = create_guards(data)

    result = path.check_path(
        [(1.45, 1.5), (1.55, 1.5)]
    )
    assert result.reason == "PATH_NEAR_UNKNOWN"
    assert result.minimum_occupied_distance_m is None


def test_unknown_center_still_blocked():
    data = [0] * (W * H)
    data[30 * W + 30] = -1
    goal, path = create_guards(data)

    assert goal.goal_clear(1.525, 1.525)[1] == (
        "GOAL_NOT_KNOWN_FREE"
    )
    assert path.check_path(
        [(1.525, 1.525), (1.55, 1.525)]
    ).reason == "PATH_CENTER_NOT_KNOWN_FREE"


def test_outside_map_still_blocked():
    goal, path = create_guards([0] * (W * H))

    assert goal.goal_clear(-0.01, 1.5)[1] == (
        "GOAL_OUTSIDE_MAP"
    )
    assert path.check_path(
        [(-0.01, 1.5), (1.5, 1.5)]
    ).reason == "PATH_OUTSIDE_MAP"


def test_rotated_map_edge_blocked():
    ox, oy = 5.0, -1.0
    yaw = math.pi / 2
    goal, path = create_guards(
        [0] * (W * H), yaw, ox, oy
    )

    def xy(lx, ly):
        return (ox - ly, oy + lx)

    assert goal.goal_clear(*xy(1.5, 1.5))[0]
    assert goal.goal_clear(*xy(0.30, 1.5))[1] == (
        "GOAL_NEAR_MAP_EDGE"
    )
    assert path.check_path([
        xy(0.30, 1.5),
        xy(0.35, 1.5),
    ]).reason == "PATH_NEAR_MAP_EDGE"


def test_missing_planner_path_still_blocked():
    _, path = create_guards([0] * (W * H))
    assert not path.check_path(None).allowed
    assert not path.check_path([(1.5, 1.5)]).allowed
    assert not path.check_path([
        (float("nan"), 1.5), (1.6, 1.5)
    ]).allowed
