"""Regression tests for exhausted Frontier representative search."""

from types import SimpleNamespace
from unittest.mock import Mock

from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker

from serving_robot_mapping.exploration_manager import (
    ExplorationManager,
)
from serving_robot_mapping.mapping_state_machine import (
    MappingState,
    MappingStateMachine,
)


def make_case(visited):
    machine = MappingStateMachine()
    machine.transition_to(MappingState.EXPLORING)

    planner = SimpleNamespace(
        signature=(),
        update_candidates=Mock(),
    )

    guard = SimpleNamespace(
        grid=(None, None, None, 0.05, 0.0, 0.0, 0.0),
        observation_candidates=(
            lambda x, y, max_results=2: [(x, y)]
        ),
    )

    manager = SimpleNamespace(
        navigation_enabled=True,
        state_machine=machine,
        last_frontier_count=12,
        visited_frontiers=visited,
        frontier_clearance_guard=guard,
        planner_client=planner,
        get_logger=lambda: Mock(),
    )

    marker = Marker()
    marker.header.frame_id = "map"
    marker.action = Marker.ADD
    marker.type = Marker.POINTS

    marker.points = [
        Point(x=0.5 * i, y=1.0, z=0.0)
        for i in range(1, 13)
    ]

    ExplorationManager.mapping_update_candidates(
        manager, marker
    )

    forwarded = planner.update_candidates.call_args.args[0]

    return manager, forwarded


def test_searches_beyond_first_ten_visited_frontiers():
    visited = [
        (0.5 * i, 1.0)
        for i in range(1, 11)
    ]

    manager, marker = make_case(visited)

    assert marker.action == Marker.ADD
    assert len(marker.points) >= 1

    for point in marker.points:
        assert point.x > 5.0

    assert manager.state_machine.state == MappingState.EXPLORING


def test_all_exhausted_frontiers_remain_uncompleted():
    visited = [
        (0.5 * i, 1.0)
        for i in range(1, 13)
    ]

    manager, marker = make_case(visited)

    assert marker.action == Marker.DELETE
    assert not marker.points
    assert manager.state_machine.state != MappingState.MAP_COMPLETE
