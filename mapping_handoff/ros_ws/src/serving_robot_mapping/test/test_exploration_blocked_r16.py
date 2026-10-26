import time
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


def make_case(candidates):
    machine = MappingStateMachine()
    machine.transition_to(MappingState.EXPLORING)

    planner = SimpleNamespace(
        signature=(),
        update_candidates=Mock(),
    )

    guard = SimpleNamespace(
        grid=("map_ready",),
        observation_candidates=(
            lambda x, y, max_results=2: list(candidates)
        ),
    )

    manager = SimpleNamespace(
        state_machine=machine,
        navigation_enabled=True,
        last_frontier_count=3,
        visited_frontiers=[],
        frontier_clearance_guard=guard,
        planner_client=planner,
        get_logger=lambda: Mock(),
    )

    marker = Marker()
    marker.header.frame_id = "map"
    marker.action = Marker.ADD
    marker.type = Marker.SPHERE_LIST
    marker.points = [Point(x=1.0, y=1.0, z=0.0)]

    return manager, marker


def test_persistent_absence_of_goals_blocks_without_completion():
    manager, marker = make_case([])

    manager._no_viable_since = time.monotonic() - 11.0

    ExplorationManager.mapping_update_candidates(
        manager, marker
    )

    assert (
        manager.state_machine.state
        == MappingState.EXPLORATION_BLOCKED
    )
    assert (
        manager.state_machine.state
        != MappingState.MAP_COMPLETE
    )

    sent = manager.planner_client.update_candidates.call_args.args[0]
    assert sent.action == Marker.DELETE


def test_reachable_goal_resumes_exploration():
    manager, marker = make_case([(2.0, 3.0)])

    manager.state_machine.transition_to(
        MappingState.EXPLORATION_BLOCKED
    )

    ExplorationManager.mapping_update_candidates(
        manager, marker
    )

    assert manager.state_machine.state == MappingState.EXPLORING

    sent = manager.planner_client.update_candidates.call_args.args[0]
    assert sent.action == Marker.ADD
    assert len(sent.points) == 1


def test_transient_empty_candidates_do_not_block():
    manager, marker = make_case([])

    ExplorationManager.mapping_update_candidates(
        manager, marker
    )

    assert manager.state_machine.state == MappingState.EXPLORING


def test_navigation_state_not_overridden():
    manager, marker = make_case([])

    manager.state_machine.transition_to(
        MappingState.NAVIGATING
    )
    manager._no_viable_since = time.monotonic() - 11.0

    ExplorationManager.mapping_update_candidates(
        manager, marker
    )

    assert manager.state_machine.state == MappingState.NAVIGATING
