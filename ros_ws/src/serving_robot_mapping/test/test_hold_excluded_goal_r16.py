"""A goal canceled by OBSTACLE_HOLD_NOT_CLEAR is not picked again."""

from types import SimpleNamespace
from unittest.mock import Mock

from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker

from serving_robot_mapping.exploration_goal_manager import (
    ExplorationGoalManager,
    VerifiedFrontierGoal,
)
from serving_robot_mapping.exploration_manager import (
    HOLD_EXCLUDE_MAX,
    HOLD_EXCLUDE_RADIUS_M,
    ExplorationManager,
    near_hold_excluded,
)
from serving_robot_mapping.mapping_state_machine import (
    MappingState,
    MappingStateMachine,
)

HOLD = (
    "Safety check invalid: OBSTACLE_HOLD_NOT_CLEAR; "
    "terminal status=5; cancel response=ACCEPTED"
)
TF = (
    "Safety check invalid: TF_NOT_STABLE; "
    "terminal status=5; cancel response=ACCEPTED"
)


def make_manager():
    m = SimpleNamespace()
    m.state_machine = MappingStateMachine()
    m.state_machine.transition_to(MappingState.EXPLORING)
    m.goal_manager = ExplorationGoalManager(state_machine=m.state_machine)
    m.last_frontier_count = 3
    m.planner_verified_goal = None
    m.visited_frontiers = []
    m.navigation_recovery_wait_s = 3.0
    m.navigation_recovery_until = 0.0
    m.request_navigation_replan = Mock()
    m.warn = Mock()
    m.get_logger = lambda: SimpleNamespace(
        info=Mock(), warn=m.warn, error=Mock()
    )
    return m


def cancel(m, goal, reason):
    m.goal_manager.begin_navigation(goal)
    ExplorationManager.navigation_finished(m, goal, "CANCELED", reason)


def test_hold_cancel_excludes_the_goal_and_still_recovers():
    m = make_manager()
    goal = VerifiedFrontierGoal(x=5.66, y=5.27, path_length_m=3.0)

    cancel(m, goal, HOLD)

    assert m.hold_excluded_goals == [(5.66, 5.27)]
    assert m.state_machine.state == MappingState.RECOVERY
    messages = [c.args[0] for c in m.warn.call_args_list]
    assert any("is excluded" in text for text in messages)


def test_other_transient_cancel_does_not_exclude():
    m = make_manager()
    goal = VerifiedFrontierGoal(x=1.0, y=1.0, path_length_m=1.0)

    cancel(m, goal, TF)

    assert getattr(m, "hold_excluded_goals", []) == []
    assert m.state_machine.state == MappingState.RECOVERY


def test_exclusion_list_is_capped():
    m = make_manager()
    m.hold_excluded_goals = [(float(i), 0.0) for i in range(HOLD_EXCLUDE_MAX)]
    goal = VerifiedFrontierGoal(x=99.0, y=99.0, path_length_m=1.0)

    cancel(m, goal, HOLD)

    assert len(m.hold_excluded_goals) == HOLD_EXCLUDE_MAX
    assert m.hold_excluded_goals[-1] == (99.0, 99.0)
    assert m.hold_excluded_goals[0] == (1.0, 0.0)


def test_near_hold_excluded_uses_the_radius():
    r = HOLD_EXCLUDE_RADIUS_M
    assert near_hold_excluded(1.0, 1.0, [(1.0 + r - 0.01, 1.0)])
    assert not near_hold_excluded(1.0, 1.0, [(1.0 + r + 0.05, 1.0)])
    assert not near_hold_excluded(1.0, 1.0, [])


def test_verified_goal_near_an_excluded_one_is_not_dispatched():
    m = make_manager()
    m.hold_excluded_goals = [(5.66, 5.27)]
    m.freshness_monitor = SimpleNamespace(map_revision=7)
    m.navigation_enabled = True
    m.navigation_client = SimpleNamespace(start=Mock(), busy=False)

    ExplorationManager.planner_results_callback(
        m, [VerifiedFrontierGoal(5.5, 5.1, 2.0)]
    )

    m.navigation_client.start.assert_not_called()
    assert m.planner_verified_goal is None
    assert m.state_machine.state == MappingState.EXPLORING


def make_candidates(excluded):
    machine = MappingStateMachine()
    machine.transition_to(MappingState.EXPLORING)
    planner = SimpleNamespace(signature=(), update_candidates=Mock())
    guard = SimpleNamespace(
        grid=(None, None, None, 0.05, 0.0, 0.0, 0.0),
        observation_candidates=lambda x, y, max_results=2: [(x, y)],
    )
    manager = SimpleNamespace(
        navigation_enabled=True,
        state_machine=machine,
        last_frontier_count=12,
        visited_frontiers=[],
        hold_excluded_goals=excluded,
        frontier_clearance_guard=guard,
        planner_client=planner,
        get_logger=lambda: Mock(),
    )
    marker = Marker()
    marker.header.frame_id = "map"
    marker.action = Marker.ADD
    marker.type = Marker.POINTS
    marker.points = [
        Point(x=0.5 * i, y=1.0, z=0.0) for i in range(1, 13)
    ]
    ExplorationManager.mapping_update_candidates(manager, marker)
    return planner.update_candidates.call_args.args[0]


def test_candidates_skip_viewpoints_near_an_excluded_goal():
    forwarded = make_candidates([(2.0, 1.0)])

    assert len(forwarded.points) >= 1
    for point in forwarded.points:
        assert not near_hold_excluded(point.x, point.y, [(2.0, 1.0)])
    assert any(p.x > 3.0 for p in forwarded.points)


def test_candidates_unchanged_without_exclusions():
    forwarded = make_candidates([])

    assert any(abs(p.x - 2.0) < 0.3 for p in forwarded.points)
