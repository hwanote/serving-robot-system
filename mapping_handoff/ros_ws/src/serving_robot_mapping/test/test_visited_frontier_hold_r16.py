from types import SimpleNamespace
from unittest.mock import Mock

from serving_robot_mapping.exploration_manager import (
    ExplorationManager,
    MappingState,
)
from serving_robot_mapping.exploration_goal_manager import (
    VerifiedFrontierGoal,
)


def test_visited_only_candidates_hold_current_map_revision():
    manager = SimpleNamespace(
        state_machine=SimpleNamespace(
            state=MappingState.EXPLORING
        ),
        last_frontier_count=3,
        visited_frontiers=[(5.44, 4.55)],
        freshness_monitor=SimpleNamespace(map_revision=38),
        get_logger=lambda: Mock(),
    )

    goals = [
        VerifiedFrontierGoal(5.57, 4.49, 0.10),
        VerifiedFrontierGoal(5.34, 4.40, 0.19),
    ]

    ExplorationManager.planner_results_callback(manager, goals)

    assert manager._visited_only_hold_revision == 38
    assert manager.planner_verified_goal is None
    assert manager.state_machine.state == MappingState.EXPLORING


def test_planning_resumes_only_after_map_revision_change():
    planner = Mock()

    manager = SimpleNamespace(
        planning_enabled=True,
        state_machine=SimpleNamespace(
            state=MappingState.EXPLORING
        ),
        last_frontier_count=3,
        navigation_enabled=False,
        navigation_rearm_pending=False,
        _visited_only_hold_revision=38,
        freshness_monitor=SimpleNamespace(map_revision=38),
        planner_client=SimpleNamespace(tick=planner),
    )

    ExplorationManager.planning_loop(manager)
    planner.assert_not_called()

    manager.freshness_monitor.map_revision = 39

    ExplorationManager.planning_loop(manager)
    planner.assert_called_once()
