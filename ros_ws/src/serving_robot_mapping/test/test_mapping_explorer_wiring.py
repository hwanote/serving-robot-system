"""Offline Explorer-to-Nav2 state transition tests."""

import unittest
from types import SimpleNamespace
from unittest.mock import Mock

from serving_robot_mapping.exploration_manager import (
    ExplorationManager,
)
from serving_robot_mapping.exploration_goal_manager import (
    ExplorationGoalManager,
    VerifiedFrontierGoal,
)
from serving_robot_mapping.mapping_state_machine import (
    MappingState,
    MappingStateMachine,
)


class ExplorerWiringTests(unittest.TestCase):

    def setUp(self):
        self.goal = VerifiedFrontierGoal(
            x=0.42,
            y=0.03,
            path_length_m=0.44,
        )

        self.manager = SimpleNamespace()
        m = self.manager

        m.state_machine = MappingStateMachine()
        m.state_machine.transition_to(
            MappingState.EXPLORING
        )

        m.goal_manager = ExplorationGoalManager(
            state_machine=m.state_machine,
        )
        m.last_frontier_count = 3
        m.planner_verified_goal = None
        m.visited_frontiers = []
        m.navigation_enabled = True
        m.navigation_max_goals = 1
        m.navigation_sent = 0
        m.navigation_recovery_wait_s = 3.0
        m.navigation_recovery_until = 0.0

        m.navigation_monitor = SimpleNamespace(
            is_safe=Mock(return_value=True)
        )
        m.navigation_client = SimpleNamespace(
            start=Mock(return_value=True),
            busy=False,
        )
        m.request_navigation_replan = Mock()

        m.get_logger = lambda: SimpleNamespace(
            info=Mock(),
            warn=Mock(),
            error=Mock(),
        )

    def test_verified_goal_starts_navigation(self):
        m = self.manager

        ExplorationManager.planner_results_callback(
            m, [self.goal]
        )

        m.navigation_client.start.assert_called_once_with(
            self.goal
        )
        self.assertEqual(
            m.state_machine.state,
            MappingState.NAVIGATING,
        )
        self.assertEqual(m.navigation_sent, 1)

    def test_unsafe_goal_does_not_dispatch(self):
        m = self.manager
        m.navigation_monitor.is_safe.return_value = False

        ExplorationManager.planner_results_callback(
            m, [self.goal]
        )

        m.navigation_client.start.assert_not_called()
        self.assertEqual(m.navigation_sent, 0)
        self.assertEqual(
            m.state_machine.state,
            MappingState.EXPLORING,
        )

    def test_goal_limit_blocks_second_dispatch(self):
        m = self.manager
        m.navigation_sent = 1

        ExplorationManager.planner_results_callback(
            m, [self.goal]
        )

        m.navigation_client.start.assert_not_called()

    def test_success_marks_frontier_visited(self):
        m = self.manager
        m.goal_manager.begin_navigation(self.goal)

        ExplorationManager.navigation_finished(
            m,
            self.goal,
            "SUCCEEDED",
            "Arrival",
        )

        self.assertEqual(
            m.state_machine.state,
            MappingState.EXPLORING,
        )
        self.assertEqual(
            m.visited_frontiers,
            [(0.42, 0.03)],
        )

    def test_safety_cancel_halts_explorer(self):
        m = self.manager
        m.goal_manager.begin_navigation(self.goal)

        ExplorationManager.navigation_finished(
            m,
            self.goal,
            "CANCELED",
            "Safety check invalid",
        )

        self.assertEqual(
            m.state_machine.state,
            MappingState.ERROR,
        )

    def test_route_failure_enters_recovery(self):
        m = self.manager
        m.goal_manager.begin_navigation(self.goal)

        ExplorationManager.navigation_finished(
            m,
            self.goal,
            "FAILED",
            "Nav2 result status=6",
        )

        self.assertEqual(
            m.state_machine.state,
            MappingState.RECOVERY,
        )

    def test_transient_cancel_enters_recovery(self):
        m = self.manager
        m.goal_manager.begin_navigation(self.goal)

        ExplorationManager.navigation_finished(
            m,
            self.goal,
            "CANCELED",
            "Safety check invalid: PERSON_LIMIT_STALE; "
            "terminal status=5; cancel response=ACCEPTED",
        )

        self.assertEqual(
            m.state_machine.state,
            MappingState.RECOVERY,
        )

    def test_collision_cancel_requires_manual_restart(self):
        m = self.manager
        m.goal_manager.begin_navigation(self.goal)

        ExplorationManager.navigation_finished(
            m,
            self.goal,
            "CANCELED",
            "Safety check invalid: COLLISION_STATE_NOT_CLEAR; "
            "terminal status=5; cancel response=ACCEPTED",
        )

        self.assertEqual(
            m.state_machine.state,
            MappingState.ERROR,
        )

    def test_unconfirmed_cancel_requires_manual_restart(self):
        m = self.manager
        m.goal_manager.begin_navigation(self.goal)

        ExplorationManager.navigation_finished(
            m,
            self.goal,
            "CANCELED",
            "Safety check invalid: HEALTH_STALE; "
            "terminal status=5; cancel response=REJECTED",
        )

        self.assertEqual(
            m.state_machine.state,
            MappingState.ERROR,
        )

    def test_timeout_failure_blocks_automatic_recovery(self):
        m = self.manager
        m.goal_manager.begin_navigation(self.goal)

        ExplorationManager.navigation_finished(
            m,
            self.goal,
            "FAILED",
            "Navigation timeout; terminal status=6; "
            "cancel response=ACCEPTED",
        )

        self.assertEqual(
            m.state_machine.state,
            MappingState.ERROR,
        )


if __name__ == "__main__":
    unittest.main(verbosity=2)
