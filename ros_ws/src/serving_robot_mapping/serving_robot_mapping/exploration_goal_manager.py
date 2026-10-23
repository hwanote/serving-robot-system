"""Manage goal selection and state transitions for exploration.

This module never sends velocity or Nav2 navigation commands.
"""

import math
from dataclasses import dataclass
from typing import Optional, Sequence

from serving_robot_mapping.mapping_state_machine import (
    MappingState,
    MappingStateMachine,
)


@dataclass(frozen=True)
class VerifiedFrontierGoal:
    """A frontier goal with a successful Nav2 planning result."""

    x: float
    y: float
    path_length_m: float
    # R16_170B_ACTUAL_PLANNER_PATH_SCREEN: validated Nav2 path, not a straight line.
    path_points: tuple[tuple[float, float], ...] = ()

    def __post_init__(self) -> None:
        values = (
            self.x,
            self.y,
            self.path_length_m,
        )

        if not all(math.isfinite(v) for v in values):
            raise ValueError(
                "Goal coordinates and path length must be finite"
            )

        if self.path_length_m < 0:
            raise ValueError("Path length cannot be negative")


class ExplorationGoalManager:
    """Track selected goals, failures, and completion eligibility."""

    def __init__(
        self,
        state_machine: MappingStateMachine,
        no_frontier_cycles: int = 5,
        stable_time_s: float = 10.0,
        failure_radius_m: float = 0.25,
        max_goal_failures: int = 2,
    ) -> None:
        if no_frontier_cycles < 1 or max_goal_failures < 1:
            raise ValueError(
                "Cycle and failure limits must be positive"
            )

        if stable_time_s < 0 or failure_radius_m <= 0:
            raise ValueError(
                "Invalid stability time or failure radius"
            )

        self.state_machine = state_machine

        self.no_frontier_cycles = no_frontier_cycles
        self.stable_time_s = stable_time_s
        self.failure_radius_m = failure_radius_m
        self.max_goal_failures = max_goal_failures

        self.active_goal: Optional[VerifiedFrontierGoal] = None

        # Each entry: (x, y, failure_count)
        self.failed_goals: list[tuple[float, float, int]] = []

        self.empty_count = 0
        self.empty_since_s: Optional[float] = None

    def _reset_empty(self) -> None:
        self.empty_count = 0
        self.empty_since_s = None

    def _failure_count(
        self,
        goal: VerifiedFrontierGoal,
    ) -> int:
        return max(
            (
                attempts
                for x, y, attempts in self.failed_goals
                if math.hypot(
                    goal.x - x,
                    goal.y - y,
                ) <= self.failure_radius_m
            ),
            default=0,
        )

    def select_goal(
        self,
        verified_goals: Sequence[VerifiedFrontierGoal],
    ) -> Optional[VerifiedFrontierGoal]:
        """Choose the shortest verified goal not blacklisted."""

        if self.state_machine.state != MappingState.EXPLORING:
            return None

        eligible = [
            goal
            for goal in verified_goals
            if self._failure_count(goal) < self.max_goal_failures
        ]

        return min(
            eligible,
            key=lambda goal: goal.path_length_m,
            default=None,
        )

    def begin_navigation(
        self,
        goal: VerifiedFrontierGoal,
    ) -> None:
        """Record navigation start, without commanding movement."""

        if self.state_machine.state != MappingState.EXPLORING:
            raise RuntimeError(
                "Navigation can start only from EXPLORING"
            )

        if self._failure_count(goal) >= self.max_goal_failures:
            raise ValueError(
                "Goal excluded after repeated failures"
            )

        self.active_goal = goal
        self._reset_empty()

        self.state_machine.transition_to(
            MappingState.NAVIGATING
        )

    def navigation_succeeded(self) -> None:
        """Return to exploration after successful navigation."""

        if (
            self.state_machine.state != MappingState.NAVIGATING
            or self.active_goal is None
        ):
            raise RuntimeError("No navigation is active")

        self.active_goal = None
        self._reset_empty()

        self.state_machine.transition_to(
            MappingState.EXPLORING
        )

    def navigation_failed(self) -> None:
        """Record navigation failure and enter recovery."""

        if (
            self.state_machine.state != MappingState.NAVIGATING
            or self.active_goal is None
        ):
            raise RuntimeError("No navigation is active")

        goal = self.active_goal

        for index, (x, y, attempts) in enumerate(
            self.failed_goals
        ):
            distance = math.hypot(
                goal.x - x,
                goal.y - y,
            )

            if distance <= self.failure_radius_m:
                self.failed_goals[index] = (
                    x,
                    y,
                    attempts + 1,
                )
                break
        else:
            self.failed_goals.append(
                (goal.x, goal.y, 1)
            )

        self.active_goal = None
        self._reset_empty()

        self.state_machine.transition_to(
            MappingState.RECOVERY
        )

    def recovery_finished(self) -> None:
        """Resume frontier selection after external recovery."""

        if self.state_machine.state != MappingState.RECOVERY:
            raise RuntimeError(
                "Recovery can finish only from RECOVERY"
            )

        self.state_machine.transition_to(
            MappingState.EXPLORING
        )

    def clear_failed_goals(self) -> None:
        """Reset failure history after a meaningful map change."""

        self.failed_goals.clear()

    def observe_frontiers(
        self,
        frontier_count: int,
        now_s: float,
    ) -> MappingState:
        """Check whether zero frontiers persisted long enough.

        A zero-frontier observation does not prove completion.
        """

        if frontier_count < 0 or not math.isfinite(now_s):
            raise ValueError("Invalid frontier observation")

        state = self.state_machine.state

        if state not in (
            MappingState.EXPLORING,
            MappingState.COMPLETION_CHECK,
        ):
            return state

        # Any new frontier cancels completion checking.
        if frontier_count > 0:
            self._reset_empty()

            if state == MappingState.COMPLETION_CHECK:
                self.state_machine.transition_to(
                    MappingState.EXPLORING
                )

            return self.state_machine.state

        # Begin or restart the zero-frontier observation window.
        if (
            self.empty_since_s is None
            or now_s < self.empty_since_s
        ):
            self.empty_since_s = now_s
            self.empty_count = 0

        self.empty_count += 1

        elapsed = now_s - self.empty_since_s

        if (
            self.empty_count >= self.no_frontier_cycles
            and elapsed >= self.stable_time_s
        ):
            self.state_machine.transition_to(
                MappingState.COMPLETION_CHECK
            )

        return self.state_machine.state

    def confirm_map_complete(
        self,
        independently_verified: bool,
    ) -> bool:
        """Require external verification before MAP_COMPLETE."""

        if (
            self.state_machine.state
            != MappingState.COMPLETION_CHECK
        ):
            return False

        if not independently_verified:
            return False

        self.state_machine.transition_to(
            MappingState.MAP_COMPLETE
        )

        return True
