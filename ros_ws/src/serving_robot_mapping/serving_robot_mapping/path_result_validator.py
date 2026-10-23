"""Validate Nav2 ComputePathToPose action results.

This module does not command robot motion.
"""

import math
from dataclasses import dataclass
from typing import Optional

from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import ComputePathToPose

from serving_robot_mapping.exploration_goal_manager import (
    VerifiedFrontierGoal,
)


@dataclass(frozen=True)
class PathValidationOutcome:
    """Result of checking a Nav2 planner response."""

    valid: bool
    reason: str
    goal: Optional[VerifiedFrontierGoal] = None


class PathResultValidator:
    """Check planner status, geometry, and requested destination."""

    def __init__(
        self,
        goal_tolerance_m: float = 0.15,
        start_tolerance_m: float = 0.50,
    ) -> None:
        if goal_tolerance_m < 0 or start_tolerance_m < 0:
            raise ValueError("Tolerances cannot be negative")

        self.goal_tolerance_m = goal_tolerance_m
        self.start_tolerance_m = start_tolerance_m

    def validate(
        self,
        requested_goal: PoseStamped,
        status: int,
        result: ComputePathToPose.Result,
        expected_frame: str = "map",
        request_start_xy: Optional[tuple[float, float]] = None,
    ) -> PathValidationOutcome:
        """Validate a result from a matching Nav2 action request.

        This method must be called from the action result callback,
        not from an arbitrary Path topic subscription.
        """

        def reject(reason: str) -> PathValidationOutcome:
            return PathValidationOutcome(
                valid=False,
                reason=reason,
            )

        if status != GoalStatus.STATUS_SUCCEEDED:
            return reject(
                f"Action did not succeed: status={status}"
            )

        if result.error_code != ComputePathToPose.Result.NONE:
            return reject(
                f"Planner error: code={result.error_code}, "
                f"message={result.error_msg}"
            )

        if requested_goal.header.frame_id != expected_frame:
            return reject("Requested goal frame mismatch")

        gx = requested_goal.pose.position.x
        gy = requested_goal.pose.position.y

        if not all(math.isfinite(v) for v in (gx, gy)):
            return reject("Non-finite requested goal")

        path = result.path

        if path.header.frame_id != expected_frame:
            return reject("Path frame mismatch")

        # This workflow expects a route with at least two poses.
        # Already-at-goal cases will be handled separately.
        if len(path.poses) < 2:
            return reject("Path contains fewer than two poses")

        for pose in path.poses:
            frame = pose.header.frame_id

            if frame and frame != expected_frame:
                return reject("Path pose frame mismatch")

            point = pose.pose.position

            if not all(
                math.isfinite(v)
                for v in (point.x, point.y)
            ):
                return reject("Path contains non-finite coordinates")

        # The planned destination must match the requested goal.
        endpoint = path.poses[-1].pose.position

        goal_error = math.hypot(
            endpoint.x - gx,
            endpoint.y - gy,
        )

        if goal_error > self.goal_tolerance_m:
            return reject(
                f"Path endpoint mismatch: {goal_error:.3f} m"
            )

        # Optionally compare with robot pose at request time.
        if request_start_xy is not None:
            sx, sy = request_start_xy

            if not all(math.isfinite(v) for v in (sx, sy)):
                return reject("Invalid request start position")

            first = path.poses[0].pose.position

            start_error = math.hypot(
                first.x - sx,
                first.y - sy,
            )

            if start_error > self.start_tolerance_m:
                return reject(
                    f"Path start mismatch: {start_error:.3f} m"
                )

        length = 0.0

        for previous, current in zip(
            path.poses,
            path.poses[1:],
        ):
            a = previous.pose.position
            b = current.pose.position

            length += math.hypot(
                b.x - a.x,
                b.y - a.y,
            )

        if not math.isfinite(length) or length <= 0:
            return reject("Invalid path length")

        verified_goal = VerifiedFrontierGoal(
            x=gx,
            y=gy,
            path_length_m=length,
            # R16_170B_ACTUAL_PLANNER_PATH_SCREEN
            path_points=tuple(
                (pose.pose.position.x, pose.pose.position.y)
                for pose in path.poses
            ),
        )

        return PathValidationOutcome(
            valid=True,
            reason="Planner result validated",
            goal=verified_goal,
        )
