"""Opt-in Nav2 navigation for frontier mapping.

No cmd_vel publisher. Navigation requires explicit enablement
and a live safety check supplied by ExplorationManager.
"""

import math
import time

from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient

from serving_robot_mapping.exploration_goal_manager import (
    VerifiedFrontierGoal,
)


class FrontierNavigationClient:
    """Manage one asynchronous NavigateToPose request."""

    def __init__(
        self,
        node,
        *,
        action_name="/navigate_to_pose",
        enabled=False,
        safety_check=None,
        safety_reason=None,
        on_finished=None,
        timeout_s=120.0,
    ):
        if not math.isfinite(timeout_s) or timeout_s <= 0:
            raise ValueError("Invalid navigation timeout")

        self.node = node
        self.client = ActionClient(
            node,
            NavigateToPose,
            action_name,
        )

        # Fail closed: navigation disabled by default.
        self.enabled = bool(enabled)
        self.safety_check = safety_check
        self.safety_reason = safety_reason
        self.on_finished = on_finished
        self.timeout_s = timeout_s

        self.phase = "IDLE"
        self.goal = None
        self.handle = None
        self.cancel_reason = None
        self.cancel_ack = None
        self.deadline = None
        self.token = 0

    @property
    def busy(self):
        return self.phase != "IDLE"

    def _allowed(self):
        """Missing or failing safety evidence blocks movement."""
        if not self.enabled or self.safety_check is None:
            return False

        try:
            return self.safety_check() is True
        except Exception as exc:
            self.node.get_logger().error(
                f"Navigation safety check failed: {exc}"
            )
            return False

    def _safety_reason(self):
        if self.safety_reason is None:
            return "UNKNOWN"

        try:
            value = self.safety_reason()
            if isinstance(value, str) and value:
                return value
        except Exception:
            pass

        return "UNKNOWN"

    def start(self, goal):
        """Request movement only for an approved goal type."""
        if not isinstance(goal, VerifiedFrontierGoal):
            return False

        if self.busy or not self._allowed():
            return False

        if not self.client.server_is_ready():
            return False

        if not (
            math.isfinite(goal.x)
            and math.isfinite(goal.y)
        ):
            return False

        pose = PoseStamped()
        pose.header.frame_id = "map"
        pose.header.stamp = (
            self.node.get_clock().now().to_msg()
        )

        pose.pose.position.x = goal.x
        pose.pose.position.y = goal.y
        pose.pose.orientation.w = 1.0

        request = NavigateToPose.Goal()
        request.pose = pose

        self.token += 1
        token = self.token

        self.phase = "WAIT_ACCEPT"
        self.goal = goal
        self.handle = None
        self.cancel_reason = None
        self.cancel_ack = None
        self.deadline = (
            time.monotonic() + self.timeout_s
        )

        try:
            future = self.client.send_goal_async(request)
            future.add_done_callback(
                lambda fut: self._accepted(token, fut)
            )
        except Exception as exc:
            self._finish(
                "ERROR",
                f"Goal dispatch failed: {exc}",
            )
            return False

        return True

    def _accepted(self, token, future):
        try:
            handle = future.result()
        except Exception as exc:
            if token == self.token and self.busy:
                self._finish(
                    "ERROR",
                    f"Goal acceptance failed: {exc}",
                )
            return

        if token != self.token or not self.busy:
            if handle is not None and handle.accepted:
                handle.cancel_goal_async()
            return

        if handle is None or not handle.accepted:
            self._finish(
                "REJECTED",
                "Nav2 rejected frontier goal",
            )
            return

        self.handle = handle
        self.phase = "MOVING"

        try:
            result_future = handle.get_result_async()
            result_future.add_done_callback(
                lambda fut: self._result(token, fut)
            )
        except Exception as exc:
            self.cancel("Result monitoring failed")
            self.node.get_logger().error(
                f"Nav2 result setup failed: {exc}"
            )
            return

        if self.cancel_reason or not self._allowed():
            self.cancel(
                self.cancel_reason
                or "Safety check invalid: "
                + self._safety_reason()
            )
            self._send_cancel()

    def _send_cancel(self):
        if self.phase == "CANCELLING":
            return

        if self.handle is None:
            return

        self.phase = "CANCELLING"
        self.cancel_ack = "PENDING"
        token = self.token

        try:
            future = self.handle.cancel_goal_async()
            future.add_done_callback(
                lambda fut: self._cancel_response(token, fut)
            )
        except Exception as exc:
            self.cancel_ack = "FAILED"
            self.node.get_logger().error(
                f"Nav2 cancel request failed: {exc}"
            )

    def _cancel_response(self, token, future):
        """Observe cancel acknowledgement, not physical stopping."""
        if token != self.token or not self.busy:
            return

        try:
            response = future.result()
            accepted = bool(response.goals_canceling)
        except Exception as exc:
            self.cancel_ack = "FAILED"
            self.node.get_logger().error(
                f"Nav2 cancel response unavailable: {exc}"
            )
            return

        if accepted:
            self.cancel_ack = "ACCEPTED"
        else:
            self.cancel_ack = "REJECTED"
            self.node.get_logger().error(
                "Nav2 rejected goal cancellation; "
                "waiting for terminal result"
            )

    def cancel(self, reason="Operator stop"):
        """Request cancellation without blocking ROS callbacks."""
        if not self.busy:
            return False

        if not self.cancel_reason:
            self.cancel_reason = reason

        self._send_cancel()
        return True

    def tick(self):
        """Monitor safety and deadline during navigation."""
        if not self.busy:
            return

        if not self._allowed():
            self.cancel(
                "Safety check invalid: "
                + self._safety_reason()
            )

        elif time.monotonic() >= self.deadline:
            self.cancel("Navigation timeout")

    def _result(self, token, future):
        if token != self.token or not self.busy:
            return

        try:
            status = future.result().status
        except Exception as exc:
            self._finish(
                "ERROR",
                f"Result unavailable: {exc}",
            )
            return

        if self.cancel_reason:
            if status == GoalStatus.STATUS_CANCELED:
                outcome = "CANCELED"
            elif status == GoalStatus.STATUS_SUCCEEDED:
                outcome = "UNSAFE_COMPLETION"
            else:
                outcome = "FAILED"

            self._finish(
                outcome,
                f"{self.cancel_reason}; "
                f"terminal status={status}; "
                f"cancel response={self.cancel_ack}",
            )

        elif status == GoalStatus.STATUS_SUCCEEDED:
            self._finish(
                "SUCCEEDED",
                "Nav2 reported arrival",
            )

        elif status == GoalStatus.STATUS_CANCELED:
            self._finish(
                "CANCELED",
                "Nav2 canceled goal",
            )

        else:
            self._finish(
                "FAILED",
                f"Nav2 result status={status}",
            )

    def _finish(self, outcome, reason):
        goal = self.goal

        self.phase = "IDLE"
        self.goal = None
        self.handle = None
        self.deadline = None
        self.cancel_reason = None
        self.cancel_ack = None

        self.node.get_logger().info(
            f"Frontier Nav2: {outcome}: {reason}"
        )

        if self.on_finished is not None:
            self.on_finished(
                goal,
                outcome,
                reason,
            )
