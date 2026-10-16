"""Safely manage Nav2 ComputePathToPose requests.

Planning only. No NavigateToPose or velocity commands.
"""

import math
import time

from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import ComputePathToPose
from rclpy.action import ActionClient
from visualization_msgs.msg import Marker

from serving_robot_mapping.path_result_validator import (
    PathResultValidator,
)


class FrontierPlanningClient:
    """Validate frontier goals using matched action responses."""

    def __init__(
        self,
        node,
        action_name,
        on_verified,
        request_timeout_s=10.0,
    ):
        if request_timeout_s <= 0:
            raise ValueError("Request timeout must be positive")

        self.node = node
        self.on_verified = on_verified
        self.request_timeout_s = request_timeout_s

        self.client = ActionClient(
            node,
            ComputePathToPose,
            action_name,
        )

        self.validator = PathResultValidator()

        self.candidates = ()
        self.signature = None
        self.completed_signature = None
        self.active_signature = None

        self.busy = False
        self.waiting_logged = False

        self.active_candidates = ()
        self.index = 0
        self.verified = []

        # Unique token for every individual action request.
        self.token_counter = 0
        self.active_token = None

        # Uses wall-clock monotonic time, not simulation time.
        self.deadline_s = None
        self.active_handle = None

    def is_current(self, token):
        """Return whether a callback belongs to the active request."""
        return (
            self.busy
            and token == self.active_token
            and self.active_signature == self.signature
        )

    def invalidate_active(self):
        """Invalidate current callbacks and cancel when possible."""

        self.active_token = None
        self.deadline_s = None
        self.busy = False

        handle = self.active_handle
        self.active_handle = None

        if handle is not None:
            try:
                handle.cancel_goal_async()
            except Exception as exc:
                self.node.get_logger().warn(
                    f"Planner cancellation request failed: {exc}"
                )

    def update_candidates(self, marker):
        """Update frontier goals and discard stale planning results."""

        if marker.action == Marker.DELETE:
            candidates = ()

        elif (
            marker.action == Marker.ADD
            and marker.type == Marker.SPHERE_LIST
            and marker.header.frame_id == "map"
        ):
            candidates = []
            seen = set()

            for point in marker.points:
                if not (
                    math.isfinite(point.x)
                    and math.isfinite(point.y)
                ):
                    continue

                key = (
                    round(point.x, 3),
                    round(point.y, 3),
                )

                if key in seen:
                    continue

                seen.add(key)
                candidates.append((point.x, point.y))

                if len(candidates) >= 10:
                    break

            candidates = tuple(candidates)

        else:
            return

        signature = tuple(
            (round(x, 3), round(y, 3))
            for x, y in candidates
        )

        if signature == self.signature:
            return

        # Cancel or ignore any response from the previous goals.
        self.invalidate_active()

        self.signature = signature
        self.candidates = candidates
        self.completed_signature = None

        # Never retain an old approved preview after goals change.
        self.on_verified([])

        self.node.get_logger().info(
            f"Planner candidates updated: {len(candidates)}"
        )

    def tick(self):
        """Start planning or handle a request timeout."""

        if self.busy:
            if (
                self.deadline_s is not None
                and time.monotonic() >= self.deadline_s
            ):
                self.node.get_logger().warn(
                    "Planner request timed out after "
                    f"{self.request_timeout_s:.1f} s"
                )

                self.invalidate_active()

                # Fail closed: do not approve partial results.
                self.verified = []
                self.on_verified([])

                # Avoid continuously retrying the same stalled goal.
                # A changed candidate list or client restart
                # allows another attempt.
                self.completed_signature = self.signature

            return

        if not self.candidates:
            return

        if self.signature == self.completed_signature:
            return

        if not self.client.server_is_ready():
            if not self.waiting_logged:
                self.node.get_logger().info(
                    "Nav2 Planner unavailable; waiting."
                )
                self.waiting_logged = True
            return

        self.waiting_logged = False
        self.busy = True

        self.active_signature = self.signature
        self.active_candidates = tuple(self.candidates)
        self.index = 0
        self.verified = []

        self.node.get_logger().info(
            "Planner available; checking frontier goals."
        )

        self.send_next()

    def send_next(self):
        """Send the next candidate or complete the batch."""

        if (
            not self.busy
            or self.active_signature != self.signature
        ):
            return

        if self.index >= len(self.active_candidates):
            results = list(self.verified)

            self.busy = False
            self.active_token = None
            self.deadline_s = None
            self.active_handle = None
            self.completed_signature = self.signature

            self.node.get_logger().info(
                "Action validation finished: "
                f"{len(results)}/{len(self.active_candidates)} valid"
            )

            self.on_verified(results)
            return

        x, y = self.active_candidates[self.index]

        pose = PoseStamped()
        pose.header.frame_id = "map"
        pose.header.stamp = self.node.get_clock().now().to_msg()

        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.orientation.w = 1.0

        request = ComputePathToPose.Goal()
        request.goal = pose
        request.use_start = False
        request.planner_id = ""

        self.token_counter += 1
        token = self.token_counter

        self.active_token = token
        self.active_handle = None
        self.deadline_s = (
            time.monotonic() + self.request_timeout_s
        )

        self.node.get_logger().info(
            f"Requesting path {self.index + 1}/"
            f"{len(self.active_candidates)}: "
            f"({x:.2f}, {y:.2f}), token={token}"
        )

        try:
            future = self.client.send_goal_async(request)

            # Capture both the request identity and original pose.
            future.add_done_callback(
                lambda f, t=token, p=pose:
                self.goal_response(f, t, p)
            )

        except Exception as exc:
            self.node.get_logger().error(
                f"Path request failed: {exc}"
            )
            self.advance(token)

    def goal_response(self, future, token, requested_pose):
        """Handle acceptance only for the matching request."""

        try:
            handle = future.result()
        except Exception as exc:
            if self.is_current(token):
                self.node.get_logger().error(
                    f"Planner response failed: {exc}"
                )
                self.advance(token)
            return

        if not self.is_current(token):
            # A late acceptance may still own a running action.
            if handle.accepted:
                try:
                    handle.cancel_goal_async()
                except Exception:
                    pass
            return

        if not handle.accepted:
            self.node.get_logger().warn(
                "Planner rejected the request"
            )
            self.advance(token)
            return

        self.active_handle = handle

        try:
            future = handle.get_result_async()

            future.add_done_callback(
                lambda f, t=token, p=requested_pose:
                self.result_response(f, t, p)
            )

        except Exception as exc:
            self.node.get_logger().error(
                f"Planner result request failed: {exc}"
            )
            self.advance(token)

    def result_response(self, future, token, requested_pose):
        """Validate a result only if its original request is active."""

        if not self.is_current(token):
            return

        try:
            response = future.result()

            outcome = self.validator.validate(
                requested_goal=requested_pose,
                status=response.status,
                result=response.result,
                expected_frame="map",
            )

            if outcome.valid and outcome.goal is not None:
                self.verified.append(outcome.goal)

                self.node.get_logger().info(
                    "Planner-validated goal: "
                    f"({outcome.goal.x:.2f}, "
                    f"{outcome.goal.y:.2f}), "
                    f"path={outcome.goal.path_length_m:.2f} m"
                )
            else:
                self.node.get_logger().warn(
                    "Planner validation failed: "
                    f"{outcome.reason}"
                )

        except Exception as exc:
            self.node.get_logger().error(
                f"Planner result processing failed: {exc}"
            )

        self.advance(token)

    def advance(self, token):
        """Proceed only from the active request."""

        if not self.is_current(token):
            return

        self.active_token = None
        self.active_handle = None
        self.deadline_s = None

        self.index += 1
        self.send_next()
