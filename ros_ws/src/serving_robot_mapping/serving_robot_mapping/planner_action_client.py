"""Safely manage Nav2 ComputePathToPose requests.

Planning only. No NavigateToPose or velocity commands.
"""

import math
import time

from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import ComputePathToPose
from rclpy.action import ActionClient
from visualization_msgs.msg import Marker

from action_msgs.msg import GoalStatus
from serving_robot_mapping.frontier_failure_memory import (
    FrontierFailureMemory,
)
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
        freshness_monitor=None,
        context_ready=None,
        replanning_policy=None,
        failure_memory=None,
    ):
        if request_timeout_s <= 0:
            raise ValueError("Request timeout must be positive")

        self.node = node
        self.on_verified = on_verified
        self.request_timeout_s = request_timeout_s

        self.freshness_monitor = freshness_monitor
        self.context_ready = context_ready
        self.replanning_policy = replanning_policy
        self.failure_memory = (
            failure_memory
            if failure_memory is not None
            else FrontierFailureMemory(
                match_radius_m=0.25,
                max_failures=3,
            )
        )
        self.last_seen_map_revision = None
        self.active_snapshot = None
        self.approved_snapshot = None
        self.context_wait_logged = False

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

    def context_status(self, snapshot):
        """Check the original map/pose/time of a request."""

        if self.freshness_monitor is None:
            return True, "Freshness monitor disabled"

        if (
            self.context_ready is not None
            and not self.context_ready()
        ):
            return False, "Map-frame robot TF unavailable"

        if snapshot is None:
            return False, "Planning snapshot unavailable"

        return self.freshness_monitor.validate(
            snapshot,
            time.monotonic(),
        )

    def invalidate_context(self, reason):
        """Reject stale planning results without retrying blindly."""

        self.node.get_logger().warn(
            f"Planning context invalidated: {reason}"
        )

        was_busy = self.busy
        self.invalidate_active()

        self.verified = []
        self.approved_snapshot = None

        if self.replanning_policy is not None:
            now = time.monotonic()

            if (
                reason == "Planning result expired"
                and not was_busy
            ):
                self.replanning_policy.request_recheck(now)
            else:
                self.replanning_policy.context_changed(now)

        # Conservative policy: a changed goal list or restart
        # is needed before another planning attempt.
        self.completed_signature = self.signature

        self.on_verified([])

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
        self.active_snapshot = None

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
        self.approved_snapshot = None

        self.signature = signature
        self.candidates = candidates
        self.completed_signature = None

        if self.replanning_policy is not None:
            self.replanning_policy.set_goal(
                signature if candidates else None,
                time.monotonic(),
            )

        # Never retain an old approved preview after goals change.
        self.on_verified([])

        self.node.get_logger().info(
            f"Planner candidates updated: {len(candidates)}"
        )

    def tick(self):
        """Start planning or handle timeouts and stale context."""

        # A stalled action must count as a failure even when
        # the planning snapshot is also about to expire.
        if (
            self.busy
            and self.deadline_s is not None
            and time.monotonic() >= self.deadline_s
        ):
            self.node.get_logger().warn(
                "Planner request timed out after "
                f"{self.request_timeout_s:.1f} s"
            )

            self.invalidate_active()
            self.verified = []
            self.on_verified([])
            self.completed_signature = self.signature

            if self.replanning_policy is not None:
                self.replanning_policy.finish_attempt(
                    False, time.monotonic()
                )

            return

        # Detect every observed map revision change,
        # including changes while waiting to retry.
        if (
            self.replanning_policy is not None
            and self.freshness_monitor is not None
        ):
            revision = self.freshness_monitor.map_revision

            if self.last_seen_map_revision is None:
                self.last_seen_map_revision = revision

            elif revision != self.last_seen_map_revision:
                # R16_172_RELEASE_STABILITY
                # A real map revision may open a previously blocked path.
                # Do not clear this memory for TF failures, timers or
                # identical repeated /map messages.
                self.last_seen_map_revision = revision
                self.failure_memory.clear_on_map_change()

                if self.busy or self.approved_snapshot is not None:
                    self.invalidate_context("Map changed")
                    self.replanning_policy.map_changed(time.monotonic())
                    return

                self.replanning_policy.map_changed(time.monotonic())

        if self.busy and self.freshness_monitor is not None:
            valid, reason = self.context_status(
                self.active_snapshot
            )

            if not valid:
                self.invalidate_context(reason)
                return

        if self.approved_snapshot is not None:
            valid, reason = self.context_status(
                self.approved_snapshot
            )

            if not valid:
                self.invalidate_context(reason)
                return

        if self.busy:
            return

        if not self.candidates:
            return

        eligible_candidates = (
            self.failure_memory.available_candidates(
                self.candidates
            )
        )

        if not eligible_candidates:
            if self.completed_signature != self.signature:
                # R16_172_RELEASE_STABILITY
                # Report exhaustion once; the Explorer can rotate the
                # whole failed batch without relaxing any validator.
                self.node.get_logger().warn(
                    "All frontier candidates blocked "
                    "by failure memory."
                )
                self.completed_signature = self.signature
                self.on_verified([])
            return

        if self.replanning_policy is not None:
            decision = self.replanning_policy.decision(
                time.monotonic()
            )

            if not decision.allowed:
                return

        elif self.signature == self.completed_signature:
            return

        if not self.client.server_is_ready():
            if not self.waiting_logged:
                self.node.get_logger().info(
                    "Nav2 Planner unavailable; waiting."
                )
                self.waiting_logged = True
            return

        if self.freshness_monitor is not None:
            ready = (
                self.context_ready is None
                or self.context_ready()
            )

            if not ready:
                if not self.context_wait_logged:
                    self.node.get_logger().info(
                        "Waiting for planning map/TF context."
                    )
                    self.context_wait_logged = True
                return

            try:
                self.active_snapshot = (
                    self.freshness_monitor.capture(
                        time.monotonic()
                    )
                )
            except (RuntimeError, ValueError) as exc:
                if not self.context_wait_logged:
                    self.node.get_logger().warn(
                        f"Planning context not ready: {exc}"
                    )
                    self.context_wait_logged = True
                return

            self.context_wait_logged = False

        if self.replanning_policy is not None:
            self.replanning_policy.begin_attempt(
                time.monotonic()
            )

        self.waiting_logged = False
        self.busy = True

        self.active_signature = self.signature
        self.active_candidates = tuple(
            eligible_candidates
        )
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

        while (
            self.index < len(self.active_candidates)
            and self.failure_memory.is_blocked(
                *self.active_candidates[self.index]
            )
        ):
            self.index += 1

        if self.index >= len(self.active_candidates):
            if self.freshness_monitor is not None:
                valid, reason = self.context_status(
                    self.active_snapshot
                )

                if not valid:
                    self.invalidate_context(reason)
                    return

            results = list(self.verified)

            if self.replanning_policy is not None:
                self.replanning_policy.finish_attempt(
                    bool(results),
                    time.monotonic(),
                )

                if not results:
                    decision = self.replanning_policy.decision(
                        time.monotonic()
                    )

                    self.node.get_logger().info(
                        "Replanning policy: "
                        f"{decision.reason}, "
                        f"wait={decision.wait_s:.1f} s"
                    )

            self.approved_snapshot = (
                self.active_snapshot if results else None
            )

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
        """Validate only matching, fresh action results."""

        if not self.is_current(token):
            return

        if self.freshness_monitor is not None:
            valid, reason = self.context_status(
                self.active_snapshot
            )

            if not valid:
                self.invalidate_context(reason)
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
                self.failure_memory.record_success(
                    requested_pose.pose.position.x,
                    requested_pose.pose.position.y,
                )

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

                # R16_172_RELEASE_STABILITY
                # Preserve fail-closed validation: endpoint mismatch,
                # malformed result, and NO_VALID_PATH all disqualify
                # this candidate for the current map. No invalid path
                # is ever promoted to navigation.
                x = requested_pose.pose.position.x
                y = requested_pose.pose.position.y
                failures = self.failure_memory.record_failure(x, y)
                self.node.get_logger().warn(
                    "Frontier planning failure: "
                    f"({x:.2f}, {y:.2f}), "
                    f"count={failures}/{self.failure_memory.max_failures}"
                )
                if self.failure_memory.is_blocked(x, y):
                    self.node.get_logger().warn(
                        "Frontier candidate excluded after "
                        "repeated invalid planner results."
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
