"""Autonomous mapping workflow manager."""

import math
import time
from typing import Optional

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import OccupancyGrid, Odometry, Path
from rclpy.qos import (
    DurabilityPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from visualization_msgs.msg import Marker
from rclpy.time import Time
from tf2_ros import Buffer, TransformException, TransformListener
from rclpy.node import Node

from serving_robot_mapping.planner_action_client import (
    FrontierPlanningClient,
)
from serving_robot_mapping.frontier_navigation_client import (
    FrontierNavigationClient,
)
from serving_robot_mapping.mapping_navigation_safety_monitor import (
    MappingNavigationSafetyMonitor,
)
from serving_robot_mapping.planning_freshness import (
    PlanningFreshnessMonitor,
)
from serving_robot_mapping.tf_freshness import (
    TfFreshnessChecker,
)
from serving_robot_mapping.tf_recovery_gate import (
    TfRecoveryGate,
)
from serving_robot_mapping.replanning_policy import (
    ReplanningPolicy,
)
from serving_robot_mapping.frontier_detector import FrontierDetector
from serving_robot_mapping.exploration_goal_manager import (
    ExplorationGoalManager,
    VerifiedFrontierGoal,
)
from serving_robot_mapping.mapping_state_machine import (
    MappingState,
    MappingStateMachine,
)
from serving_robot_mapping.motion_controller import (
    RelativeRotationController,
    quaternion_to_yaw,
)



from sensor_msgs.msg import LaserScan as MappingGuardLaserScan
from rclpy.qos import qos_profile_sensor_data as mapping_guard_scan_qos
from rclpy.time import Time as MappingGuardTime
from serving_robot_mapping.mapping_frontier_clearance import (
    FrontierClearanceGuard,
)

class ExplorationManager(Node):
    """Manage the autonomous mapping process."""

    def __init__(self) -> None:
        super().__init__("exploration_manager")

        self.declare_parameter(
            "robot_id",
            "robot_01",
        )

        self.declare_parameter(
            "topics.odom",
            "/robot_01/odom",
        )

        self.declare_parameter(
            "topics.cmd_vel",
            "/mapping/test_only_cmd_vel_unconnected",
        )

        self.declare_parameter(
            "motion.rotation.kp",
            0.80,
        )

        self.declare_parameter(
            "motion.rotation.max_angular_speed_rps",
            0.30,
        )

        self.declare_parameter(
            "motion.rotation.min_angular_speed_rps",
            0.05,
        )

        self.declare_parameter(
            "motion.rotation.angle_tolerance_rad",
            0.02,
        )

        self.declare_parameter(
            "motion.rotation.stop_angular_velocity_rps",
            0.02,
        )

        self.declare_parameter(
            "motion.rotation.settle_cycles",
            5,
        )

        self.declare_parameter(
            "initial_scan.target_angle_rad",
            2.0 * math.pi,
        )

        self.declare_parameter(
            "initial_scan.skip", False
        )
        self.declare_parameter(
            "topics.map", "/map"
        )
        self.declare_parameter(
            "topics.validated_path",
            "/mapping/frontier_valid_path"
        )
        self.declare_parameter(
            "planning.enabled", False
        )
        self.declare_parameter("navigation.enabled", False)
        self.declare_parameter(
            "navigation.action", "/navigate_to_pose"
        )
        self.declare_parameter(
            "navigation.max_goals_per_run", 1
        )
        self.declare_parameter(
            "navigation.timeout_s", 120.0
        )
        self.declare_parameter(
            "navigation.recovery_wait_s", 3.0
        )
        self.declare_parameter(
            "navigation.recovery_stable_s", 3.0
        )
        self.declare_parameter(
            "planning.max_age_s", 10.0
        )
        self.declare_parameter(
            "planning.tf_max_past_age_s", 0.50
        )
        self.declare_parameter(
            "planning.tf_max_future_offset_s", 0.30
        )
        self.declare_parameter(
            "planning.tf_recovery_stable_s", 1.0
        )
        self.declare_parameter(
            "planning.max_translation_m", 0.15
        )
        self.declare_parameter(
            "planning.max_rotation_deg", 15.0
        )
        self.declare_parameter(
            "planning.action", "/compute_path_to_pose"
        )
        self.declare_parameter(
            "topics.frontier_goals",
            "/mapping/frontier_goals"
        )
        self.declare_parameter(
            "frontier.min_size_m", 0.40
        )
        self.declare_parameter(
            "frontier.free_threshold", 25
        )
        self.declare_parameter(
            "completion.no_frontier_cycles", 5
        )
        self.declare_parameter(
            "completion.stable_time_s", 10.0
        )

        self.robot_id = self.get_parameter(
            "robot_id"
        ).value

        self.odom_topic = self.get_parameter(
            "topics.odom"
        ).value

        self.cmd_vel_topic = self.get_parameter(
            "topics.cmd_vel"
        ).value

        self.initial_scan_target = float(
            self.get_parameter(
                "initial_scan.target_angle_rad"
            ).value
        )

        self.state_machine = MappingStateMachine()

        self.freshness_monitor = PlanningFreshnessMonitor(
            max_age_s=float(
                self.get_parameter(
                    "planning.max_age_s"
                ).value
            ),
            max_translation_m=float(
                self.get_parameter(
                    "planning.max_translation_m"
                ).value
            ),
            max_rotation_rad=math.radians(
                float(
                    self.get_parameter(
                        "planning.max_rotation_deg"
                    ).value
                )
            ),
        )

        self.tf_recovery_gate = TfRecoveryGate(
            stable_s=float(
                self.get_parameter(
                    "planning.tf_recovery_stable_s"
                ).value
            )
        )

        self.tf_freshness_checker = TfFreshnessChecker(
            max_past_age_s=float(
                self.get_parameter(
                    "planning.tf_max_past_age_s"
                ).value
            ),
            max_future_offset_s=float(
                self.get_parameter(
                    "planning.tf_max_future_offset_s"
                ).value
            ),
        )

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(
            self.tf_buffer, self
        )

        self.freshness_context_ready = False
        self.freshness_tf_missing = False

        self.skip_initial_scan = bool(
            self.get_parameter("initial_scan.skip").value
        )

        self.map_topic = str(
            self.get_parameter("topics.map").value
        )

        self.frontier_detector = FrontierDetector(
            min_size_m=float(
                self.get_parameter(
                    "frontier.min_size_m"
                ).value
            ),
            free_threshold=int(
                self.get_parameter(
                    "frontier.free_threshold"
                ).value
            ),
        )

        self.goal_manager = ExplorationGoalManager(
            state_machine=self.state_machine,
            no_frontier_cycles=int(
                self.get_parameter(
                    "completion.no_frontier_cycles"
                ).value
            ),
            stable_time_s=float(
                self.get_parameter(
                    "completion.stable_time_s"
                ).value
            ),
        )

        self.planning_enabled = bool(
            self.get_parameter("planning.enabled").value
        )

        self.planner_verified_goal = None

        self.replanning_policy = ReplanningPolicy(
            settle_s=2.0,
            min_interval_s=3.0,
            retry_base_s=2.0,
            retry_max_s=16.0,
            max_failures=3,
        )

        self.planner_client = FrontierPlanningClient(
            node=self,
            action_name=str(
                self.get_parameter("planning.action").value
            ),
            on_verified=self.planner_results_callback,
            freshness_monitor=self.freshness_monitor,
            context_ready=lambda: self.freshness_context_ready,
            replanning_policy=self.replanning_policy,
        )

        self.navigation_enabled = bool(
            self.get_parameter("navigation.enabled").value
        )
        self.navigation_max_goals = int(
            self.get_parameter(
                "navigation.max_goals_per_run"
            ).value
        )
        self.navigation_timeout_s = float(
            self.get_parameter("navigation.timeout_s").value
        )
        self.navigation_recovery_wait_s = float(
            self.get_parameter(
                "navigation.recovery_wait_s"
            ).value
        )

        if self.navigation_max_goals < 0:
            raise ValueError(
                "navigation.max_goals_per_run must be >= 0"
            )

        if (
            not math.isfinite(self.navigation_recovery_wait_s)
            or self.navigation_recovery_wait_s < 0
        ):
            raise ValueError("Invalid recovery wait")

        if self.navigation_enabled and not self.planning_enabled:
            raise ValueError(
                "Navigation requires planning.enabled:=true"
            )

        if (
            self.navigation_enabled
            and not bool(
                self.get_parameter("initial_scan.skip").value
            )
        ):
            raise ValueError(
                "Navigation requires initial_scan.skip:=true"
            )

        self.declare_parameter(
            "navigation.goal_clearance_m", 0.52
        )
        self.declare_parameter(
            "navigation.start_clearance_m", 0.50
        )

        self.frontier_clearance_guard = FrontierClearanceGuard(
            goal_clearance_m=float(
                self.get_parameter(
                    "navigation.goal_clearance_m"
                ).value
            ),
            start_clearance_m=float(
                self.get_parameter(
                    "navigation.start_clearance_m"
                ).value
            ),
        )

        self.navigation_start_scan = None
        self.navigation_start_scan_wall = None
        self._mapping_guard_required = True

        if self.navigation_enabled:
            self.mapping_guard_scan_sub = (
                self.create_subscription(
                    MappingGuardLaserScan,
                    f"/{self.robot_id}/scan",
                    self.mapping_guard_scan_callback,
                    mapping_guard_scan_qos,
                )
            )

        self.navigation_sent = 0
        self.navigation_recovery_until = 0.0
        self.navigation_recovery_safe_since = None
        self.navigation_recovery_stable_s = float(
            self.get_parameter(
                "navigation.recovery_stable_s"
            ).value
        )
        if (
            not math.isfinite(self.navigation_recovery_stable_s)
            or self.navigation_recovery_stable_s < 1.0
        ):
            raise ValueError("Invalid recovery stable time")
        self.navigation_rearm_pending = False
        self.visited_frontiers = []

        self.navigation_monitor = None
        self.navigation_client = None

        if self.navigation_enabled:
            self.navigation_monitor = (
                MappingNavigationSafetyMonitor(
                    self,
                    self.tf_buffer,
                    robot_id=self.robot_id,
                )
            )
            self.navigation_client = FrontierNavigationClient(
                self,
                action_name=str(
                    self.get_parameter(
                        "navigation.action"
                    ).value
                ),
                enabled=True,
                safety_check=self.navigation_monitor.is_safe,
                safety_reason=lambda: (
                    self.navigation_monitor.last_result[1]
                ),
                on_finished=self.navigation_finished,
                timeout_s=self.navigation_timeout_s,
            )

            self.get_logger().warn(
                "Mapping Nav2 navigation ENABLED. "
                f"Goal limit={self.navigation_max_goals} "
                "(0 means unlimited)."
            )
        else:
            self.get_logger().info(
                "Mapping Nav2 navigation disabled."
            )

        self.last_frontier_count = None
        self.pending_goal = None

        self.validated_path_topic = str(
            self.get_parameter(
                "topics.validated_path"
            ).value
        )

        self.current_yaw: Optional[float] = None

        self.rotation_controller = RelativeRotationController(
            kp=float(
                self.get_parameter(
                    "motion.rotation.kp"
                ).value
            ),
            max_angular_speed=float(
                self.get_parameter(
                    "motion.rotation.max_angular_speed_rps"
                ).value
            ),
            min_angular_speed=float(
                self.get_parameter(
                    "motion.rotation.min_angular_speed_rps"
                ).value
            ),
            angle_tolerance=float(
                self.get_parameter(
                    "motion.rotation.angle_tolerance_rad"
                ).value
            ),
            stop_angular_velocity=float(
                self.get_parameter(
                    "motion.rotation.stop_angular_velocity_rps"
                ).value
            ),
            settle_cycles=int(
                self.get_parameter(
                    "motion.rotation.settle_cycles"
                ).value
            ),
        )

        self.cmd_vel_pub = self.create_publisher(
            Twist,
            self.cmd_vel_topic,
            10,
        )

        self.odom_sub = self.create_subscription(
            Odometry,
            self.odom_topic,
            self.odom_callback,
            20,
        )

        self.map_sub = self.create_subscription(
            OccupancyGrid,
            self.map_topic,
            self.map_callback,
            10,
        )

        self.validated_path_sub = self.create_subscription(
            Path,
            self.validated_path_topic,
            self.validated_path_callback,
            10,
        )

        marker_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )

        self.frontier_goals_sub = self.create_subscription(
            Marker,
            str(
                self.get_parameter(
                    "topics.frontier_goals"
                ).value
            ),
            self.mapping_update_candidates,
            marker_qos,
        )

        self.planning_timer = self.create_timer(
            1.0,
            self.planning_loop,
        )

        self.navigation_timer = self.create_timer(
            0.2,
            self.navigation_loop,
        )

        self.freshness_timer = self.create_timer(
            1.0,
            self.freshness_observation_loop,
        )

        self.control_timer = self.create_timer(
            0.05,
            self.control_loop,
        )

        self.get_logger().info(
            f"Exploration Manager ready for {self.robot_id}"
        )

        self.get_logger().info(
            f"Waiting for odometry on {self.odom_topic}"
        )

    def odom_callback(self, msg: Odometry) -> None:
        """Forward odometry to the rotation controller."""
        orientation = msg.pose.pose.orientation

        yaw = quaternion_to_yaw(
            orientation.x,
            orientation.y,
            orientation.z,
            orientation.w,
        )

        self.current_yaw = yaw

        self.rotation_controller.update_odometry(
            current_yaw=yaw,
            angular_velocity=msg.twist.twist.angular.z,
        )

    def mapping_update_candidates(self, marker):
        """Expand Frontier viewpoints BEFORE Nav2 path verification."""
        if (
            not self.navigation_enabled
            or marker.action != marker.ADD
            or marker.type != marker.SPHERE_LIST
            or marker.header.frame_id != "map"
        ):
            self.planner_client.update_candidates(marker)
            return

        groups = []

        for point in marker.points[:10]:
            groups.append(
                self.frontier_clearance_guard.observation_candidates(
                    point.x,
                    point.y,
                    max_results=2,
                )
            )

        # Round-robin selection prevents one Frontier cluster
        # from monopolizing Nav2's 10-candidate planning budget.
        selected = []
        seen = set()

        for alternative_index in range(2):
            for group in groups:
                if alternative_index >= len(group):
                    continue

                x, y = group[alternative_index]
                key = (round(x, 2), round(y, 2))

                if key in seen:
                    continue

                seen.add(key)
                selected.append((x, y))

                if len(selected) >= 10:
                    break

            if len(selected) >= 10:
                break

        forwarded = type(marker)()
        forwarded.header = marker.header
        forwarded.type = marker.type
        forwarded.action = (
            marker.ADD if selected else marker.DELETE
        )

        # Keep RViz's original Frontier markers untouched.
        if marker.points:
            point_type = type(marker.points[0])

            for x, y in selected:
                point = point_type()
                point.x = float(x)
                point.y = float(y)
                point.z = 0.0
                forwarded.points.append(point)

        self.planner_client.update_candidates(forwarded)

        summary = (len(marker.points), len(selected))
        if summary != getattr(
            self, "_last_observation_summary", None
        ):
            self._last_observation_summary = summary
            self.get_logger().info(
                "Mapping observation candidates: "
                f"{summary[0]} Frontier points -> "
                f"{summary[1]} clearance-qualified viewpoints"
            )

    def mapping_guard_scan_callback(self, msg):
        self.navigation_start_scan = msg
        self.navigation_start_scan_wall = time.monotonic()

    def mapping_start_clearance_check(self):
        msg = self.navigation_start_scan
        received = self.navigation_start_scan_wall

        if msg is None or received is None:
            return False, "START_SCAN_MISSING"

        if not msg.header.frame_id:
            return False, "START_SCAN_FRAME_MISSING"

        # This conservative radial check assumes LiDAR and base
        # centers coincide in XY, as measured on robot_01.
        # Reject changed sensor mounting rather than guessing.
        try:
            transform = self.tf_buffer.lookup_transform(
                "base_footprint",
                msg.header.frame_id,
                MappingGuardTime(),
            )
        except Exception:
            return False, "START_LIDAR_TF_UNAVAILABLE"

        t = transform.transform.translation
        q = transform.transform.rotation

        yaw = math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y*q.y + q.z*q.z),
        )

        if (
            math.hypot(t.x, t.y) > 0.02
            or abs(yaw) > 0.05
        ):
            return False, "START_LIDAR_FRAME_OFFSET"

        return self.frontier_clearance_guard.start_clear(
            ranges=msg.ranges,
            range_min=msg.range_min,
            range_max=msg.range_max,
            wall_age=time.monotonic() - received,
        )

    def planner_results_callback(self, verified_goals):
        """Dispatch an action-verified frontier only when safe."""
        self.planner_verified_goal = None

        if self.state_machine.state != MappingState.EXPLORING:
            return

        if not self.last_frontier_count:
            return

        # Do not repeatedly navigate to a reached frontier.
        eligible = [
            goal for goal in verified_goals
            if all(
                math.hypot(goal.x - x, goal.y - y) > 0.25
                for x, y in self.visited_frontiers
            )
        ]

        if (
            self.navigation_enabled
            and getattr(self, "_mapping_guard_required", False)
        ):
            clearance_ok = []

            for goal in eligible:
                allowed, why = (
                    self.frontier_clearance_guard.goal_clear(
                        goal.x, goal.y
                    )
                )

                if allowed:
                    clearance_ok.append(goal)
                else:
                    self.get_logger().warn(
                        "Frontier rejected by Mapping clearance: "
                        f"({goal.x:.2f}, {goal.y:.2f}) {why}"
                    )

            eligible = clearance_ok

            if not eligible:
                self.get_logger().warn(
                    "No clearance-qualified Frontier; "
                    "no Nav2 goal dispatched."
                )
                return

        selected = self.goal_manager.select_goal(eligible)
        if selected is None:
            return

        self.planner_verified_goal = selected

        if not self.navigation_enabled:
            self.get_logger().info(
                "ACTION-VERIFIED PREVIEW: "
                f"({selected.x:.2f}, {selected.y:.2f}), "
                f"path={selected.path_length_m:.2f} m; "
                "navigation disabled."
            )
            return

        if (
            self.navigation_max_goals > 0
            and self.navigation_sent >= self.navigation_max_goals
        ):
            self.get_logger().info(
                "Mapping navigation goal limit reached."
            )
            return

        if self.navigation_client.busy:
            return

        if not self.navigation_monitor.is_safe():
            self.get_logger().warn(
                "Frontier navigation blocked by live safety."
            )
            self.request_navigation_replan()
            return

        if getattr(self, "_mapping_guard_required", False):
            allowed, why = self.mapping_start_clearance_check()

            if not allowed:
                self.get_logger().warn(
                    "Mapping start clearance BLOCKED: "
                    f"{why}; no goal dispatched."
                )
                return

        if not self.navigation_client.start(selected):
            self.get_logger().warn(
                "Nav2 goal not dispatched; will replan."
            )
            self.request_navigation_replan()
            return

        self.goal_manager.begin_navigation(selected)
        self.navigation_sent += 1

        self.get_logger().info(
            "MAPPING NAV2 GOAL SENT: "
            f"({selected.x:.2f}, {selected.y:.2f}), "
            f"path={selected.path_length_m:.2f} m, "
            f"request={self.navigation_sent}"
        )

    def planning_loop(self):
        """Plan only while exploring and live conditions allow."""
        if not self.planning_enabled:
            return

        if self.state_machine.state != MappingState.EXPLORING:
            return

        if not self.last_frontier_count:
            return

        if (
            self.navigation_enabled
            and self.navigation_max_goals > 0
            and self.navigation_sent >= self.navigation_max_goals
        ):
            return

        if self.navigation_rearm_pending:
            if self.planner_client.busy:
                return

            self.planner_client.invalidate_context(
                "Mapping navigation transition"
            )
            self.planner_client.completed_signature = None
            self.replanning_policy.request_recheck(
                time.monotonic()
            )
            self.navigation_rearm_pending = False

        if (
            self.navigation_enabled
            and not self.navigation_monitor.is_safe()
        ):
            return

        self.planner_client.tick()

    def request_navigation_replan(self):
        """Refresh TF/pose before another path request."""
        self.navigation_rearm_pending = True
        self.freshness_context_ready = False
        self.tf_recovery_gate.observe(
            False, time.monotonic()
        )
        self.planner_verified_goal = None

    def navigation_finished(self, goal, outcome, reason):
        """Handle one terminal Nav2 action result."""
        self.get_logger().info(
            f"Mapping navigation result: {outcome}: {reason}"
        )

        if self.state_machine.state != MappingState.NAVIGATING:
            return

        if outcome == "SUCCEEDED":
            if goal is not None:
                self.visited_frontiers.append(
                    (goal.x, goal.y)
                )
                self.visited_frontiers = (
                    self.visited_frontiers[-200:]
                )

            self.goal_manager.navigation_succeeded()
            self.request_navigation_replan()

            if (
                self.navigation_max_goals > 0
                and self.navigation_sent >= self.navigation_max_goals
            ):
                self.get_logger().warn(
                    "Single-run goal limit reached. "
                    "No additional goals will be sent."
                )
            return

        self.goal_manager.navigation_failed()

        # Only a confirmed terminal CANCELED result and
        # accepted cancellation can authorize recovery.
        # Physical stopping still relies on Nav2's safety
        # chain, not the cancellation acknowledgement alone.
        transient_reasons = {
            "HEALTH_STALE",
            "HEALTH_SIM_STALE",
            "PERSON_LIMIT_STALE",
            "TF_NOT_STABLE",
            "MAP_ODOM_TF_STALE",
            "ODOM_BASE_TF_STALE",
            "SLAM_NOT_ACTIVE",
            "SCAN_STALE",
            "ODOM_STALE",
            "MAP_STALE",
        }

        prefix = "Safety check invalid: "
        safety_code = None

        if reason.startswith(prefix):
            safety_code = (
                reason[len(prefix):].split(";", 1)[0].strip()
            )

        recoverable_cancel = (
            outcome == "CANCELED"
            and safety_code in transient_reasons
            and "cancel response=ACCEPTED" in reason
            and "terminal status=5" in reason
        )

        # Only an ordinary Nav2 ABORTED result may retry.
        # A failed cancellation or timeout must stop.
        route_failure = (
            outcome == "FAILED"
            and reason == "Nav2 result status=6"
        )

        if recoverable_cancel or route_failure:
            self.navigation_recovery_until = (
                time.monotonic()
                + self.navigation_recovery_wait_s
            )
            self.navigation_recovery_safe_since = None
            self.request_navigation_replan()
            self.get_logger().warn(
                "Mapping recovery required: "
                f"{safety_code or outcome}. "
                "Waiting for continuous healthy evidence "
                "before replanning."
            )
            return

        # Cancel, rejection, timeout, and ambiguous outcomes
        # require an operator to restart the Explorer.
        self.state_machine.transition_to(MappingState.ERROR)
        self.get_logger().error(
            "Mapping navigation halted after "
            f"{outcome}. Manual restart required."
        )

    def navigation_loop(self):
        """Continuously monitor active movement and recovery."""
        if not self.navigation_enabled:
            return

        state = self.state_machine.state

        if state == MappingState.NAVIGATING:
            self.navigation_client.tick()
            return

        if state == MappingState.RECOVERY:
            now = time.monotonic()

            if not self.navigation_monitor.is_safe():
                self.navigation_recovery_safe_since = None
                return

            if self.navigation_recovery_safe_since is None:
                self.navigation_recovery_safe_since = now

            stable_for = (
                now - self.navigation_recovery_safe_since
            )

            if (
                now >= self.navigation_recovery_until
                and stable_for
                >= self.navigation_recovery_stable_s
            ):
                self.goal_manager.recovery_finished()
                self.get_logger().info(
                    "Safety recovered continuously; "
                    "returning to verified frontier planning."
                )
            return

        if state == MappingState.EXPLORING:
            # Keep the safety evidence and lifecycle probe warm.
            self.navigation_monitor.evaluate()

    def validated_path_callback(self, msg: Path) -> None:
        """Preview a planner-validated goal without navigating."""

        if self.state_machine.state != MappingState.EXPLORING:
            return

        if not self.last_frontier_count:
            return

        if msg.header.frame_id != "map":
            return

        if len(msg.poses) < 2:
            return

        length = 0.0

        for previous, current in zip(
            msg.poses, msg.poses[1:]
        ):
            a = previous.pose.position
            b = current.pose.position

            if not all(
                math.isfinite(v)
                for v in (a.x, a.y, b.x, b.y)
            ):
                return

            length += math.hypot(
                b.x - a.x,
                b.y - a.y,
            )

        endpoint = msg.poses[-1].pose.position

        candidate = VerifiedFrontierGoal(
            x=endpoint.x,
            y=endpoint.y,
            path_length_m=length,
        )

        selected = self.goal_manager.select_goal(
            [candidate]
        )

        if selected is None or selected == self.pending_goal:
            return

        self.pending_goal = selected

        self.get_logger().info(
            "Path candidate received: "
            f"({selected.x:.2f}, {selected.y:.2f}), "
            f"length={selected.path_length_m:.2f} m"
        )

        self.get_logger().info(
            "PREVIEW ONLY: Navigation is disabled."
        )

    def mark_tf_invalid(self, reason):
        """Block planning and discard approvals on invalid TF."""

        self.tf_recovery_gate.observe(
            False, time.monotonic()
        )

        if not self.freshness_tf_missing:
            self.get_logger().warn(
                f"Planning TF invalid: {reason}"
            )

        self.freshness_tf_missing = True
        self.freshness_context_ready = False

        # Invalidate an ongoing or previously approved plan.
        # The client also clears the Exploration Manager preview
        # through its on_verified callback.
        client = self.planner_client

        if (
            client.busy
            or client.approved_snapshot is not None
        ):
            client.invalidate_context(
                "TF unavailable or stale"
            )

    def freshness_observation_loop(self) -> None:
        """Validate the TF chain before updating robot pose."""

        if self.state_machine.state not in (
            MappingState.EXPLORING,
            MappingState.COMPLETION_CHECK,
        ):
            return

        try:
            map_odom = self.tf_buffer.lookup_transform(
                "map", "odom", Time()
            )

            odom_base = self.tf_buffer.lookup_transform(
                "odom", "base_footprint", Time()
            )

            map_base = self.tf_buffer.lookup_transform(
                "map", "base_footprint", Time()
            )

        except TransformException as exc:
            self.mark_tf_invalid(
                f"Transform unavailable: {exc}"
            )
            return

        # IMPORTANT: use ROS simulation time for TF stamps.
        # time.monotonic() remains appropriate for action
        # timeouts, but NOT for TF timestamp comparisons.
        sim_now_s = (
            self.get_clock().now().nanoseconds / 1e9
        )

        map_stamp = map_odom.header.stamp
        odom_stamp = odom_base.header.stamp

        result = self.tf_freshness_checker.validate(
            sim_now_s=sim_now_s,
            map_odom_stamp_s=(
                map_stamp.sec
                + map_stamp.nanosec * 1e-9
            ),
            odom_base_stamp_s=(
                odom_stamp.sec
                + odom_stamp.nanosec * 1e-9
            ),
        )

        if not result.valid:
            self.mark_tf_invalid(result.reason)
            return

        # After a TF failure, require a continuous healthy
        # interval before restoring the planning context.
        if not self.tf_recovery_gate.observe(
            True, time.monotonic()
        ):
            return

        position = map_base.transform.translation
        orientation = map_base.transform.rotation

        yaw = quaternion_to_yaw(
            orientation.x,
            orientation.y,
            orientation.z,
            orientation.w,
        )

        try:
            self.freshness_monitor.update_robot_pose(
                x=position.x,
                y=position.y,
                yaw=yaw,
            )
        except ValueError as exc:
            self.mark_tf_invalid(
                f"Invalid TF robot pose: {exc}"
            )
            return

        if self.freshness_tf_missing:
            self.get_logger().info(
                "Planning TF recovered: timestamps valid."
            )

        self.freshness_tf_missing = False

        if not self.freshness_context_ready:
            try:
                snapshot = self.freshness_monitor.capture(
                    now_s=time.monotonic()
                )
            except RuntimeError:
                # The initial SLAM map is not ready yet.
                return

            self.get_logger().info(
                "Planning context ready: "
                f"map revision={snapshot.map_revision}, "
                f"robot=({snapshot.robot_x:.2f}, "
                f"{snapshot.robot_y:.2f}), "
                f"yaw={math.degrees(snapshot.robot_yaw):.1f} deg"
            )

            self.get_logger().info(
                "TF freshness OK: "
                f"map->odom age={result.map_odom_age_s:.3f} s, "
                f"odom->base age={result.odom_base_age_s:.3f} s"
            )

            self.freshness_context_ready = True

    def map_callback(self, msg: OccupancyGrid) -> None:
        """Update map freshness and exploration observations."""

        if msg.header.frame_id not in ("", "map"):
            self.get_logger().warn(
                "Ignoring map with unexpected frame: "
                f"{msg.header.frame_id}"
            )
            return

        origin = msg.info.origin
        q = origin.orientation

        origin_yaw = quaternion_to_yaw(
            q.x, q.y, q.z, q.w
        )

        try:
            changed = self.freshness_monitor.update_map(
                data=msg.data,
                width=int(msg.info.width),
                height=int(msg.info.height),
                resolution=float(msg.info.resolution),
                origin_x=origin.position.x,
                origin_y=origin.position.y,
                origin_yaw=origin_yaw,
            )

        except (ValueError, OverflowError) as exc:
            self.get_logger().warn(
                f"Map freshness update failed: {exc}"
            )
            return

        if self.navigation_enabled:
            self.frontier_clearance_guard.update_map(
                data=msg.data,
                width=int(msg.info.width),
                height=int(msg.info.height),
                resolution=float(msg.info.resolution),
                origin_x=origin.position.x,
                origin_y=origin.position.y,
                origin_yaw=origin_yaw,
            )

        if changed:
            revision = self.freshness_monitor.map_revision

            # Avoid flooding the terminal during active SLAM.
            if revision == 1 or revision % 10 == 0:
                self.get_logger().info(
                    f"Map content revision: {revision}"
                )

        state = self.state_machine.state

        if state not in (
            MappingState.EXPLORING,
            MappingState.COMPLETION_CHECK,
        ):
            return

        width = int(msg.info.width)
        height = int(msg.info.height)
        resolution = float(msg.info.resolution)

        # Never assess completion on a map with no known-free area.
        if not any(
            0 <= value <= self.frontier_detector.free_threshold
            for value in msg.data
        ):
            return

        try:
            clusters = self.frontier_detector.detect(
                data=msg.data,
                width=width,
                height=height,
                resolution=resolution,
            )
        except ValueError as exc:
            self.get_logger().warn(
                f"Invalid map for frontier detection: {exc}"
            )
            return

        frontier_count = len(clusters)

        if frontier_count == 0:
            self.pending_goal = None
            self.planner_verified_goal = None

        previous_state = self.state_machine.state

        current_state = self.goal_manager.observe_frontiers(
            frontier_count=frontier_count,
            now_s=(
                self.get_clock().now().nanoseconds / 1e9
            ),
        )

        if frontier_count != self.last_frontier_count:
            self.get_logger().info(
                f"Frontier clusters: {frontier_count}"
            )
            self.last_frontier_count = frontier_count

        if current_state != previous_state:
            self.get_logger().info(
                f"State: {previous_state.value} "
                f"-> {current_state.value}"
            )

        # No navigation command is issued here.

    def control_loop(self) -> None:
        """Run behavior for the active mapping state."""
        if self.current_yaw is None:
            return

        state = self.state_machine.state

        if state == MappingState.IDLE:
            self.start_initial_scan()
            return

        if state == MappingState.INITIAL_SCAN:
            self.run_initial_scan()

    def start_initial_scan(self) -> None:
        """Request initial rotation or enter observation-only mode."""

        if self.skip_initial_scan:
            self.state_machine.transition_to(
                MappingState.EXPLORING
            )
            self.get_logger().info(
                "TEST MODE: Initial scan skipped."
            )
            self.get_logger().info(
                "State: IDLE -> EXPLORING"
            )
            return

        self.state_machine.transition_to(
            MappingState.INITIAL_SCAN
        )

        self.rotation_controller.start(
            target_angle=self.initial_scan_target,
            current_yaw=self.current_yaw,
        )

        self.get_logger().info(
            "State: IDLE -> INITIAL_SCAN"
        )

        self.get_logger().info(
            "Starting relative rotation: "
            f"{math.degrees(self.initial_scan_target):.2f} deg"
        )

    def run_initial_scan(self) -> None:
        """Execute the active relative rotation command."""
        angular_command = (
            self.rotation_controller.compute_command()
        )

        command = Twist()
        command.angular.z = angular_command

        # Continue sending zero velocity during the settling phase.
        self.cmd_vel_pub.publish(command)

        if self.rotation_controller.is_complete:
            self.finish_initial_scan()

    def finish_initial_scan(self) -> None:
        """Finish INITIAL_SCAN after physical motion has settled."""
        accumulated = (
            self.rotation_controller.accumulated_angle
        )

        error = self.rotation_controller.error

        self.cmd_vel_pub.publish(Twist())

        self.state_machine.transition_to(
            MappingState.EXPLORING
        )

        self.get_logger().info(
            "Initial scan complete."
        )

        self.get_logger().info(
            f"Target rotation: "
            f"{math.degrees(self.initial_scan_target):.2f} deg"
        )

        self.get_logger().info(
            f"Measured rotation: "
            f"{math.degrees(accumulated):.2f} deg"
        )

        self.get_logger().info(
            f"Final rotation error: "
            f"{math.degrees(error):.2f} deg"
        )

        self.get_logger().info(
            "State: INITIAL_SCAN -> EXPLORING"
        )

        self.get_logger().info(
            "Frontier exploration is not implemented yet."
        )

    def stop_robot(self) -> None:
        """Best-effort stop while ROS context is valid."""
        if not self.context.ok():
            return

        try:
            self.cmd_vel_pub.publish(Twist())
        except Exception:
            # Shutdown may invalidate the context between
            # the validity check and the publish call.
            pass

    def destroy_node(self) -> bool:
        """Request Nav2 cancellation before node shutdown."""
        client = getattr(self, "navigation_client", None)
        if client is not None and client.busy:
            client.cancel("Mapping Explorer shutting down")
        try:
            for _ in range(10):
                self.stop_robot()
        except Exception:
            pass

        return super().destroy_node()


def main(args=None) -> None:
    """Run the exploration manager."""
    rclpy.init(args=args)

    node = ExplorationManager()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        # ROS context may already be invalid after SIGINT.
        # Do not publish ROS logs during shutdown.
        pass

    finally:
        client = getattr(node, "navigation_client", None)

        if (
            client is not None
            and client.busy
            and rclpy.ok()
        ):
            client.cancel("Mapping Explorer shutting down")
            deadline = time.monotonic() + 3.0

            while (
                rclpy.ok()
                and client.busy
                and time.monotonic() < deadline
            ):
                try:
                    rclpy.spin_once(node, timeout_sec=0.1)
                except Exception:
                    break

            if client.busy:
                node.get_logger().error(
                    "Nav2 cancellation NOT confirmed. "
                    "Check robot motion and safety chain."
                )

        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
