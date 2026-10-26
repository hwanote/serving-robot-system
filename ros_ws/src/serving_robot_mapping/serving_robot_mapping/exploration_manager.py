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

from serving_robot_mapping.frontier_marker_adapter import frontier_representatives
from serving_robot_mapping.mapping_completion_verifier import verify_closed_free_region


from serving_robot_mapping.mapping_path_risk import MappingPathRiskGuard
from serving_robot_mapping.mapping_safety_diagnostics import (
    format_mapping_safety_diagnostic,
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
        self.declare_parameter("mapping.resume_report_path", "")
        self._r16_214_resume_report = str(
            self.get_parameter(
                "mapping.resume_report_path"
            ).value
        )
        self._r16_214_resume_pending = bool(
            self._r16_214_resume_report
        )

        self.declare_parameter("mapping.lidar_only", False)
        self.mapping_lidar_only = self.get_parameter(
            "mapping.lidar_only"
        ).value

        if type(self.mapping_lidar_only) is not bool:
            raise ValueError(
                "mapping.lidar_only must be boolean"
            )

        self.navigation_monitor = None
        self.navigation_client = None

        if self.navigation_enabled:
            self.navigation_monitor = (
                MappingNavigationSafetyMonitor(
                    self,
                    self.tf_buffer,
                    robot_id=self.robot_id,
                    lidar_only=self.mapping_lidar_only,
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
                safety_check=self.mapping_navigation_safety_check,
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
            or marker.type not in (marker.SPHERE_LIST, marker.POINTS)
            or marker.header.frame_id != "map"
        ):
            self.planner_client.update_candidates(marker)
            return

        # R16_171B_ROTATE_FAILED_PATHS
        # Remember the current Frontier marker. A rejected path
        # may trigger a new candidate batch without a map change.
        # R16_172_RELEASE_STABILITY: callback works with an
        # Explorer fixture that has no freshness monitor. A real
        # Explorer always supplies the monitor; absent revision is
        # never treated as evidence that a map has changed.
        revision = getattr(
            getattr(self, "freshness_monitor", None),
            "map_revision", None
        )
        if revision != getattr(
            self, "_r16_171b_revision", object()
        ):
            self._r16_171b_revision = revision
            self._r16_171b_rejected = set()
            self._r16_171b_refresh_pending = False

        self._r16_171b_marker = marker

        groups = []

        if marker.type == marker.POINTS:
            # Frontier Visualizer publishes ALL frontier cells
            # as POINTS, not SPHERE_LIST cluster centroids.
            grid = self.frontier_clearance_guard.grid

            if grid is None:
                frontier_xy = []
            else:
                _, _, _, res, ox, oy, yaw = grid

                frontier_xy = frontier_representatives(
                    marker.points,
                    resolution=res,
                    origin_x=ox,
                    origin_y=oy,
                    origin_yaw=yaw,
                    limit=10,
                )
        else:
            # Preserve legacy SPHERE_LIST compatibility.
            frontier_xy = [
                (point.x, point.y)
                for point in marker.points[:10]
            ]

        for x, y in frontier_xy:
            groups.append(
                self.frontier_clearance_guard.observation_candidates(
                    x,
                    y,
                    max_results=2,
                )
            )

        # Round-robin selection prevents one Frontier cluster
        # from monopolizing Nav2's 10-candidate planning budget.
        selected = []
        # Rejected actual Nav2 paths must not be retried
        # on an unchanged map revision.
        seen = set(getattr(
            self, "_r16_171b_rejected", ()
        ))

        for alternative_index in range(2):
            for group in groups:
                if alternative_index >= len(group):
                    continue

                x, y = group[alternative_index]
                key = (round(x, 2), round(y, 2))

                if key in seen:
                    continue

                # Exclude reached viewpoints BEFORE Nav2 planning.
                # Do not reduce obstacle clearance or claim completion.
                if any(
                    math.hypot(x - vx, y - vy) <= 0.25
                    for vx, vy in getattr(
                        self, "visited_frontiers", ()
                    )
                ):
                    continue

                seen.add(key)
                selected.append((x, y))

                if len(selected) >= 10:
                    break

            if len(selected) >= 10:
                break

        # The initial search samples only 10 representative
        # Frontier positions. If all are exhausted, search
        # additional regions before declaring exploration blocked.
        # R16_171C_FILL_PARTIAL_FRONTIER_BATCH
        # Once previous paths have been rejected, refill a
        # partial batch from additional Frontier regions.
        # Every result still requires Nav2 validation and
        # the unchanged Mapping path and live safety gates.
        if (
            (
                not selected
                or (
                    len(selected) < 10
                    and bool(getattr(
                        self, "_r16_171b_rejected", ()
                    ))
                )
            )
            and marker.type == marker.POINTS
            and grid is not None
            and marker.points
        ):
            expanded_frontiers = frontier_representatives(
                marker.points,
                resolution=res,
                origin_x=ox,
                origin_y=oy,
                origin_yaw=yaw,
                limit=64,
            )

            examined = 0

            for fx, fy in expanded_frontiers:
                examined += 1

                alternatives = (
                    self.frontier_clearance_guard
                    .observation_candidates(
                        fx,
                        fy,
                        max_results=4,
                    )
                )

                for x, y in alternatives:
                    key = (round(x, 2), round(y, 2))

                    if key in seen:
                        continue

                    if any(
                        math.hypot(x - vx, y - vy) <= 0.25
                        for vx, vy in getattr(
                            self, "visited_frontiers", ()
                        )
                    ):
                        continue

                    seen.add(key)
                    selected.append((x, y))

                    if len(selected) >= 10:
                        break

                if len(selected) >= 10:
                    break

            if selected:
                expanded_signature = (
                    examined,
                    tuple(
                        (round(x, 2), round(y, 2))
                        for x, y in selected
                    ),
                )

                if expanded_signature != getattr(
                    self, "_last_expanded_frontier_log", None
                ):
                    self._last_expanded_frontier_log = (
                        expanded_signature
                    )
                    self.get_logger().info(
                        "Expanded Frontier search found "
                        f"{len(selected)} unvisited safe viewpoints "
                        f"from {examined} examined representatives."
                    )

        # Whole-free-region visibility fallback:
        # only used when normal and expanded Frontier selection
        # found no unvisited clearance-qualified viewpoint.
        #
        # Keep valid cached viewpoints between marker updates.
        # Recheck both clearance and visited history on every use.
        # Expensive visibility scans are rate limited.
        if (
            not selected
            and marker.type == marker.POINTS
            and grid is not None
            and marker.points
        ):
            now = time.monotonic()
            visited = getattr(self, "visited_frontiers", ())

            for vx, vy in getattr(
                self, "_visibility_fallback_cache", ()
            ):
                if (round(vx, 2), round(vy, 2)) in seen:
                    continue

                if any(
                    math.hypot(vx - px, vy - py) <= 0.25
                    for px, py in visited
                ):
                    continue

                if not self.frontier_clearance_guard.goal_clear(
                    vx, vy
                )[0]:
                    continue

                selected.append((vx, vy))

                if len(selected) >= 8:
                    break

            if (
                not selected
                and now >= getattr(
                    self, "_visibility_fallback_retry_after",
                    0.0,
                )
            ):
                self._visibility_fallback_retry_after = (
                    now + 15.0
                )
                self._visibility_fallback_cache = ()

                try:
                    from serving_robot_mapping.mapping_visibility_fallback import (
                        select_visibility_fallback,
                    )

                    tf = self.tf_buffer.lookup_transform(
                        "map", "base_footprint", Time()
                    )
                    robot = tf.transform.translation

                    # Do not rescan identical unsuccessful
                    # conditions every 15 seconds.
                    scan_key = (
                        hash(grid),
                        round(robot.x, 1),
                        round(robot.y, 1),
                        tuple(
                            (round(vx, 2), round(vy, 2))
                            for vx, vy in visited
                        ),
                    )

                    if scan_key != getattr(
                        self,
                        "_visibility_fallback_last_scanned_context",
                        None,
                    ):
                        fallback, metrics = select_visibility_fallback(
                            grid=grid,
                            robot_xy=(robot.x, robot.y),
                            visited=visited,
                            goal_clear=(
                                self.frontier_clearance_guard.goal_clear
                            ),
                            max_results=8,
                        )

                        self._visibility_fallback_cache = (
                            tuple(fallback)
                        )
                        selected.extend(
                            (vx, vy)
                            for vx, vy in fallback
                            if (round(vx, 2), round(vy, 2))
                            not in seen
                        )

                        self.get_logger().info(
                            "Visibility fallback: "
                            f"{len(fallback)} clearance-qualified "
                            "viewpoints; "
                            f"visible_boundary="
                            f"{metrics['visible_boundary_cells']}, "
                            f"qualified_candidates="
                            f"{metrics['geometry_candidates']}, "
                            f"guard_rejected="
                            f"{metrics['guard_rejections']}, "
                            f"visited_rejected="
                            f"{metrics['visited_rejections']}, "
                            f"scored_viewpoints="
                            f"{metrics['scored_viewpoints']}, "
                            f"viewpoint_stride="
                        f"{metrics['viewpoint_stride_cells']}, "
                        f"guard_reasons="
                            f"{metrics['guard_rejection_reasons']}; "
                            "Nav2 path and live safety still required."
                        )

                        self._visibility_fallback_last_scanned_context = scan_key

                except Exception as exc:
                    # Fail closed: no unverified navigation goal.
                    self.get_logger().warn(
                        "Visibility fallback unavailable: "
                        f"{type(exc).__name__}: {exc}"
                    )

        forwarded = type(marker)()
        forwarded.header = marker.header
        forwarded.type = marker.SPHERE_LIST
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

        # New unvisited candidates must release any old
        # visited-only planning hold, even on the same map revision.
        if selected:
            signature = tuple(
                (round(x, 3), round(y, 3))
                for x, y in selected
            )
            if signature != getattr(
                self.planner_client, "signature", None
            ):
                self._visited_only_hold_revision = None

        # A lack of safe viewpoints is not proof of map completion.
        # Debounce temporary empty candidate sets before pausing.
        state_machine = getattr(self, "state_machine", None)
        current_state = getattr(state_machine, "state", None)

        if selected:
            self._no_viable_since = None

            if current_state == MappingState.EXPLORATION_BLOCKED:
                state_machine.transition_to(MappingState.EXPLORING)
                self.get_logger().info(
                    "New reachable Frontier candidate detected; "
                    "EXPLORATION_BLOCKED -> EXPLORING"
                )

        elif (
            marker.points
            and getattr(
                self.frontier_clearance_guard, "grid", None
            ) is not None
            and getattr(self, "last_frontier_count", 0) > 0
            and current_state == MappingState.EXPLORING
        ):
            now = time.monotonic()
            since = getattr(self, "_no_viable_since", None)

            if since is None or now < since:
                self._no_viable_since = now

            elif now - since >= 10.0:
                reason = (
                    "VISITED_VIEWPOINTS_ONLY"
                    if any(groups)
                    else "NO_CLEARANCE_QUALIFIED_VIEWPOINT"
                )

                state_machine.transition_to(
                    MappingState.EXPLORATION_BLOCKED
                )

                self.get_logger().warn(
                    "EXPLORATION_BLOCKED: "
                    f"{reason}; no Nav2 goal dispatched. "
                    "Waiting for new reachable Frontier. "
                    "MAP_COMPLETE not asserted."
                )

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

        if getattr(self, "_r16_214_resume_pending", False):
            return

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

        # Do not repeatedly replan only previously visited goals.
        # This is a planning hold, NOT proof of map completion.
        if verified_goals and not eligible:
            revision = getattr(
                getattr(self, "freshness_monitor", None),
                "map_revision", None
            )
            if revision is not None:
                self._visited_only_hold_revision = revision
                self.get_logger().warn(
                    "Only visited Frontier goals remain; "
                    f"holding until map revision changes ({revision}). "
                    "MAP_COMPLETE not asserted."
                )
            return

        if eligible:
            self._visited_only_hold_revision = None

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

        # R16_170B_ACTUAL_PLANNER_PATH_SCREEN
        # Screen the ACTUAL planner polyline against the freshest Mapping grid.
        # Missing path geometry or a missing map fails closed. This gate never
        # replaces the live LiDAR, Collision Monitor, or Soft Stop limiter.
        if (
            self.navigation_enabled
            and getattr(self, "_mapping_guard_required", False)
            and eligible
        ):
            path_guard = MappingPathRiskGuard(occupied_clearance_m=0.48)
            grid = self.frontier_clearance_guard.grid
            grid_ready = grid is not None and path_guard.update_grid(*grid)
            path_approved = []
            # R16_172_RELEASE_STABILITY
            # Even when another candidate is approved, do not forget
            # the known-unsafe actual paths within the same revision.
            rejected = set(getattr(self, "_r16_171b_rejected", ()))
            for candidate in eligible:
                result = path_guard.check_path(
                    getattr(candidate, "path_points", None)
                    if grid_ready else None
                )
                if result.allowed:
                    path_approved.append(candidate)
                else:
                    rejected.add((round(candidate.x, 2), round(candidate.y, 2)))
                    self.get_logger().warn(
                        "Frontier rejected by Mapping path screen: "
                        f"({candidate.x:.2f}, {candidate.y:.2f}) "
                        f"{result.reason}; samples={result.samples}"
                    )
            self._r16_171b_rejected = rejected
            if not path_approved:
                # R16_171B_ROTATE_FAILED_PATHS
                # Record only goals whose actual Planner paths
                # failed the unchanged Mapping safety screen.
                rejected = getattr(
                    self, "_r16_171b_rejected", set()
                )
                rejected.update(
                    (round(goal.x, 2), round(goal.y, 2))
                    for goal in eligible
                )
                self._r16_171b_rejected = rejected

                # Defer marker reprocessing to the planning
                # timer, outside this Action result callback.
                self._r16_171b_refresh_pending = True

                self.get_logger().warn(
                    "No path-screen-qualified Frontier; "
                    "no Nav2 goal dispatched. MAP_COMPLETE not asserted. "
                    f"Excluded viewpoints={len(rejected)}; "
                    "trying unused Frontier viewpoints."
                )
                return

            eligible = path_approved

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

        # R16_171B_ROTATE_FAILED_PATHS
        # Never dispatch a rejected path. Refresh candidate
        # selection only after the Planner callback finishes.
        if getattr(
            self, "_r16_171b_refresh_pending", False
        ):
            self._r16_171b_refresh_pending = False
            marker = getattr(
                self, "_r16_171b_marker", None
            )
            if marker is not None:
                self.mapping_update_candidates(marker)
            return

        # R16_171C_EXHAUSTED_PLANNER_ROTATION
        # A planner-validation failure is not a safe path.
        # Once the existing retry budget is exhausted,
        # exclude that unchanged candidate batch and
        # search unused Frontier viewpoints instead.
        policy = getattr(
            self.planner_client, "replanning_policy", None
        )
        candidates = tuple(getattr(
            self.planner_client, "candidates", ()
        ))
        signature = getattr(
            self.planner_client, "signature", None
        )
        max_failures = getattr(
            policy, "max_failures", None
        )
        failures = getattr(
            policy, "consecutive_failures", None
        )
        current_revision = getattr(
            self.freshness_monitor, "map_revision", None
        )

        # R16_172_RELEASE_STABILITY
        # Individual NO_VALID_PATH/mismatch failures may exhaust
        # the failure-memory list before policy retries are exhausted.
        memory = getattr(self.planner_client, "failure_memory", None)
        memory_exhausted = (
            memory is not None
            and bool(candidates)
            and hasattr(memory, "available_candidates")
            and not memory.available_candidates(candidates)
        )

        exhausted = (
            not getattr(self.planner_client, "busy", False)
            and bool(candidates)
            and signature is not None
            and getattr(policy, "goal_key", None) == signature
            and getattr(
                self.planner_client,
                "completed_signature",
                None,
            ) == signature
            and type(max_failures) is int
            and max_failures > 0
            and type(failures) is int
            and (failures >= max_failures or memory_exhausted)
            and current_revision is not None
            and current_revision == getattr(
                self, "_r16_171b_revision", None
            )
        )

        if exhausted:
            marker = getattr(
                self, "_r16_171b_marker", None
            )
            if marker is not None:
                rejected = set(getattr(
                    self, "_r16_171b_rejected", ()
                ))
                rejected.update(
                    (round(x, 2), round(y, 2))
                    for x, y in candidates
                )
                self._r16_171b_rejected = rejected

                self.get_logger().warn(
                    "Planner retry budget exhausted; "
                    f"rotating {len(candidates)} "
                    "unverified viewpoints; "
                    f"excluded_total={len(rejected)}. "
                    "MAP_COMPLETE not asserted."
                )

                # Only candidate generation is retried.
                # No Nav2 motion is authorized here.
                self.mapping_update_candidates(marker)
                return

        # Continue the existing 10-second no-viewpoint
        # debounce even when the Frontier publisher is quiet.
        if (
            getattr(self, "_r16_171b_rejected", None)
            and not getattr(
                self.planner_client, "candidates", ()
            )
            and getattr(self, "_no_viable_since", None)
            is not None
        ):
            marker = getattr(
                self, "_r16_171b_marker", None
            )
            if marker is not None:
                self.mapping_update_candidates(marker)
            return

        held_revision = getattr(
            self, "_visited_only_hold_revision", None
        )
        current_revision = getattr(
            getattr(self, "freshness_monitor", None),
            "map_revision", None
        )
        if (
            held_revision is not None
            and held_revision == current_revision
        ):
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
            result = getattr(
                self.navigation_monitor,
                "last_result",
                (False, "UNKNOWN"),
            )
            reason = (
                str(result[1])
                if isinstance(result, tuple) and len(result) >= 2
                else "UNKNOWN"
            )

            now = time.monotonic()
            previous_reason = getattr(
                self, "_last_planning_block_reason", None
            )
            previous_time = getattr(
                self, "_last_planning_block_time", float("-inf")
            )

            if (
                reason != previous_reason
                or now - previous_time >= 10.0
            ):
                self.get_logger().warn(
                    f"Mapping planning safety BLOCKED: {reason}"
                )
                self._last_planning_block_reason = reason
                self._last_planning_block_time = now
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
            # A temporary SLOW/STOP person limit requires
            # a completed, acknowledged Nav2 cancellation.
            # Recovery still waits for continuous NORMAL
            # safety evidence; this does not permit motion
            # while a person limit is active.
            "PERSON_LIMIT_NOT_NORMAL",
            "TF_NOT_STABLE",
            "OBSTACLE_HOLD_NOT_CLEAR",
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


    def mapping_navigation_safety_check(self):
        """Use unchanged live gate and record rejection."""
        allowed = self.navigation_monitor.is_safe()
        if not allowed:
            self.log_mapping_safety_diagnostic("NAVIGATING")
        return allowed

    def log_mapping_safety_diagnostic(self, phase):
        """Rate-limit evidence logs, not safety decisions."""
        monitor = self.navigation_monitor
        if monitor is None:
            return

        now = time.monotonic()
        result = monitor.last_result
        reason = (
            str(result[1])
            if isinstance(result, tuple) and len(result) > 1
            else "UNKNOWN"
        )

        previous = getattr(
            self, "_mapping_diag_last", {}
        )
        last = previous.get(phase)

        if (
            last is not None
            and last[0] == reason
            and now - last[1] < 5.0
        ):
            return

        previous[phase] = (reason, now)
        self._mapping_diag_last = previous

        try:
            detail = format_mapping_safety_diagnostic(
                monitor, now, phase
            )
            self.get_logger().warn(
                "Mapping safety diagnostic: " + detail
            )
        except Exception as exc:
            self.get_logger().warn(
                "Mapping safety diagnostic unavailable: "
                f"{type(exc).__name__}"
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
                self.log_mapping_safety_diagnostic(
                    "RECOVERY"
                )
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
            MappingState.EXPLORATION_BLOCKED,
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

        if getattr(self, "_r16_214_resume_pending", False):
            from serving_robot_mapping.mapping_visit_resume import (
                restore_visits,
            )

            # Any invalid previous-map evidence raises and stops
            # this Explorer rather than silently repeating goals.
            restored = restore_visits(
                self._r16_214_resume_report,
                msg.data,
                int(msg.info.width),
                int(msg.info.height),
                float(msg.info.resolution),
                float(origin.position.x),
                float(origin.position.y),
                origin_yaw,
            )
            self.visited_frontiers = restored
            self._r16_214_resume_pending = False
            self.get_logger().info(
                "R16-214 VISITED_HISTORY_RESTORED: "
                f"{len(restored)} successful prior goals."
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
            MappingState.EXPLORATION_BLOCKED,
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

            # A blocked explorer can re-enter normal zero-frontier
            # completion checking when the frontier count vanishes.
            if (
                self.state_machine.state
                == MappingState.EXPLORATION_BLOCKED
            ):
                self.state_machine.transition_to(
                    MappingState.EXPLORING
                )
                self._no_viable_since = None
                self.get_logger().info(
                    "Frontiers disappeared; resuming "
                    "independent completion checking."
                )

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

        if current_state == MappingState.COMPLETION_CHECK:
            verified, reason = verify_closed_free_region(
                data=msg.data,
                width=width,
                height=height,
                resolution=resolution,
                free_threshold=self.frontier_detector.free_threshold,
                min_free_area_m2=10.0,
            )

            if self.goal_manager.confirm_map_complete(verified):
                self.get_logger().info(
                    "MAP_COMPLETE: stable zero Frontiers and "
                    "independent closed-free-region verification passed."
                )
            elif reason != getattr(
                self, "_last_completion_block_reason", None
            ):
                self._last_completion_block_reason = reason
                self.get_logger().warn(
                    "Completion verification pending: "
                    f"{reason}; MAP_COMPLETE not asserted."
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


import sys as _r16_203_sys


def _r16_203_settle_navigation(
    node,
    spin_once_fn,
    context_ok_fn,
    now_fn,
    max_wait_s=6.0,
):
    """Request cancellation while ROS context remains usable.

    Client becoming idle is not independent proof of physical
    stopping or a particular Nav2 terminal action status.
    """
    client = getattr(node, "navigation_client", None)

    if client is None or not client.busy:
        return "NO_ACTIVE_MAPPING_GOAL"

    if not context_ok_fn():
        return "CONTEXT_INVALID_CANCEL_UNCONFIRMED"

    try:
        client.cancel("Mapping Explorer shutting down")
    except Exception as exc:
        print(
            "R16-203 CANCEL_REQUEST_EXCEPTION:",
            repr(exc),
            file=_r16_203_sys.stderr,
            flush=True,
        )
        return "CANCEL_REQUEST_EXCEPTION"

    deadline = now_fn() + max_wait_s

    while (
        client.busy
        and context_ok_fn()
        and now_fn() < deadline
    ):
        node.stop_robot()

        try:
            spin_once_fn(node, timeout_sec=0.1)
        except Exception as exc:
            print(
                "R16-203 CANCEL_SPIN_EXCEPTION:",
                repr(exc),
                file=_r16_203_sys.stderr,
                flush=True,
            )
            break

    node.stop_robot()

    if not context_ok_fn():
        return "CONTEXT_INVALID_CANCEL_UNCONFIRMED"

    if client.busy:
        return "CANCEL_UNCONFIRMED"

    return "CLIENT_IDLE_TERMINAL_STATUS_NOT_INDEPENDENTLY_VERIFIED"


def main(args=None) -> None:
    """Run Explorer with context-preserving signal shutdown."""
    import signal

    from rclpy.signals import SignalHandlerOptions

    # Keep ROS context valid long enough to request cancellation
    # and process the resulting action callbacks.
    rclpy.init(
        args=args,
        signal_handler_options=SignalHandlerOptions.NO,
    )

    received = {"signal": None}

    def request_shutdown(signum, _frame):
        # Do not call ROS APIs from a Python signal handler.
        received["signal"] = signum

    original_sigint = signal.getsignal(signal.SIGINT)
    original_sigterm = signal.getsignal(signal.SIGTERM)

    signal.signal(signal.SIGINT, request_shutdown)
    signal.signal(signal.SIGTERM, request_shutdown)

    node = None
    shutdown_status = "NOT_STARTED"
    process_error = None

    try:
        node = ExplorationManager()

        while rclpy.ok() and received["signal"] is None:
            rclpy.spin_once(node, timeout_sec=0.1)

    except KeyboardInterrupt:
        received["signal"] = signal.SIGINT

    except Exception as exc:
        process_error = exc
        print(
            "R16-203 EXPLORER_SPIN_EXCEPTION:",
            repr(exc),
            file=_r16_203_sys.stderr,
            flush=True,
        )

    finally:
        try:
            if node is not None:
                shutdown_status = _r16_203_settle_navigation(
                    node,
                    rclpy.spin_once,
                    rclpy.ok,
                    time.monotonic,
                    max_wait_s=6.0,
                )

                print(
                    "R16-203 NAVIGATION_SHUTDOWN_STATUS=",
                    shutdown_status,
                    flush=True,
                )

                node.stop_robot()
                node.destroy_node()

        except Exception as exc:
            process_error = exc
            print(
                "R16-203 SHUTDOWN_EXCEPTION:",
                repr(exc),
                file=_r16_203_sys.stderr,
                flush=True,
            )

        finally:
            if rclpy.ok():
                rclpy.shutdown()

            signal.signal(signal.SIGINT, original_sigint)
            signal.signal(signal.SIGTERM, original_sigterm)

    unsafe_shutdown = shutdown_status in (
        "CONTEXT_INVALID_CANCEL_UNCONFIRMED",
        "CANCEL_REQUEST_EXCEPTION",
        "CANCEL_UNCONFIRMED",
    )

    if process_error is not None or unsafe_shutdown:
        print(
            "R16-203 EXPLORER_SHUTDOWN_UNVERIFIED",
            file=_r16_203_sys.stderr,
            flush=True,
        )
        raise SystemExit(2)


if __name__ == "__main__":
    main()
