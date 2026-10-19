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
from serving_robot_mapping.planning_freshness import (
    PlanningFreshnessMonitor,
)
from serving_robot_mapping.tf_freshness import (
    TfFreshnessChecker,
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
            "/robot_01/cmd_vel",
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
            self.planner_client.update_candidates,
            marker_qos,
        )

        self.planning_timer = self.create_timer(
            1.0,
            self.planning_loop,
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

    def planner_results_callback(self, verified_goals):
        """Store direct action results for preview only."""

        self.planner_verified_goal = None

        if self.state_machine.state != MappingState.EXPLORING:
            return

        if not self.last_frontier_count:
            return

        selected = self.goal_manager.select_goal(
            verified_goals
        )

        if selected is None:
            return

        self.planner_verified_goal = selected

        self.get_logger().info(
            "ACTION-VERIFIED PREVIEW: "
            f"({selected.x:.2f}, {selected.y:.2f}), "
            f"path={selected.path_length_m:.2f} m"
        )

        self.get_logger().info(
            "Navigation remains disabled."
        )

    def planning_loop(self):
        """Run planning only when explicitly enabled."""

        if not self.planning_enabled:
            return

        if self.state_machine.state != MappingState.EXPLORING:
            return

        if not self.last_frontier_count:
            return

        self.planner_client.tick()

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
        """Command zero velocity."""
        self.cmd_vel_pub.publish(Twist())

    def destroy_node(self) -> bool:
        """Stop the robot before node shutdown."""
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
        node.get_logger().info(
            "Exploration Manager interrupted by user"
        )

    finally:
        for _ in range(10):
            node.stop_robot()

        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
