"""Autonomous mapping workflow manager."""

import math
from typing import Optional

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node

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
        """Request the generic controller to rotate 360 degrees."""
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
