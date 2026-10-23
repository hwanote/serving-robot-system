"""Visualize reachable approach goals for detected frontiers."""

import math

import rclpy
from geometry_msgs.msg import Point
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from rclpy.time import Time
from tf2_ros import Buffer, TransformException, TransformListener
from visualization_msgs.msg import Marker

from serving_robot_mapping.frontier_detector import FrontierDetector
from serving_robot_mapping.frontier_selector import FrontierSelector


def quaternion_to_yaw(q):
    """Extract planar yaw from a quaternion."""
    return math.atan2(
        2.0 * (q.w * q.z + q.x * q.y),
        1.0 - 2.0 * (q.y * q.y + q.z * q.z),
    )


class FrontierGoalVisualizer(Node):
    """Publish safe grid-reachable frontier goal candidates."""

    def __init__(self):
        super().__init__("frontier_goal_visualizer")

        self.declare_parameter("topics.map", "/map")
        self.declare_parameter(
            "topics.goals", "/mapping/frontier_goals"
        )
        self.declare_parameter("frames.base", "base_footprint")

        self.declare_parameter("frontier.min_size_m", 0.40)
        self.declare_parameter("frontier.free_threshold", 25)
        self.declare_parameter(
            "frontier.goal_clearance_m", 0.35
        )
        self.declare_parameter(
            "frontier.max_approach_distance_m", 1.0
        )
        self.declare_parameter(
            "frontier.min_goal_distance_m", 0.35
        )

        self.map_topic = self.get_parameter("topics.map").value
        self.goal_topic = self.get_parameter("topics.goals").value
        self.base_frame = self.get_parameter("frames.base").value

        self.detector = FrontierDetector(
            min_size_m=float(
                self.get_parameter("frontier.min_size_m").value
            ),
            free_threshold=int(
                self.get_parameter(
                    "frontier.free_threshold"
                ).value
            ),
        )

        self.selector = FrontierSelector(
            goal_clearance_m=float(
                self.get_parameter(
                    "frontier.goal_clearance_m"
                ).value
            ),
            max_approach_distance_m=float(
                self.get_parameter(
                    "frontier.max_approach_distance_m"
                ).value
            ),
            min_goal_distance_m=float(
                self.get_parameter(
                    "frontier.min_goal_distance_m"
                ).value
            ),
            free_threshold=int(
                self.get_parameter(
                    "frontier.free_threshold"
                ).value
            ),
        )

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(
            self.tf_buffer, self
        )

        self.latest_map = None
        self.last_status = None

        map_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )

        marker_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )

        self.map_sub = self.create_subscription(
            OccupancyGrid,
            self.map_topic,
            self.map_callback,
            map_qos,
        )

        self.goal_pub = self.create_publisher(
            Marker,
            self.goal_topic,
            marker_qos,
        )

        self.timer = self.create_timer(
            1.0, self.process_map
        )

        self.get_logger().info(
            "Frontier Goal Visualizer started."
        )
        self.get_logger().info(
            f"Map topic: {self.map_topic}"
        )
        self.get_logger().info(
            f"Goal marker topic: {self.goal_topic}"
        )
        self.get_logger().info(
            f"Robot base frame: {self.base_frame}"
        )

    def report_status(self, message):
        """Log only when the status changes."""
        if message != self.last_status:
            self.get_logger().info(message)
            self.last_status = message

    def map_callback(self, msg):
        self.latest_map = msg

    def publish_goals(self, goals, frame_id, resolution):
        """Publish all valid goals as green spheres."""
        marker = Marker()

        marker.header.frame_id = frame_id
        marker.header.stamp = self.get_clock().now().to_msg()

        marker.ns = "frontier_goal_candidates"
        marker.id = 0
        marker.type = Marker.SPHERE_LIST
        marker.pose.orientation.w = 1.0

        marker.scale.x = max(0.12, resolution * 2.0)
        marker.scale.y = marker.scale.x
        marker.scale.z = 0.10

        marker.color.r = 0.1
        marker.color.g = 1.0
        marker.color.b = 0.2
        marker.color.a = 1.0

        for goal in goals:
            point = Point()
            point.x = goal.x
            point.y = goal.y
            point.z = 0.12
            marker.points.append(point)

        marker.action = (
            Marker.ADD if goals else Marker.DELETE
        )

        self.goal_pub.publish(marker)

    def process_map(self):
        msg = self.latest_map

        if msg is None:
            self.report_status("Waiting for /map...")
            return

        frame_id = msg.header.frame_id or "map"

        try:
            transform = self.tf_buffer.lookup_transform(
                frame_id,
                self.base_frame,
                Time(),
            )
        except TransformException as exc:
            self.report_status(
                f"Waiting for TF {frame_id} -> "
                f"{self.base_frame}: {exc}"
            )
            return

        robot_x = transform.transform.translation.x
        robot_y = transform.transform.translation.y

        width = int(msg.info.width)
        height = int(msg.info.height)
        resolution = float(msg.info.resolution)

        if width <= 0 or height <= 0 or resolution <= 0:
            self.report_status("Waiting for valid map metadata")
            return

        clusters = self.detector.detect(
            data=msg.data,
            width=width,
            height=height,
            resolution=resolution,
        )

        origin = msg.info.origin
        origin_yaw = quaternion_to_yaw(
            origin.orientation
        )

        try:
            goals = self.selector.select(
                data=msg.data,
                width=width,
                height=height,
                resolution=resolution,
                origin_x=origin.position.x,
                origin_y=origin.position.y,
                origin_yaw=origin_yaw,
                robot_x=robot_x,
                robot_y=robot_y,
                clusters=clusters,
            )
        except ValueError as exc:
            self.publish_goals([], frame_id, resolution)
            self.report_status(
                f"No goal candidates: {exc}"
            )
            return

        self.publish_goals(
            goals,
            frame_id,
            resolution,
        )

        status = (
            f"Frontiers: {len(clusters)} clusters | "
            f"Reachable goals: {len(goals)} | "
            f"Robot: ({robot_x:.2f}, {robot_y:.2f})"
        )

        if goals:
            best = goals[0]
            status += (
                f" | Nearest goal: "
                f"({best.x:.2f}, {best.y:.2f}), "
                f"path estimate={best.travel_distance_m:.2f} m"
            )

        self.report_status(status)


def main(args=None):
    rclpy.init(args=args)
    node = FrontierGoalVisualizer()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
