"""Publish detected map frontiers as RViz visualization markers."""

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
from visualization_msgs.msg import Marker

from serving_robot_mapping.frontier_detector import FrontierDetector


class FrontierVisualizer(Node):
    """Visualize detected frontier cells without moving the robot."""

    def __init__(self) -> None:
        super().__init__("frontier_visualizer")

        self.declare_parameter("map_topic", "/map")
        self.declare_parameter(
            "marker_topic", "/mapping/frontiers"
        )
        self.declare_parameter("min_size_m", 0.40)
        self.declare_parameter("free_threshold", 25)

        self.map_topic = str(
            self.get_parameter("map_topic").value
        )
        self.marker_topic = str(
            self.get_parameter("marker_topic").value
        )

        self.detector = FrontierDetector(
            min_size_m=float(
                self.get_parameter("min_size_m").value
            ),
            free_threshold=int(
                self.get_parameter("free_threshold").value
            ),
        )

        # Subscribe to live SLAM map updates.
        map_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )

        # Keep the latest marker available for RViz.
        marker_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )

        self.marker_pub = self.create_publisher(
            Marker,
            self.marker_topic,
            marker_qos,
        )

        self.map_sub = self.create_subscription(
            OccupancyGrid,
            self.map_topic,
            self.map_callback,
            map_qos,
        )

        self.get_logger().info(
            f"Frontier Visualizer started. Map: {self.map_topic}"
        )
        self.get_logger().info(
            f"Publishing markers: {self.marker_topic}"
        )

    def map_callback(self, msg: OccupancyGrid) -> None:
        width = int(msg.info.width)
        height = int(msg.info.height)
        resolution = float(msg.info.resolution)

        try:
            clusters = self.detector.detect(
                data=msg.data,
                width=width,
                height=height,
                resolution=resolution,
            )
        except ValueError as exc:
            self.get_logger().warn(
                f"Invalid occupancy grid: {exc}"
            )
            return

        origin = msg.info.origin.position
        q = msg.info.origin.orientation

        # OccupancyGrid origin can have a yaw rotation.
        origin_yaw = math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z),
        )

        cos_yaw = math.cos(origin_yaw)
        sin_yaw = math.sin(origin_yaw)

        marker = Marker()
        marker.header = msg.header

        if not marker.header.frame_id:
            marker.header.frame_id = "map"

        marker.ns = "frontier_cells"
        marker.id = 0
        marker.type = Marker.POINTS
        marker.pose.orientation.w = 1.0

        marker.scale.x = resolution
        marker.scale.y = resolution

        # Bright orange frontier cells.
        marker.color.r = 1.0
        marker.color.g = 0.45
        marker.color.b = 0.0
        marker.color.a = 1.0

        cell_count = 0

        for cluster in clusters:
            for cell_x, cell_y in cluster.cells:
                # Cell center in local map coordinates.
                local_x = (cell_x + 0.5) * resolution
                local_y = (cell_y + 0.5) * resolution

                # Convert to world coordinates using map origin.
                world_x = (
                    origin.x
                    + cos_yaw * local_x
                    - sin_yaw * local_y
                )

                world_y = (
                    origin.y
                    + sin_yaw * local_x
                    + cos_yaw * local_y
                )

                point = Point()
                point.x = world_x
                point.y = world_y
                point.z = origin.z + 0.05

                marker.points.append(point)
                cell_count += 1

        # Delete an earlier marker when no frontier remains.
        marker.action = (
            Marker.ADD if cell_count > 0 else Marker.DELETE
        )

        self.marker_pub.publish(marker)

        self.get_logger().info(
            f"Map: {width}x{height}, "
            f"resolution={resolution:.3f} m | "
            f"Frontiers: {len(clusters)} clusters, "
            f"{cell_count} cells"
        )


def main(args=None) -> None:
    rclpy.init(args=args)
    node = FrontierVisualizer()

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
