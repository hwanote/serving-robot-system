"""Hold map-to-odom correction when the loaded map provides little local evidence.

This node never commands velocity. If the map changes, it withholds TF until an
explicit new initial pose is supplied; Nav2 then stops through its normal TF
timeout rather than driving with a transform from the previous map.
"""
import math
import zlib

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from geometry_msgs.msg import PoseWithCovarianceStamped, TransformStamped
from nav_msgs.msg import OccupancyGrid, Odometry
from tf2_ros import TransformBroadcaster


def heading(q):
    return math.atan2(
        2.0 * (q.w * q.z + q.x * q.y),
        1.0 - 2.0 * (q.y * q.y + q.z * q.z),
    )


def normalize(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def known_fraction(grid, x, y, radius):
    info = grid.info
    resolution = info.resolution
    if resolution <= 0.0:
        return 0.0
    origin = info.origin.position
    angle = -heading(info.origin.orientation)
    dx, dy = x - origin.x, y - origin.y
    gx = (math.cos(angle) * dx - math.sin(angle) * dy) / resolution
    gy = (math.sin(angle) * dx + math.cos(angle) * dy) / resolution
    cx, cy = math.floor(gx), math.floor(gy)
    cells = max(1, math.ceil(radius / resolution))
    known = total = 0
    for row in range(cy - cells, cy + cells + 1, 2):
        if row < 0 or row >= info.height:
            continue
        for col in range(cx - cells, cx + cells + 1, 2):
            if col < 0 or col >= info.width:
                continue
            if (col + 0.5 - gx) ** 2 + (row + 0.5 - gy) ** 2 > cells ** 2:
                continue
            total += 1
            if grid.data[row * info.width + col] >= 0:
                known += 1
    return known / total if total else 0.0


class MapOdomGuard(Node):
    def __init__(self):
        super().__init__("map_odom_guard")
        for name, default in (
            ("robot_id", "robot_01"),
            ("initial_map_x", 0.0),
            ("initial_map_y", 0.0),
            ("initial_map_yaw", 0.0),
            ("known_radius_m", 0.75),
            ("min_known_fraction", 0.2),
        ):
            self.declare_parameter(name, default)
        self.robot = self.get_parameter("robot_id").value
        self.tx = float(self.get_parameter("initial_map_x").value)
        self.ty = float(self.get_parameter("initial_map_y").value)
        self.da = float(self.get_parameter("initial_map_yaw").value)
        self.radius = float(self.get_parameter("known_radius_m").value)
        self.minimum = float(self.get_parameter("min_known_fraction").value)
        if self.radius <= 0.0 or not 0.0 <= self.minimum <= 1.0:
            raise ValueError("Invalid map observation parameters")
        self.map = None
        self.map_identity = None
        self.map_changed_at = None
        self.pending_initial_pose = False
        self.odom = None
        self.last_coverage = None
        self.accepted = 0
        self.held = 0
        self.broadcaster = TransformBroadcaster(self)
        qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.create_subscription(
            OccupancyGrid, f"/{self.robot}/map", self.on_map, qos
        )
        self.create_subscription(
            Odometry, f"/{self.robot}/odom", self.on_odom, 10
        )
        self.create_subscription(
            PoseWithCovarianceStamped,
            f"/{self.robot}/amcl_pose",
            self.on_amcl,
            10,
        )
        self.create_subscription(
            PoseWithCovarianceStamped,
            f"/{self.robot}/initialpose",
            self.on_initial_pose,
            10,
        )
        self.create_timer(0.1, self.publish)

    def on_map(self, msg):
        info = msg.info
        identity = (
            info.width,
            info.height,
            info.resolution,
            info.origin.position.x,
            info.origin.position.y,
            heading(info.origin.orientation),
            zlib.crc32(bytes(value + 1 for value in msg.data)),
        )
        if self.map_identity is not None and identity != self.map_identity:
            self.pending_initial_pose = True
            self.map_changed_at = self.get_clock().now().nanoseconds
            self.get_logger().error(
                "Map changed; withholding map-to-odom TF until a new initial pose"
            )
        self.map_identity = identity
        self.map = msg

    def on_odom(self, msg):
        p = msg.pose.pose.position
        self.odom = (p.x, p.y, heading(msg.pose.pose.orientation))

    def set_transform_from_pose(self, x, y, yaw):
        ox, oy, oa = self.odom
        self.da = normalize(yaw - oa)
        ca, sa = math.cos(self.da), math.sin(self.da)
        self.tx = x - (ca * ox - sa * oy)
        self.ty = y - (sa * ox + ca * oy)

    def on_initial_pose(self, msg):
        if not self.pending_initial_pose or self.odom is None:
            return
        if msg.header.frame_id not in ("", "map"):
            return
        p = msg.pose.pose.position
        self.set_transform_from_pose(p.x, p.y, heading(msg.pose.pose.orientation))
        self.pending_initial_pose = False
        self.get_logger().info("Accepted initial pose for the new map")

    def on_amcl(self, msg):
        if self.map is None or self.odom is None or self.pending_initial_pose:
            return
        if msg.header.frame_id not in ("", "map"):
            return
        ox, oy, _ = self.odom
        ca, sa = math.cos(self.da), math.sin(self.da)
        predicted_x = self.tx + ca * ox - sa * oy
        predicted_y = self.ty + sa * ox + ca * oy
        self.last_coverage = known_fraction(
            self.map, predicted_x, predicted_y, self.radius
        )
        if self.last_coverage < self.minimum:
            self.held += 1
            return
        p = msg.pose.pose.position
        self.set_transform_from_pose(p.x, p.y, heading(msg.pose.pose.orientation))
        self.accepted += 1

    def publish(self):
        if self.pending_initial_pose:
            return
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = "map"
        t.child_frame_id = f"{self.robot}/odom"
        t.transform.translation.x = self.tx
        t.transform.translation.y = self.ty
        t.transform.rotation.z = math.sin(self.da / 2.0)
        t.transform.rotation.w = math.cos(self.da / 2.0)
        self.broadcaster.sendTransform(t)


def main():
    rclpy.init()
    node = MapOdomGuard()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
