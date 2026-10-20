#!/usr/bin/env python3
"""Publish /robot_01/scan_dynamic: the laser scan without known obstacles.

Laser points that land on occupied cells of the map (walls, counter, table
legs, pots) are removed; points on anything the map does not contain (chairs,
people, belongings) stay. The collision monitor's stop and slow zones read this
topic, so the robot can come close to walls (e.g. to turn at a corner node)
and still reacts to unexpected obstacles. The last-defence hard stop keeps
using the raw scan.

The map must contain fixed objects only (map the store without chairs or
people). If the map or the robot pose is not available the scan is passed
through unchanged: when in doubt, every point counts as an obstacle.
"""
import copy
import math

import numpy as np
import rclpy
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from rclpy.qos import (DurabilityPolicy, QoSProfile, ReliabilityPolicy,
                       qos_profile_sensor_data)
from rclpy.time import Time
from sensor_msgs.msg import LaserScan
from tf2_ros import Buffer, TransformException, TransformListener

from serving_robot_navigation.interfaces import names_for
from serving_robot_navigation.monitoring.scan_filter import StaticMask, filter_ranges


def _yaw(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class DynamicScanFilter(Node):
    def __init__(self):
        super().__init__('dynamic_scan_filter')
        nm = names_for(self.declare_parameter('robot_name', '').value)  # '' = default_robot
        self.declare_parameter('scan_topic', nm.topic('scan'))
        self.declare_parameter('output_topic', nm.topic('scan_dynamic'))
        self.declare_parameter('map_topic', '/map')
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('tolerance_m', 0.10)
        self.declare_parameter('occupied_threshold', 65)

        self.map_frame = self.get_parameter('map_frame').value
        self.tolerance = float(self.get_parameter('tolerance_m').value)
        self.threshold = int(self.get_parameter('occupied_threshold').value)
        self.mask = None

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        map_qos = QoSProfile(
            depth=1, reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(
            OccupancyGrid, self.get_parameter('map_topic').value,
            self._on_map, map_qos)
        self.pub = self.create_publisher(
            LaserScan, self.get_parameter('output_topic').value,
            qos_profile_sensor_data)
        self.create_subscription(
            LaserScan, self.get_parameter('scan_topic').value,
            self._on_scan, qos_profile_sensor_data)
        self.get_logger().info(
            'waiting for the map; until then the scan is passed through')

    def _on_map(self, msg):
        info = msg.info
        self.mask = StaticMask(
            msg.data, info.width, info.height, info.resolution,
            info.origin.position.x, info.origin.position.y,
            self.tolerance, self.threshold)
        self.get_logger().info(
            'map received (%dx%d, %.3f m/cell): known obstacles within %.2f m '
            'are removed from the scan'
            % (info.width, info.height, info.resolution, self.tolerance))

    def _sensor_pose(self, msg):
        """(x, y, yaw) of the laser in the map frame, or None."""
        for when in (msg.header.stamp, Time()):
            try:
                t = self.tf_buffer.lookup_transform(
                    self.map_frame, msg.header.frame_id, when)
            except TransformException:
                continue
            p, q = t.transform.translation, t.transform.rotation
            return p.x, p.y, _yaw(q)
        return None

    def _on_scan(self, msg):
        out = copy.copy(msg)
        pose = self._sensor_pose(msg) if self.mask is not None else None
        if pose is None:
            self.get_logger().warn(
                'map or robot pose not available: scan passed through '
                'unchanged', throttle_duration_sec=10.0)
        else:
            ranges, removed = filter_ranges(
                msg.ranges, msg.angle_min, msg.angle_increment,
                msg.range_min, msg.range_max, pose, self.mask)
            out.ranges = ranges.astype(np.float32).tolist()
            self.get_logger().info(
                'removed %d known-obstacle points of %d' % (
                    removed, len(msg.ranges)), throttle_duration_sec=15.0)
        self.pub.publish(out)


def main():
    rclpy.init()
    node = DynamicScanFilter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
