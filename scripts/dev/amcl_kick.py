#!/usr/bin/env python3
"""Test tool (VM): lose the robot's position on purpose, and give it back.

    python3 ~/serving-robot-system/scripts/dev/amcl_kick.py lost
        AMCL forgets where the robot is (global localization: the particles spread
        over the whole map) -> localization_state LOST within a second or two.
    python3 ~/serving-robot-system/scripts/dev/amcl_kick.py fix
        Tell AMCL the true pose (read from Gazebo) on /initialpose -> not LOST any more.
        --std 0.5 gives a rough pose like RViz's "2D Pose Estimate" (DEGRADED, and a
        stopped robot cannot improve it); the default 0.1 is a precise one (OK).

The robot itself is not moved. Options: --robot robot_02 (name in the Gazebo world).
"""
import argparse
import math
import re
import subprocess
import sys
import time

import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped
from rclpy.node import Node
from std_srvs.srv import Empty


def gazebo_truth(robot, world):
    """(x, y, yaw_rad) of the robot in the Gazebo world, None if not found."""
    try:
        s = subprocess.run(['gz', 'topic', '-e', '-n', '1', '-t', '/world/%s/pose/info' % world],
                           capture_output=True, text=True, timeout=25).stdout
    except subprocess.TimeoutExpired:
        return None
    m = re.search(
        r'name: "%s"\s+id: \d+\s+position \{\s+x: (\S+)\s+y: (\S+)\s+z: \S+\s+\}'
        r'\s+orientation \{\s+x: \S+\s+y: \S+\s+z: (\S+)\s+w: (\S+)' % re.escape(robot), s)
    if not m:
        return None
    x, y, z, w = map(float, m.groups())
    return x, y, 2.0 * math.atan2(z, w)


def main():
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawTextHelpFormatter)
    p.add_argument('mode', choices=['lost', 'fix'])
    p.add_argument('--robot', default='robot_01', help='robot name in the Gazebo world')
    p.add_argument('--world', default='cafe')
    p.add_argument('--std', type=float, default=0.1,
                   help='uncertainty (m and rad) given with the pose (default 0.1)')
    p.add_argument('--service', default='/reinitialize_global_localization')
    p.add_argument('--topic', default='/initialpose')
    args = p.parse_args()

    rclpy.init()
    node = Node('amcl_kick')
    code = 0
    if args.mode == 'lost':
        client = node.create_client(Empty, args.service)
        if not client.wait_for_service(timeout_sec=5.0):
            print('AMCL service %s not available' % args.service)
            code = 2
        else:
            fut = client.call_async(Empty.Request())
            rclpy.spin_until_future_complete(node, fut, timeout_sec=5.0)
            print('AMCL spread its particles: expect localization LOST in a moment'
                  if fut.done() else 'no answer from AMCL')
            code = 0 if fut.done() else 1
    else:
        truth = gazebo_truth(args.robot, args.world)
        if truth is None:
            print('cannot read the pose of %s from Gazebo (world %s)' % (args.robot, args.world))
            code = 2
        else:
            x, y, yaw = truth
            msg = PoseWithCovarianceStamped()
            msg.header.frame_id = 'map'
            msg.pose.pose.position.x = x
            msg.pose.pose.position.y = y
            msg.pose.pose.orientation.z = math.sin(yaw / 2.0)
            msg.pose.pose.orientation.w = math.cos(yaw / 2.0)
            # default std 0.1 m / 0.1 rad: well inside the health OK limits
            # (0.25 m / 0.20 rad). A stationary robot cannot make AMCL more sure
            # (it only updates after moving), so the given value is what stays.
            var = args.std * args.std
            msg.pose.covariance[0] = msg.pose.covariance[7] = var
            msg.pose.covariance[35] = var
            pub = node.create_publisher(PoseWithCovarianceStamped, args.topic, 1)
            end = time.monotonic() + 5.0
            while pub.get_subscription_count() == 0 and time.monotonic() < end:
                rclpy.spin_once(node, timeout_sec=0.1)
            for _ in range(3):                         # a few times: AMCL may be busy
                msg.header.stamp = node.get_clock().now().to_msg()
                pub.publish(msg)
                rclpy.spin_once(node, timeout_sec=0.3)
            print('true pose given to AMCL: x=%.2f y=%.2f yaw=%.0f deg (std %.2f)'
                  % (x, y, math.degrees(yaw), args.std))
    node.destroy_node()
    rclpy.shutdown()
    return code


if __name__ == '__main__':
    sys.exit(main())
