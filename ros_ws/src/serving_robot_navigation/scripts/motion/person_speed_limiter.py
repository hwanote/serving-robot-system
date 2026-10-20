#!/usr/bin/env python3
"""Semantic speed limit: the person safety state limits the velocity command.

    velocity_smoother -> cmd_vel_smoothed -> person_speed_limiter
        -> cmd_vel_semantic -> collision_monitor -> ... -> base

Reads the person state of person_state (/robot_01/person_detection, JSON) and clamps
the command: NORMAL passes, SLOW limits the speeds, STOP blocks forward driving
(backing up and turning stay allowed at the SLOW caps, so detours work),
SENSOR_STALE / a silent detector limits like SLOW and logs a warning.
This node never publishes the base cmd_vel (the soft stop limiter stays the only
owner of it) and never judges by itself. See serving_robot_navigation/motion/person_limit.py.
If this node stops, the collision monitor receives no commands and the robot
stands still (fails safe).
"""
import json

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String

from serving_robot_navigation.interfaces import names_for
from serving_robot_navigation.motion.person_limit import (
    NORMAL, STALE, STOP, PersonLimitCore, parse_message)
from serving_robot_navigation.monitoring.clearance import forward_clearance


class PersonSpeedLimiter(Node):
    def __init__(self):
        super().__init__('person_speed_limiter')
        d = self.declare_parameter
        nm = names_for(d('robot_name', '').value)   # '' = default_robot
        d('in_topic', 'cmd_vel_smoothed')
        d('out_topic', 'cmd_vel_semantic')
        d('person_topic', nm.topic('person_detection'))
        d('state_topic', nm.topic('person_limit'))
        d('slow_linear_mps', 0.10)
        d('slow_angular_rps', 0.40)
        d('message_timeout_s', 1.0)
        d('scan_topic', nm.topic('scan_dynamic'))
        d('front_m', 0.26)           # same as soft_stop_limiter
        d('half_width_m', 0.20)
        d('clear_m', 1.0)

        g = lambda n: self.get_parameter(n).value  # noqa: E731
        self.core = PersonLimitCore(
            float(g('slow_linear_mps')), float(g('slow_angular_rps')),
            float(g('message_timeout_s')), clear_m=float(g('clear_m')))
        self.front = float(g('front_m'))
        self.half_w = float(g('half_width_m'))
        self.last_state = None
        self.silent_warned = False
        self.pub = self.create_publisher(Twist, g('out_topic'), 10)
        self.create_subscription(Twist, g('in_topic'), self._on_cmd, 10)
        self.create_subscription(String, g('person_topic'), self._on_person, 10)
        self.state_pub = self.create_publisher(String, g('state_topic'), 10)
        self.create_subscription(LaserScan, g('scan_topic'), self._on_scan,
                                 qos_profile_sensor_data)
        self.create_timer(0.5, self._watch)
        self.get_logger().info(
            'person speed limiter: SLOW <= %.2f m/s and %.2f rad/s, STOP = no forward '
            '(back up / turn allowed at the same caps), '
            'detector silent for %.1f s = SLOW limit (a silent STOP holds while the LiDAR sees something ahead)' % (
                g('slow_linear_mps'), g('slow_angular_rps'),
                g('message_timeout_s')))

    def _now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def _on_person(self, msg):
        parsed = parse_message(msg.data)
        if parsed is not None:
            self.core.on_message(self._now(), parsed[0], parsed[1])
            self._publish_state()

    def _on_scan(self, msg):
        self.core.set_clearance(self._now(), forward_clearance(
            msg.ranges, msg.angle_min, msg.angle_increment, msg.range_min,
            msg.range_max, self.front, self.half_w))

    def _on_cmd(self, msg):
        lin, ang, state = self.core.limit(
            self._now(), msg.linear.x, msg.angular.z)
        out = Twist()
        out.linear.x = float(lin)
        out.angular.z = float(ang)
        self.pub.publish(out)
        self._log_state(state)

    def _publish_state(self):
        """What is applied now: /robot_01/person_limit (JSON: state, silent, source).
        The route executor and the UI read this, not the detector, so the
        speed limit and everyone else see the same decision."""
        now = self._now()
        body = self.core.status(now)
        body['time'] = round(now, 1)
        self.state_pub.publish(String(data=json.dumps(body)))

    def _watch(self):
        """Also log while no command arrives (e.g. the detector just died)."""
        now = self._now()
        self._publish_state()
        self._log_state(self.core.effective_state(now))
        silent = self.core.is_silent(now)
        if silent and self.core.state == STOP and not self.silent_warned:
            self.silent_warned = True
            self.get_logger().warning(
                'detector silent while STOP: holding STOP only while the LiDAR sees '
                'something ahead (the way ahead clear = SLOW limit)')
        elif not silent:
            self.silent_warned = False

    def _log_state(self, state):
        if state == self.last_state:
            return
        previous, self.last_state = self.last_state, state
        # separate call sites: rclpy does not allow a call site to change severity
        if state == STALE:
            self.get_logger().warning(
                'PERSON LIMIT %s -> %s: no usable person state, limiting '
                'like SLOW' % (previous, state))
        elif state == STOP:
            self.get_logger().info(
                'PERSON LIMIT %s -> %s: no forward driving' % (previous, state))
        elif state == NORMAL:
            self.get_logger().info(
                'PERSON LIMIT %s -> %s: speed unchanged' % (previous, state))
        else:
            self.get_logger().info(
                'PERSON LIMIT %s -> %s: speed limited' % (previous, state))


def main():
    rclpy.init()
    node = PersonSpeedLimiter()
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
