#!/usr/bin/env python3
"""Turn in place out of an obstacle hold (mapping mode).

    /robot_01/obstacle_hold  +  /robot_01/scan_dynamic  +  navigate_to_pose status
        -> hold_recovery -> /cmd_vel_nav  (start of the velocity chain)

After the hold has lasted `hold_s` seconds with no navigation goal active, the
robot turns in place towards the nearest heading with enough room. The command
goes through velocity_smoother, collision_monitor and soft_stop_limiter like any
Nav2 command. Rules and limits: serving_robot_navigation/motion/hold_recovery.py.
Started by launch/motion.launch.py only with hold_recovery:=true (mapping mode).
"""
import rclpy
from action_msgs.msg import GoalStatus, GoalStatusArray
from geometry_msgs.msg import Twist
from rclpy.node import Node
from rclpy.qos import (DurabilityPolicy, QoSProfile, ReliabilityPolicy,
                       qos_profile_sensor_data)
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool

from serving_robot_navigation.interfaces import names_for
from serving_robot_navigation.monitoring.clearance import forward_clearance
from serving_robot_navigation.motion.hold_recovery import (
    HoldRecoveryCore, heading_options)

ACTIVE = (GoalStatus.STATUS_ACCEPTED, GoalStatus.STATUS_EXECUTING,
          GoalStatus.STATUS_CANCELING)


class HoldRecovery(Node):
    def __init__(self):
        super().__init__('hold_recovery')
        d = self.declare_parameter
        nm = names_for(d('robot_name', '').value)   # '' = default_robot
        d('hold_topic', nm.topic('obstacle_hold'))
        d('scan_topic', nm.topic('scan_dynamic'))
        d('cmd_topic', 'cmd_vel_nav')
        d('goal_status_topic', '/navigate_to_pose/_action/status')
        d('rate_hz', 20.0)
        d('hold_s', 5.0)
        d('resume_clear_distance_m', 1.0)    # the limiter's value (launch passes it)
        d('clear_margin_m', 0.3)
        d('ang_speed_rps', 0.4)
        d('max_rotate_s', 30.0)
        d('settle_s', 6.0)
        d('giveup_s', 10.0)
        d('front_m', 0.26)
        d('half_width_m', 0.20)

        g = lambda n: self.get_parameter(n).value  # noqa: E731
        self.core = HoldRecoveryCore(
            hold_s=float(g('hold_s')),
            target_clear_m=float(g('resume_clear_distance_m'))
            + float(g('clear_margin_m')),
            ang_speed_rps=float(g('ang_speed_rps')),
            max_rotate_s=float(g('max_rotate_s')),
            settle_s=float(g('settle_s')), giveup_s=float(g('giveup_s')))
        self.front = float(g('front_m'))
        self.half_w = float(g('half_width_m'))
        self.rotating = False

        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                             reliability=ReliabilityPolicy.RELIABLE)
        self.create_subscription(Bool, g('hold_topic'), self._on_hold, latched)
        self.create_subscription(
            LaserScan, g('scan_topic'), self._on_scan, qos_profile_sensor_data)
        self.create_subscription(
            GoalStatusArray, g('goal_status_topic'), self._on_status, latched)
        self.pub = self.create_publisher(Twist, g('cmd_topic'), 10)
        self.create_timer(1.0 / float(g('rate_hz')), self._tick)
        self.get_logger().info(
            'hold recovery: after %.1f s in a hold with no goal active, turn in '
            'place at %.2f rad/s until %.2f m is clear'
            % (g('hold_s'), g('ang_speed_rps'), self.core.target_clear_m))

    def _now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def _on_hold(self, msg):
        self.core.set_hold(msg.data)

    def _on_status(self, msg):
        self.core.set_goal_active(any(s.status in ACTIVE for s in msg.status_list))

    def _on_scan(self, msg):
        args = (msg.ranges, msg.angle_min, msg.angle_increment,
                msg.range_min, msg.range_max, self.front, self.half_w)
        self.core.set_scan(self._now(), forward_clearance(*args),
                           heading_options(*args))

    def _tick(self):
        w = self.core.step(self._now())
        for line in self.core.pop_events():
            self.get_logger().warn('hold recovery: ' + line)
        if w != 0.0:
            out = Twist()
            out.angular.z = float(w)
            self.pub.publish(out)
            self.rotating = True
        elif self.rotating:
            self.pub.publish(Twist())       # one explicit zero when the turn ends
            self.rotating = False


def main():
    rclpy.init()
    node = HoldRecovery()
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
