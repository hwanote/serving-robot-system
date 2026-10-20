#!/usr/bin/env python3
"""Soft stop + resume rule between the collision monitor and the robot base.

    collision_monitor -> /cmd_vel_safe -> soft_stop_limiter -> /robot_01/cmd_vel

It also latches a collision: the contact sensor (/robot_01/contact) reports a
touch -> output zero at once and stay zero until the restart service
/robot_01/clear_collision (std_srvs/Trigger) clears it.

See serving_robot_navigation/motion/soft_stop.py for the rules. If this node stops,
the robot receives no commands and stands still (fails safe).
"""
import rclpy
from geometry_msgs.msg import Twist
from nav2_msgs.msg import CollisionMonitorState
from rclpy.node import Node
from rclpy.qos import (DurabilityPolicy, QoSProfile, ReliabilityPolicy,
                       qos_profile_sensor_data)
from ros_gz_interfaces.msg import Contacts
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool
from std_srvs.srv import Trigger

from serving_robot_navigation.interfaces import names_for
from serving_robot_navigation.monitoring.clearance import (
    forward_clearance, touched_name)
from serving_robot_navigation.motion.soft_stop import LimiterCore


class SoftStopLimiter(Node):
    def __init__(self):
        super().__init__('soft_stop_limiter')
        d = self.declare_parameter
        nm = names_for(d('robot_name', '').value)   # '' = default_robot
        d('in_topic', 'cmd_vel_safe')
        d('out_topic', nm.topic('cmd_vel'))
        d('state_topic', nm.nav2('collision_monitor_state'))
        d('scan_topic', nm.topic('scan_dynamic'))
        d('rate_hz', 50.0)
        d('lin_accel_mps2', 0.40)
        d('lin_decel_mps2', 0.30)
        d('ang_accel_rps2', 0.80)
        d('ang_decel_rps2', 0.80)
        d('cmd_timeout_s', 0.50)
        d('resume_clear_distance_m', 1.0)
        d('resume_hold_s', 2.0)
        d('front_m', 0.26)           # body front edge from the base centre
        d('half_width_m', 0.20)
        d('contact_topic', nm.topic('contact'))
        d('clear_service', nm.service('clear_collision'))
        d('latched_topic', nm.topic('collision_latched'))
        d('hold_topic', nm.topic('obstacle_hold'))
        d('collision_release_s', 3.0)
        d('self_model', nm.robot)

        g = lambda n: self.get_parameter(n).value  # noqa: E731
        self.core = LimiterCore(
            g('lin_accel_mps2'), g('lin_decel_mps2'),
            g('ang_accel_rps2'), g('ang_decel_rps2'),
            g('cmd_timeout_s'), g('resume_clear_distance_m'),
            g('resume_hold_s'), g('collision_release_s'))
        self.self_model = str(g('self_model'))
        self.clear_name = str(g('clear_service'))
        self.front = float(g('front_m'))
        self.half_w = float(g('half_width_m'))
        self.period = 1.0 / float(g('rate_hz'))

        self.pub = self.create_publisher(Twist, g('out_topic'), 10)
        self.create_subscription(Twist, g('in_topic'), self._on_cmd, 10)
        self.create_subscription(
            CollisionMonitorState, g('state_topic'), self._on_state, 10)
        self.create_subscription(
            LaserScan, g('scan_topic'), self._on_scan, qos_profile_sensor_data)
        self.create_subscription(Contacts, g('contact_topic'), self._on_contact, 10)
        self.create_service(Trigger, g('clear_service'), self._on_clear)
        # Late joiners (UI, fleet) get the current state at once.
        self.latched_pub = self.create_publisher(
            Bool, g('latched_topic'),
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                       reliability=ReliabilityPolicy.RELIABLE))
        self._publish_latched()
        # "Stopped for an obstacle, waiting for the way to be clear": the
        # route executor times this for its alarm / detour rules.
        self.hold_pub = self.create_publisher(
            Bool, g('hold_topic'),
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                       reliability=ReliabilityPolicy.RELIABLE))
        self.hold_pub.publish(Bool(data=False))
        self.last = self._now()
        self.was_holding = False
        self.create_timer(self.period, self._tick)
        self.get_logger().info(
            'soft stop limiter: decel %.2f m/s2, resume after %.1f m clear for '
            '%.1f s' % (g('lin_decel_mps2'), g('resume_clear_distance_m'),
                        g('resume_hold_s')))

    def _now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def _on_cmd(self, msg):
        self.core.set_cmd(self._now(), msg.linear.x, msg.angular.z)

    def _on_state(self, msg):
        self.core.on_state(msg.polygon_name, msg.action_type)

    def _on_scan(self, msg):
        c = forward_clearance(
            msg.ranges, msg.angle_min, msg.angle_increment,
            msg.range_min, msg.range_max, self.front, self.half_w)
        self.core.set_clearance(self._now(), c)

    def _publish_latched(self):
        self.latched_pub.publish(Bool(data=self.core.latched))

    def _on_contact(self, msg):
        name = touched_name(msg.contacts, self.self_model)
        if not name:
            return
        if self.core.on_contact(self._now(), name):
            self._publish_latched()
            self.get_logger().error(
                'COLLISION #%d: the robot touched %s -> stopped and latched. '
                'Clear it with: ros2 service call %s '
                'std_srvs/srv/Trigger' % (self.core.collisions, name,
                                          self.clear_name))

    def _on_clear(self, _request, response):
        ok, message = self.core.try_clear(self._now())
        response.success = ok
        response.message = message
        if ok:
            self._publish_latched()
        self.get_logger().info('clear_collision: %s (%s)' % (
            'cleared' if ok else 'refused', message))
        return response

    def _tick(self):
        now = self._now()
        dt = now - self.last
        self.last = now
        if dt <= 0.0:
            return
        lin, ang = self.core.step(now, min(dt, 0.2))
        out = Twist()
        out.linear.x = float(lin)
        out.angular.z = float(ang)
        self.pub.publish(out)
        if self.core.latched:
            return                      # the latch has its own log lines
        if self.core.holding != self.was_holding:
            self.was_holding = self.core.holding
            self.hold_pub.publish(Bool(data=self.core.holding))
            self.get_logger().info(
                'stopped for an obstacle: waiting for the way ahead to be '
                'clear' if self.core.holding else
                'way ahead has been clear: driving on')


def main():
    rclpy.init()
    node = SoftStopLimiter()
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
