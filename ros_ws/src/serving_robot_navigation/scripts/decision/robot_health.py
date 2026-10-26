#!/usr/bin/env python3
"""Robot health (judge node): one summary of the robot on /robot_01/health.

    ros2 run serving_robot_navigation robot_health.py --ros-args -p use_sim_time:=true

    in   /robot_01/scan, /robot_01/odom             (are the sensors alive)
         /robot_01/mission_status   (route_executor) /robot_01/person_limit
         /robot_01/person_detection (silent_cameras) /robot_01/obstacle_hold
         /robot_01/collision_latched                  amcl_pose (covariance)
         TF map -> base_footprint                     (pose)
    out  /robot_01/health  std_msgs/String, JSON, once a second and at once when
         robot_state / safety_state / localization_state / reasons change:
             {schema_version, robot_id, time, robot_state, safety_state,
              localization_state, pose, localization, mission, checks, reasons}

The rules are in serving_robot_navigation/decision/health.py (team enums,
Q3-3 of the integration guide). It only reports.
"""
import json
import math

import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import (DurabilityPolicy, QoSProfile, ReliabilityPolicy,
                       qos_profile_sensor_data)
from rclpy.time import Time
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool, String
from tf2_ros import Buffer, TransformException, TransformListener

from serving_robot_navigation.decision.health import HealthCore, HealthParams
from serving_robot_navigation.interfaces import names_for


class RobotHealth(Node):
    def __init__(self):
        super().__init__('robot_health')
        d = self.declare_parameter
        nm = names_for(d('robot_name', '').value)   # '' = default_robot
        defaults = HealthParams()
        d('robot_id', nm.robot)
        d('map_revision', 0)
        d('person_expected', True)
        for key in ('boot_timeout_s', 'scan_stale_s', 'odom_stale_s', 'msg_stale_s',
                    'pose_lost_s', 'ok_std_m', 'ok_std_rad', 'lost_std_m',
                    'blocked_after_s'):
            d(key, float(getattr(defaults, key)))
        d('map_frame', 'map')
        d('base_frame', 'base_footprint')
        d('rate_hz', 5.0)
        d('publish_every_s', 1.0)
        g = lambda n: self.get_parameter(n).value  # noqa: E731
        self.g = g
        self.core = HealthCore(HealthParams(
            robot_id=str(g('robot_id')), map_revision=int(g('map_revision')),
            person_expected=bool(g('person_expected')),
            **{k: float(g(k)) for k in (
                'boot_timeout_s', 'scan_stale_s', 'odom_stale_s', 'msg_stale_s',
                'pose_lost_s', 'ok_std_m', 'ok_std_rad', 'lost_std_m',
                'blocked_after_s')}))

        latch = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                           reliability=ReliabilityPolicy.RELIABLE)
        sub = self.create_subscription
        sub(LaserScan, nm.topic('scan'), lambda m: self.core.on_scan(self._now()),
            qos_profile_sensor_data)
        sub(Odometry, nm.topic('odom'), lambda m: self.core.on_odom(self._now()),
            qos_profile_sensor_data)
        sub(String, nm.topic('mission_status'),
            lambda m: self._json(m, self.core.on_mission), 10)
        sub(String, nm.topic('person_limit'),
            lambda m: self._json(m, self.core.on_person_limit), 10)
        sub(String, nm.topic('person_detection'),
            lambda m: self._json(m, self.core.on_person_detection), 10)
        sub(Bool, nm.topic('obstacle_hold'), lambda m: self.core.on_hold(m.data), latch)
        sub(Bool, nm.topic('collision_latched'),
            lambda m: self.core.on_latched(m.data), latch)
        sub(PoseWithCovarianceStamped, nm.nav2('amcl_pose'), self._on_amcl, latch)
        self.tf = Buffer()
        TransformListener(self.tf, self)
        self.pub = self.create_publisher(String, nm.topic('health'), 10)
        self.last_summary = None
        self.last_pub = None
        self.create_timer(1.0 / float(g('rate_hz')), self._tick)
        self.get_logger().info('robot health ready -> %s' % nm.topic('health'))

    def _now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def _json(self, msg, fn):
        try:
            body = json.loads(msg.data)
        except ValueError:
            return
        if isinstance(body, dict):
            fn(self._now(), body)

    def _on_amcl(self, msg):
        c = msg.pose.covariance
        self.core.on_amcl(c[0], c[7], c[35])

    def _tick(self):
        now = self._now()
        try:
            t = self.tf.lookup_transform(self.g('map_frame'), self.g('base_frame'),
                                         Time(), timeout=Duration(seconds=0.0))
            q = t.transform.rotation
            self.core.on_pose(now, t.transform.translation.x, t.transform.translation.y,
                              math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                                         1.0 - 2.0 * (q.y * q.y + q.z * q.z)))
        except TransformException:
            pass
        report = self.core.report(now)
        summary = HealthCore.summary(report)
        due = (self.last_pub is None
               or now - self.last_pub >= float(self.g('publish_every_s')))
        if summary != self.last_summary or due:
            self.pub.publish(String(data=json.dumps(report)))
            self.last_pub = now
        if summary != self.last_summary:
            self.get_logger().info('HEALTH %s safety %s localization %s %s' % (
                report['robot_state'], report['safety_state'],
                report['localization_state'], report['reasons']))
            self.last_summary = summary


def main():
    rclpy.init()
    node = RobotHealth()
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
