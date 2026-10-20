#!/usr/bin/env python3
"""Person state (judge node): camera observations + LiDAR -> person safety state.

    ros2 run serving_robot_navigation person_state.py --ros-args -p use_sim_time:=true

    in   /robot_01/person_observations  std_msgs/String, JSON (person_detector)
         /robot_01/scan_dynamic         sensor_msgs/LaserScan (clearance ahead)
         /robot_01/obstacle_hold        std_msgs/Bool (the LiDAR side holds the robot)
    out  /robot_01/person_detection     std_msgs/String, JSON, one per observation:
             {state, reason, source, distance_m, nearest_m, front_clear_m,
              obstacle_hold, confidence, count, bbox, frame_age_s, camera, time,
              side_m, side_camera, rear_m, silent_cameras}
         state: NORMAL / SLOW / STOP / SENSOR_STALE (see decision/person_safety.py)

The one place where the person state is decided (one judge, however many
cameras). The primary camera's observations drive the state and the answer:
if the camera node dies, person_detection stops too and the speed limiter /
route executor see "detector silent". The other cameras add the direction
rules (beside: SLOW only; behind: no effect) and their health (silent_cameras),
see decision/person_judge.py. It only reports: it does not touch the velocity chain.
"""
import json

import rclpy
from rclpy.node import Node
from rclpy.qos import (DurabilityPolicy, QoSProfile, ReliabilityPolicy,
                       qos_profile_sensor_data)
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool, String

from serving_robot_navigation.interfaces import names_for
from serving_robot_navigation.decision.person_judge import PersonJudge
from serving_robot_navigation.decision.person_safety import PersonSafetyCore
from serving_robot_navigation.monitoring.clearance import forward_clearance
from serving_robot_navigation.monitoring.person_observation import parse_observation


class PersonState(Node):
    def __init__(self):
        super().__init__('person_state')
        d = self.declare_parameter
        nm = names_for(d('robot_name', '').value)   # '' = default_robot
        d('observation_topic', nm.topic('person_observations'))
        d('out_topic', nm.topic('person_detection'))
        d('hold_topic', nm.topic('obstacle_hold'))
        d('scan_topic', nm.topic('scan_dynamic'))
        d('stop_m', 0.8)
        d('slow_m', 1.5)
        d('hysteresis_m', 0.2)
        d('release_frames', 3)
        d('median_n', 3)
        d('stale_s', 2.0)
        d('lost_hold_s', 3.0)
        d('front_m', 0.26)           # same as soft_stop_limiter
        d('half_width_m', 0.20)
        d('clear_m', 1.0)
        d('clear_hold_s', 2.0)
        d('scan_max_age_s', 0.5)
        d('primary_camera', 'camera_front')
        d('cameras', ['camera_front', 'camera_rear_left', 'camera_rear_right'])
        d('front_half_deg', 60.0)
        d('side_max_deg', 120.0)
        d('side_slow_m', 0.8)
        d('side_release_s', 1.0)
        g = lambda n: self.get_parameter(n).value  # noqa: E731
        self.g = g

        self.judge = PersonJudge(PersonSafetyCore(
            stop_m=float(g('stop_m')), slow_m=float(g('slow_m')),
            hysteresis_m=float(g('hysteresis_m')),
            release_frames=int(g('release_frames')),
            median_n=int(g('median_n')), stale_s=float(g('stale_s')),
            lost_hold_s=float(g('lost_hold_s')),
            clear_m=float(g('clear_m')), clear_hold_s=float(g('clear_hold_s'))),
            primary_camera=str(g('primary_camera')), cameras=list(g('cameras')),
            front_half_deg=float(g('front_half_deg')),
            side_max_deg=float(g('side_max_deg')),
            side_slow_m=float(g('side_slow_m')),
            side_release_s=float(g('side_release_s')),
            camera_stale_s=float(g('stale_s')))
        self.ext_hold = False
        self.clear = None            # clearance ahead from the scan
        self.clear_t = None
        self.last_state = None
        self.pub = self.create_publisher(String, g('out_topic'), 10)
        self.create_subscription(String, g('observation_topic'), self._on_obs, 10)
        self.create_subscription(LaserScan, g('scan_topic'), self._on_scan,
                                 qos_profile_sensor_data)
        latch = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                           reliability=ReliabilityPolicy.RELIABLE)
        self.create_subscription(Bool, g('hold_topic'), self._on_hold, latch)
        core = self.judge.core
        self.get_logger().info(
            'person state ready: STOP < %.2f m, SLOW < %.2f m, from %s'
            % (core.stop_m, core.slow_m, g('observation_topic')))

    def _now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def _on_scan(self, msg):
        self.clear = forward_clearance(
            msg.ranges, msg.angle_min, msg.angle_increment, msg.range_min,
            msg.range_max, float(self.g('front_m')), float(self.g('half_width_m')))
        self.clear_t = self._now()

    def _front_clear(self, now):
        if self.clear is None or now - self.clear_t > float(self.g('scan_max_age_s')):
            return None             # unknown counts as blocked
        return self.clear

    def _on_hold(self, msg):
        self.ext_hold = bool(msg.data)

    def _on_obs(self, msg):
        obs = parse_observation(msg.data)
        if obs is None:
            self.get_logger().warning('unreadable observation: %.80s' % msg.data)
            return
        now = self._now()
        body = self.judge.on_observation(now, obs, self.ext_hold,
                                         self._front_clear(now))
        if body is None:
            return                  # another camera: remembered, the primary answers
        self.pub.publish(String(data=json.dumps(body)))
        if body['state'] != self.last_state:
            self.get_logger().info('PERSON %s -> %s (distance %s)' % (
                self.last_state, body['state'], body['distance_m']))
            self.last_state = body['state']


def main():
    rclpy.init()
    node = PersonState()
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
