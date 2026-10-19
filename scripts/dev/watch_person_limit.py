#!/usr/bin/env python3
"""Watch the person state and the speed through the limit chain (one line per 0.5 s).

    python3 ~/serving-robot-system/scripts/dev/watch_person_limit.py

    state=SLOW  src=lidar  dist=1.12 | in=0.25  limited=0.10  final=0.10 m/s

in      cmd_vel_smoothed   (what Nav2 wants, after the smoother)
limited cmd_vel_semantic   (after the person speed limit)
final   /robot_01/cmd_vel  (what the robot base gets, after collision monitor + soft stop)
A value shows `-` when nothing arrived for 1 s. Needs `source ros_ws/install/setup.bash`.
"""
import json
import time

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from std_msgs.msg import String

from serving_robot_navigation.interfaces import names_for


class Watch(Node):
    def __init__(self):
        super().__init__('watch_person_limit')
        self.vals = {}
        nm = names_for('')
        self.create_subscription(String, nm.topic('person_detection'), self._person, 10)
        for key, topic in (('in', 'cmd_vel_smoothed'), ('limited', 'cmd_vel_semantic'),
                           ('final', nm.topic('cmd_vel'))):
            self.create_subscription(
                Twist, topic, lambda m, k=key: self._set(k, m.linear.x), 10)
        self.create_timer(0.5, self._print)

    def _set(self, key, value):
        self.vals[key] = (value, time.time())

    def _person(self, msg):
        try:
            self.vals['person'] = (json.loads(msg.data), time.time())
        except ValueError:
            pass

    def _get(self, key):
        v = self.vals.get(key)
        return v[0] if v and time.time() - v[1] <= 1.0 else None

    def _print(self):
        p = self._get('person') or {}
        num = lambda k: '-' if self._get(k) is None else '%.2f' % self._get(k)  # noqa: E731
        dist = p.get('nearest_m') if p.get('nearest_m') is not None else p.get('front_clear_m')
        print('state=%-12s src=%-6s dist=%-5s | in=%-5s limited=%-5s final=%-5s m/s' % (
            p.get('state', '-'), p.get('source') or '-',
            '-' if dist is None else '%.2f' % dist, num('in'), num('limited'), num('final')),
            flush=True)


def main():
    rclpy.init()
    node = Watch()
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
