#!/usr/bin/env python3
"""Watch what every camera sees and what the judge decides (one block per second).

    python3 ~/serving-robot-system/scripts/dev/watch_people.py

    camera_front       1 person : 0.84 m @ +38 deg            (age 0.2 s)
    camera_rear_left   0 people                                (age 0.4 s)
    camera_rear_right  0 people                                (age 0.3 s)
    judge: SLOW  source=camera  ahead=0.84  side=-  rear=-  silent=[]  | reason

distance = real distance from the camera (depth corrected with the pixel
angle), deg = direction seen from the robot (0 ahead, + left). Needs
`source ros_ws/install/setup.bash` and a running operation_mode.
"""
import json
import math

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from serving_robot_navigation.interfaces import names_for


class Watch(Node):
    def __init__(self):
        super().__init__('watch_people')
        nm = names_for('')
        self.cams = {}
        self.judge = None
        self.create_subscription(String, nm.topic('person_observations'), self._obs, 20)
        self.create_subscription(String, nm.topic('person_detection'), self._det, 10)
        self.create_timer(1.0, self._print)

    def _now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def _obs(self, msg):
        o = json.loads(msg.data)
        if o.get('new_frame'):
            self.cams[o.get('camera', '?')] = (o, self._now())

    def _det(self, msg):
        self.judge = json.loads(msg.data)

    def _print(self):
        now = self._now()
        lines = []
        for name in sorted(self.cams):
            o, t = self.cams[name]
            people = ['%s @ %s' % ('?' if p.get('distance_m') is None else '%.2f m' % p['distance_m'],
                                   '?' if p.get('bearing_rad') is None
                                   else '%+.0f deg' % math.degrees(p['bearing_rad']))
                      for p in o.get('people', [])]
            lines.append('%-18s %d %-7s: %-40s (age %.1f s)' % (
                name, len(people), 'person' if len(people) == 1 else 'people',
                ', '.join(people), now - t))
        j = self.judge
        if j:
            f = lambda v: '-' if v is None else '%.2f' % v  # noqa: E731
            lines.append('judge: %-12s source=%-6s ahead=%s  side=%s %s  rear=%s  silent=%s  | %s' % (
                j.get('state'), j.get('source') or '-', f(j.get('distance_m')),
                f(j.get('side_m')), j.get('side_camera', ''), f(j.get('rear_m')),
                j.get('silent_cameras'), j.get('reason', '')))
        else:
            lines.append('judge: (no person_detection yet)')
        print('\n'.join(lines) + '\n', flush=True)


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
