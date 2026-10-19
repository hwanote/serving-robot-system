#!/usr/bin/env python3
"""Print /robot_01/health as one short line per message.

    python3 ~/serving-robot-system/scripts/dev/watch_health.py

    t=  52.1  IDLE        safety=NORMAL  loc=OK (0.06 m, 0.03 rad)  mission=IDLE  stopped=0.0  hold=-  reasons=[]
"""
import json

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from serving_robot_navigation.interfaces import names_for


def line(h):
    loc = h.get('localization') or {}
    m = h.get('mission') or {}
    f = lambda v, u: '-' if v is None else '%.2f %s' % (v, u)  # noqa: E731
    return ('t=%6.1f  %-11s safety=%-6s  loc=%s (%s, %s)  mission=%s  stopped=%s  hold=%s  reasons=%s'
            % (h.get('time', 0.0), h.get('robot_state'), h.get('safety_state'),
               h.get('localization_state'), f(loc.get('std_xy_m'), 'm'),
               f(loc.get('std_yaw_rad'), 'rad'), m.get('state'), m.get('stopped_s'),
               m.get('loc_hold') or '-', h.get('reasons')))


class Watch(Node):
    def __init__(self):
        super().__init__('watch_health')
        self.create_subscription(String, names_for('').topic('health'), self._on, 10)

    def _on(self, msg):
        print(line(json.loads(msg.data)), flush=True)


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
