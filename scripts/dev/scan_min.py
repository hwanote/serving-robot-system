#!/usr/bin/env python3
"""Diagnosis (VM): how close is the nearest thing around the robot?

    python3 ~/serving-robot-system/scripts/dev/scan_min.py [--robot robot_01]

Reads one raw scan (what HardStop uses) and one scan_dynamic (what StopZone uses)
and prints the nearest distance, its direction (0 deg = straight ahead, + = left) and
the nearest distance per 45-degree sector. HardStop stops the robot when something is
within hard_stop_distance_m of the BODY EDGE (about 0.10 m; the lidar is inside the
body, so the scan distance is larger than the body-edge distance).
"""
import argparse
import math
import sys

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan

from serving_robot_navigation.interfaces import names_for


def summarize(scan):
    best = None
    sectors = {}
    for i, r in enumerate(scan.ranges):
        if not (scan.range_min <= r <= scan.range_max) or math.isinf(r) or math.isnan(r):
            continue
        a = math.degrees(scan.angle_min + i * scan.angle_increment)
        a = (a + 180.0) % 360.0 - 180.0
        if best is None or r < best[0]:
            best = (r, a)
        k = int(math.floor((a + 22.5) / 45.0)) % 8
        sectors[k] = min(sectors.get(k, 99.0), r)
    return best, sectors


def main():
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawTextHelpFormatter)
    p.add_argument('--robot', default='')
    args, _ = p.parse_known_args(sys.argv[1:])
    nm = names_for(args.robot)
    rclpy.init()
    node = Node('scan_min')
    got = {}
    for key in ('scan', 'scan_dynamic'):
        node.create_subscription(LaserScan, nm.topic(key),
                                 lambda m, k=key: got.setdefault(k, m),
                                 qos_profile_sensor_data)
    end = node.get_clock().now().nanoseconds + 8_000_000_000
    while len(got) < 2 and node.get_clock().now().nanoseconds < end:
        rclpy.spin_once(node, timeout_sec=0.2)
    names = ['ahead', 'ahead-left', 'left', 'behind-left', 'behind', 'behind-right',
             'right', 'ahead-right']
    # sector index k: 0 = ahead, 1 = +45 (left) ... (angles run counter-clockwise)
    order = {0: 'ahead', 1: 'ahead-left', 2: 'left', 3: 'behind-left', 4: 'behind',
             5: 'behind-right', 6: 'right', 7: 'ahead-right'}
    del names
    for key in ('scan', 'scan_dynamic'):
        if key not in got:
            print('%-12s no message in 8 s' % key)
            continue
        best, sectors = summarize(got[key])
        if best is None:
            print('%-12s nothing in range' % key)
            continue
        print('%-12s nearest %.2f m at %+.0f deg' % (key, best[0], best[1]))
        print('             ' + '  '.join(
            '%s %.2f' % (order[k], sectors[k]) for k in sorted(sectors)))
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
