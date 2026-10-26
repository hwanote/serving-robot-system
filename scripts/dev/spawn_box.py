#!/usr/bin/env python3
"""Put a test box in front of the robot in Gazebo (or remove it) - for obstacle tests.

    python3 ~/serving-robot-system/scripts/dev/spawn_box.py 0.7      # box 0.7 m ahead of the robot's front
    python3 ~/serving-robot-system/scripts/dev/spawn_box.py 1.2
    python3 ~/serving-robot-system/scripts/dev/spawn_box.py -0.02     # box 2 cm inside the body: contact test
    python3 ~/serving-robot-system/scripts/dev/spawn_box.py --at -3.2 -0.6  # box centre at map x y (before a mission)
    python3 ~/serving-robot-system/scripts/dev/spawn_box.py --remove

The distance is measured from the front edge of the robot body to the near
face of the box, straight ahead along the robot's heading. The box is 0.3 x
0.3 m and 0.5 m high (the LiDAR looks at 0.30 m height, so it must be taller),
static, and not in the map (an "unknown" obstacle). Needs the simulation to run
(operation_mode) and `source ros_ws/install/setup.bash`.
"""
import argparse
import math
import subprocess
import sys
import time

NAME = 'test_box'
WORLD = 'cafe'
SIZE = (0.3, 0.3, 0.5)
FRONT_EDGE = 0.26        # robot body front edge from the base centre (footprint)


def box_center(x, y, yaw, ahead_m, size_x=SIZE[0]):
    """Map-frame centre of a box whose near face is `ahead_m` in front of the body."""
    d = FRONT_EDGE + ahead_m + size_x / 2.0
    return x + d * math.cos(yaw), y + d * math.sin(yaw)


def _gz(service, reqtype, req):
    return subprocess.run(
        ['gz', 'service', '-s', '/world/%s/%s' % (WORLD, service),
         '--reqtype', reqtype, '--reptype', 'gz.msgs.Boolean',
         '--timeout', '5000', '--req', req],
        capture_output=True, text=True).stdout.strip()


def remove():
    return _gz('remove', 'gz.msgs.Entity', 'name: "%s" type: MODEL' % NAME)


def spawn(cx, cy, yaw):
    sx, sy, sz = SIZE
    geom = '<geometry><box><size>%g %g %g</size></box></geometry>' % SIZE
    sdf = ("<sdf version='1.9'><model name='%s'><static>true</static>"
           "<pose>%.3f %.3f %.3f 0 0 %.4f</pose><link name='link'>"
           "<visual name='visual'>%s</visual>"
           "<collision name='collision'>%s</collision></link></model></sdf>"
           % (NAME, cx, cy, sz / 2.0, yaw, geom, geom))
    return _gz('create', 'gz.msgs.EntityFactory', 'sdf: "%s"' % sdf)


def robot_pose():
    import rclpy
    from rclpy.node import Node
    from rclpy.time import Time
    from tf2_ros import Buffer, TransformException, TransformListener
    rclpy.init()
    node = Node('spawn_box_pose')
    buf = Buffer()
    TransformListener(buf, node)
    end = time.monotonic() + 8.0
    pose = None
    while time.monotonic() < end and pose is None:
        rclpy.spin_once(node, timeout_sec=0.2)
        try:
            t = buf.lookup_transform('map', 'base_footprint', Time())
        except TransformException:
            continue
        q = t.transform.rotation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                         1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        pose = (t.transform.translation.x, t.transform.translation.y, yaw)
    node.destroy_node()
    rclpy.shutdown()
    return pose


def main():
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawTextHelpFormatter)
    p.add_argument('ahead', nargs='?', type=float,
                   help='metres from the robot front edge to the box')
    p.add_argument('--remove', action='store_true')
    p.add_argument('--at', nargs=2, type=float, metavar=('X', 'Y'),
                   help='box centre in the map frame (needs no robot position)')
    args = p.parse_args()

    if args.remove:
        print('remove:', remove() or 'no answer (is the simulation running?)')
        return 0
    if args.at is not None:
        remove()
        print('box centre x=%.2f y=%.2f' % tuple(args.at))
        print('create:', spawn(args.at[0], args.at[1], 0.0)
              or 'no answer (is the simulation running?)')
        return 0
    if args.ahead is None or args.ahead < -0.10:
        p.error('give the distance ahead in metres (e.g. 0.7), or --remove. '
                'A small negative value (e.g. -0.02) puts the box 2 cm into '
                'the robot body to test the contact sensor')

    pose = robot_pose()
    if pose is None:
        print('robot position unknown (no map->base_footprint). '
              'Is operation_mode running and the workspace sourced?')
        return 2
    x, y, yaw = pose
    cx, cy = box_center(x, y, yaw, args.ahead)
    remove()                                   # only one test box at a time
    print('robot at x=%.2f y=%.2f heading %.0f deg -> box centre x=%.2f y=%.2f '
          '(near face %.2f m ahead of the body front)'
          % (x, y, math.degrees(yaw), cx, cy, args.ahead))
    print('create:', spawn(cx, cy, yaw) or 'no answer (is the simulation running?)')
    return 0


if __name__ == '__main__':
    sys.exit(main())
