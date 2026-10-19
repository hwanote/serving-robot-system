#!/usr/bin/env python3
"""Put a test person (Gazebo Fuel model "Standing person") in front of the robot, or remove it.

    python3 ~/serving-robot-system/scripts/dev/spawn_person.py 2.0     # 2.0 m ahead of the robot's front edge, facing the robot
    python3 ~/serving-robot-system/scripts/dev/spawn_person.py 1.2
    python3 ~/serving-robot-system/scripts/dev/spawn_person.py --at -3.2 -0.6   # person at map x y
    python3 ~/serving-robot-system/scripts/dev/spawn_person.py --remove

Download the model once first (the simulation then finds it in the local cache):
    gz fuel download -u "https://fuel.gazebosim.org/1.0/OpenRobotics/models/Standing person"

The distance is measured from the front edge of the robot body to the middle
of the person. Needs the simulation to run (operation_mode) and
`source ros_ws/install/setup.bash`.
"""
import argparse
import math
import subprocess
import sys
import time

NAME = 'test_person'
WORLD = 'cafe'
URI = 'https://fuel.gazebosim.org/1.0/OpenRobotics/models/Standing person'
FRONT_EDGE = 0.26        # robot body front edge from the base centre (footprint)


def _gz(service, reqtype, req, timeout_ms=20000):
    return subprocess.run(
        ['gz', 'service', '-s', '/world/%s/%s' % (WORLD, service),
         '--reqtype', reqtype, '--reptype', 'gz.msgs.Boolean',
         '--timeout', str(timeout_ms), '--req', req],
        capture_output=True, text=True).stdout.strip()


def remove():
    return _gz('remove', 'gz.msgs.Entity', 'name: "%s" type: MODEL' % NAME, 5000)


def spawn(x, y, yaw):
    sdf = ("<sdf version='1.9'><include><uri>%s</uri><name>%s</name>"
           "<pose>%.3f %.3f 0 0 0 %.4f</pose></include></sdf>"
           % (URI, NAME, x, y, yaw))
    return _gz('create', 'gz.msgs.EntityFactory', 'sdf: "%s"' % sdf)


def robot_pose():
    import rclpy
    from rclpy.node import Node
    from rclpy.time import Time
    from tf2_ros import Buffer, TransformException, TransformListener
    rclpy.init()
    node = Node('spawn_person_pose')
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
                   help='metres from the robot front edge to the person')
    p.add_argument('--remove', action='store_true')
    p.add_argument('--at', nargs=2, type=float, metavar=('X', 'Y'),
                   help='person position in the map frame (needs no robot position)')
    p.add_argument('--yaw-deg', type=float, default=180.0,
                   help='with --at: the direction the person faces (default 180)')
    args = p.parse_args()

    if args.remove:
        print('remove:', remove() or 'no answer (is the simulation running?)')
        return 0
    if args.at is not None:
        remove()
        print('person at x=%.2f y=%.2f' % tuple(args.at))
        print('create:', spawn(args.at[0], args.at[1], math.radians(args.yaw_deg))
              or 'no answer (is the simulation running? is the model downloaded?)')
        return 0
    if args.ahead is None or args.ahead < 0.2:
        p.error('give the distance ahead in metres (0.2 or more, e.g. 2.0), or --remove')
    pose = robot_pose()
    if pose is None:
        print('robot position unknown (no map->base_footprint). '
              'Is operation_mode running and the workspace sourced?')
        return 2
    x, y, yaw = pose
    d = FRONT_EDGE + args.ahead
    px, py = x + d * math.cos(yaw), y + d * math.sin(yaw)
    remove()
    print('robot at x=%.2f y=%.2f heading %.0f deg -> person at x=%.2f y=%.2f '
          '(%.2f m ahead of the body front)'
          % (x, y, math.degrees(yaw), px, py, args.ahead))
    print('create:', spawn(px, py, yaw + math.pi)
          or 'no answer (is the simulation running? is the model downloaded?)')
    return 0


if __name__ == '__main__':
    sys.exit(main())
