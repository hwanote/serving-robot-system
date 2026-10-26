#!/usr/bin/env python3
"""Check on the running robot that the names and types of docs/INTERFACES.md exist.

    source ~/serving-robot-system/ros_ws/install/setup.bash
    python3 ~/serving-robot-system/scripts/dev/check_interfaces_live.py
    python3 .../check_interfaces_live.py --robot robot_02 --no-person

Reads the names from config/common/interfaces.yaml, then asks ROS 2:
  - every topic / service is in `ros2 topic list -t` / `ros2 service list -t`
    with the promised type
  - the three JSON state topics (person_detection, person_limit, mission_status)
    really send a message within --wait seconds (default 20: the simulation
    runs slower than real time, and a fresh `ros2 topic echo` needs a while to
    find the publisher)
Prints one line per name and exits 0 when all is well, 1 otherwise.
--no-person: the robot runs with person_limit:=false (no detector / limiter).
"""
import argparse
import os
import subprocess
import sys

ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..'))
sys.path.insert(0, os.path.join(ROOT, 'ros_ws', 'src', 'serving_robot_navigation'))
from serving_robot_navigation.interfaces import names_for  # noqa: E402

TYPES = {
    'scan': 'sensor_msgs/msg/LaserScan',
    'scan_dynamic': 'sensor_msgs/msg/LaserScan',
    'camera_image': 'sensor_msgs/msg/Image',
    'camera_depth': 'sensor_msgs/msg/Image',
    'camera_info': 'sensor_msgs/msg/CameraInfo',
    'camera_rear_left_image': 'sensor_msgs/msg/Image',
    'camera_rear_left_depth': 'sensor_msgs/msg/Image',
    'camera_rear_left_info': 'sensor_msgs/msg/CameraInfo',
    'camera_rear_right_image': 'sensor_msgs/msg/Image',
    'camera_rear_right_depth': 'sensor_msgs/msg/Image',
    'camera_rear_right_info': 'sensor_msgs/msg/CameraInfo',
    'contact': 'ros_gz_interfaces/msg/Contacts',
    'odom': 'nav_msgs/msg/Odometry',
    'person_observations': 'std_msgs/msg/String',
    'person_detection': 'std_msgs/msg/String',
    'person_limit': 'std_msgs/msg/String',
    'obstacle_hold': 'std_msgs/msg/Bool',
    'collision_latched': 'std_msgs/msg/Bool',
    'health': 'std_msgs/msg/String',
    'cmd_vel': 'geometry_msgs/msg/Twist',
    'mission_cmd': 'std_msgs/msg/String',
    'mission_status': 'std_msgs/msg/String',
    'alarm': 'std_msgs/msg/String',
}
SERVICE_TYPE = 'std_srvs/srv/Trigger'
PERSON_ONLY = {'person_observations', 'person_detection', 'person_limit'}
SENDING = ('person_observations', 'person_detection', 'person_limit', 'mission_status',
           'health')


def listing(kind):
    try:
        out = subprocess.run(['ros2', kind, 'list', '-t'], capture_output=True,
                             text=True, timeout=30).stdout
    except FileNotFoundError:
        raise SystemExit('ros2 not found: source ros_ws/install/setup.bash first')
    found = {}
    for line in out.splitlines():
        if ' [' in line:
            name, typ = line.split(' [', 1)
            found[name.strip()] = typ.rstrip(']').strip()
    return found


def sends(topic, wait_s):
    try:
        r = subprocess.run(['ros2', 'topic', 'echo', '--once', topic, '--field',
                            'data'], capture_output=True, text=True,
                           timeout=wait_s)
        return r.returncode == 0 and bool(r.stdout.strip())
    except subprocess.TimeoutExpired:
        return False


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawTextHelpFormatter)
    ap.add_argument('--robot', default='')
    ap.add_argument('--no-person', action='store_true')
    ap.add_argument('--wait', type=float, default=20.0,
                    help='seconds to wait for a message per state topic')
    args = ap.parse_args()
    nm = names_for(args.robot)
    topics, services = listing('topic'), listing('service')
    bad = 0

    def line(ok, kind, name, note):
        nonlocal bad
        bad += 0 if ok else 1
        print('%-4s %-8s %-38s %s' % ('ok' if ok else 'FAIL', kind, name, note))

    for key, name in nm.all_topics().items():
        if args.no_person and key in PERSON_ONLY:
            continue
        typ = topics.get(name)
        if typ is None:
            line(False, 'topic', name, 'not found')
        elif typ != TYPES[key]:
            line(False, 'topic', name, 'type %s, expected %s' % (typ, TYPES[key]))
        else:
            line(True, 'topic', name, typ)
    for key, name in nm.all_services().items():
        typ = services.get(name)
        line(typ == SERVICE_TYPE, 'service', name,
             typ or 'not found')
    for key in SENDING:
        if args.no_person and key in PERSON_ONLY:
            continue
        name = nm.topic(key)
        line(sends(name, args.wait), 'message', name,
             'a message arrives within %.0f s' % args.wait)
    print('interfaces live: %s' % ('all good' if not bad else '%d problem(s)' % bad))
    return 1 if bad else 0


if __name__ == '__main__':
    raise SystemExit(main())
