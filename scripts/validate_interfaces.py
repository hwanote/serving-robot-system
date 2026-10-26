#!/usr/bin/env python3
"""Check that the files which cannot read interfaces.yaml agree with it.

    python3 scripts/validate_interfaces.py

Checked against the names of the default robot:
  - bridge.yaml: every /robot_01/... topic is known, every sensor/drive topic
    the contract promises (scan, camera_*, contact, odom, cmd_vel) is bridged
  - nav2.yaml, amcl.yaml, slam.yaml, collision_monitor.yaml,
    dynamic_scan_filter.yaml: every /robot_01/... string is a known topic
  - docs/INTERFACES.md: every /robot_01/... name is known (planned names are
    listed in PLANNED below)
Exit code 0 = consistent, 1 = problems printed.
"""
import os
import re
import sys

import yaml

ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), '..'))
sys.path.insert(0, os.path.join(ROOT, 'ros_ws', 'src', 'serving_robot_navigation'))
from serving_robot_navigation.interfaces import names_for  # noqa: E402

NAV = os.path.join(ROOT, 'ros_ws', 'src', 'serving_robot_navigation', 'config')
BRIDGE = os.path.join(ROOT, 'ros_ws', 'src', 'serving_robot_gazebo', 'config',
                      'bridge.yaml')
STATIC_YAMLS = ['nav2.yaml', 'amcl.yaml', 'slam.yaml', 'collision_monitor.yaml',
                'dynamic_scan_filter.yaml']
BRIDGED_KEYS = ['scan', 'camera_image', 'camera_depth', 'camera_info',
                'camera_rear_left_image', 'camera_rear_left_depth',
                'camera_rear_left_info', 'camera_rear_right_image',
                'camera_rear_right_depth', 'camera_rear_right_info',
                'contact', 'odom', 'cmd_vel']
PLANNED = set()                         # named in docs/INTERFACES.md, not built yet


def find(text, robot):
    return set(re.findall(r'/%s/[A-Za-z0-9_/]+' % re.escape(robot), text))


def check(config_dir=None, root=ROOT):
    problems = []
    names = names_for('', config_dir)
    robot = names.robot
    known = set(names.all_topics().values()) | set(names.all_services().values())

    with open(os.path.join(root, 'ros_ws', 'src', 'serving_robot_gazebo',
                           'config', 'bridge.yaml'), encoding='utf-8') as f:
        bridge = {e['ros_topic_name'] for e in yaml.safe_load(f)}
    for t in sorted(t for t in bridge if t.startswith('/%s/' % robot)):
        if t not in known:
            problems.append('bridge.yaml: %s is not in interfaces.yaml' % t)
    for key in BRIDGED_KEYS:
        if names.topic(key) not in bridge:
            problems.append('bridge.yaml: %s (%s) is not bridged'
                            % (names.topic(key), key))

    navdir = os.path.join(root, 'ros_ws', 'src', 'serving_robot_navigation',
                          'config')
    for fname in STATIC_YAMLS:
        path = os.path.join(navdir, fname)
        if not os.path.isfile(path):
            continue
        with open(path, encoding='utf-8') as f:
            for t in sorted(find(f.read(), robot)):
                if t not in known:
                    problems.append('%s: %s is not in interfaces.yaml'
                                    % (fname, t))

    doc = os.path.join(root, 'docs', 'INTERFACES.md')
    if os.path.isfile(doc):
        with open(doc, encoding='utf-8') as f:
            for t in sorted(find(f.read(), robot)):
                if t not in known and t.rsplit('/', 1)[-1] not in PLANNED:
                    problems.append('docs/INTERFACES.md: %s is not in '
                                    'interfaces.yaml' % t)
    return problems


def main():
    problems = check()
    for p in problems:
        print('PROBLEM', p)
    print('interfaces: %s' % ('consistent' if not problems
                              else '%d problem(s)' % len(problems)))
    return 1 if problems else 0


if __name__ == '__main__':
    raise SystemExit(main())
