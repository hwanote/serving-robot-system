#!/usr/bin/env python3
"""Emergency reset: the robot was put by hand on a node, start fresh from there.

    ros2 run serving_robot_navigation reset.py kitchen
    ros2 run serving_robot_navigation reset.py standby_01 --robot robot_02
    ros2 run serving_robot_navigation reset.py kitchen --place     (simulation only)

Use it when the robot is stuck in a way nobody understands. A person puts the
robot on the numbered floor node and names it here. The route executor then drops
the mission and every hold (stop, localization, detour, timers), cancels the Nav2
goal, gives the node's pose to AMCL, and goes back to IDLE (alarm ROBOT_RESET).
New orders are accepted again when the position is not LOST.
The robot must stand within about 0.3 m of the node (the executor cannot check that
on the real robot: the person puts it there).

--place (simulation): sends the reset first (the robot stops and the old goal is
dropped), then moves the robot in Gazebo to the node, like the person would. Needs the Gazebo world running (default world: cafe).
Exit code: 0 reset done, 1 refused / no answer, 2 executor not running.
"""
import argparse
import json
import math
import subprocess
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions
from std_msgs.msg import String

from serving_robot_navigation.interfaces import names_for
from serving_robot_navigation.site_config import SiteConfigError, load_site


def place_in_gazebo(pose, robot, world):
    """Move the robot model in Gazebo to the pose (what the person does by hand)."""
    req = ('name: "%s", position: {x: %.3f, y: %.3f, z: 0.02}, '
           'orientation: {z: %.5f, w: %.5f}'
           % (robot, pose.x, pose.y, math.sin(pose.yaw_rad / 2.0),
              math.cos(pose.yaw_rad / 2.0)))
    try:
        r = subprocess.run(
            ['gz', 'service', '-s', '/world/%s/set_pose' % world,
             '--reqtype', 'gz.msgs.Pose', '--reptype', 'gz.msgs.Boolean',
             '--timeout', '3000', '--req', req],
            capture_output=True, text=True, timeout=15)
    except (OSError, subprocess.TimeoutExpired) as e:
        print('cannot move the robot in Gazebo: %s' % e)
        return False
    ok = 'true' in r.stdout.lower()
    if not ok:
        print('Gazebo did not move the robot: %s' % (r.stdout + r.stderr).strip())
    return ok


def _reset(node_name, cmd_topic, status_topic, alarm_topic, after_ack=None):
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = Node('reset_cli')
    pub = node.create_publisher(String, cmd_topic, 10)
    box = {'status': None, 'done': None, 'bad': None}

    def on_status(msg):
        try:
            box['status'] = json.loads(msg.data)
        except ValueError:
            pass

    def on_alarm(msg):
        try:
            a = json.loads(msg.data)
        except ValueError:
            return
        print('   ALARM %s: %s' % (a.get('code'), a.get('text')), flush=True)
        if a.get('code') == 'ROBOT_RESET':
            box['done'] = a
        elif a.get('code') == 'BAD_COMMAND':
            box['bad'] = a

    node.create_subscription(String, status_topic, on_status, 10)
    node.create_subscription(String, alarm_topic, on_alarm, 10)

    def finish(code):
        node.destroy_node()
        rclpy.shutdown()
        return code

    end = time.monotonic() + 10.0
    while (pub.get_subscription_count() == 0 or box['status'] is None) \
            and time.monotonic() < end:
        rclpy.spin_once(node, timeout_sec=0.2)
    if pub.get_subscription_count() == 0 or box['status'] is None:
        print('route executor not running (no listener on %s / no status)' % cmd_topic)
        return finish(2)
    pub.publish(String(data=json.dumps({'cmd': 'reset', 'node': node_name})))
    print('reset sent (node %s)' % node_name, flush=True)
    end = time.monotonic() + 5.0
    while time.monotonic() < end:
        rclpy.spin_once(node, timeout_sec=0.2)
        if box['bad'] is not None:
            print('REFUSED: %s' % box['bad'].get('text'))
            return finish(1)
        st = box['status'] or {}
        # the alarm comes before the next 2 Hz status: wait until it says IDLE
        if box['done'] is not None and st.get('state') == 'IDLE':
            if after_ack is not None and not after_ack():
                return finish(1)
            print('OK: reset done (state %s, localization %s)'
                  % (st.get('state'), st.get('localization')))
            return finish(0)
    print('no answer from the route executor (is it still running?)')
    return finish(1)


def main():
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawTextHelpFormatter)
    p.add_argument('node', help='name of the node / pose the robot stands on')
    p.add_argument('--robot', default='',
                   help='robot name (default: default_robot of interfaces.yaml)')
    p.add_argument('--place', action='store_true',
                   help='simulation: move the robot in Gazebo to the node first')
    p.add_argument('--world', default='cafe', help='Gazebo world name (with --place)')
    args, _ = p.parse_known_args(sys.argv[1:])
    nm = names_for(args.robot)
    try:
        pose = load_site().resolve(args.node)
    except SiteConfigError as e:
        print('REFUSED: %s' % e)
        return 1
    # --place: reset FIRST (the robot stops, the old goal is gone), then move it,
    # like a person: stop, lift, put down. Moving first would let the old goal drive
    # the robot on from the new place while this tool is still starting up.
    after = (lambda: place_in_gazebo(pose, nm.robot, args.world)) if args.place else None
    return _reset(args.node, nm.topic('mission_cmd'), nm.topic('mission_status'),
                  nm.topic('alarm'), after)


if __name__ == '__main__':
    raise SystemExit(main())
