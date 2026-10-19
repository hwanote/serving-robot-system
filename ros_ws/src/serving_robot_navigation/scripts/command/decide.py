#!/usr/bin/env python3
"""Answer LOCALIZATION_RECOVERED: continue the held mission or cancel it and go home.

    ros2 run serving_robot_navigation decide.py resume   drive on to the same destination
    ros2 run serving_robot_navigation decide.py home     cancel the mission, drive to home
    ros2 run serving_robot_navigation decide.py resume --robot robot_02

After a localization loss the route executor stops the robot and holds the mission
(alarm LOCALIZATION_LOST). When the position is back (OK or DEGRADED) it raises
LOCALIZATION_RECOVERED and waits for this answer: it never moves by itself.
Exit code: 0 the choice was accepted, 1 refused / nothing is waiting for a choice,
2 executor not running.
"""
import argparse
import json
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions
from std_msgs.msg import String

from serving_robot_navigation.interfaces import names_for


def _decide(choice, cmd_topic, status_topic, alarm_topic):
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = Node('decide_cli')
    pub = node.create_publisher(String, cmd_topic, 10)
    box = {'status': None, 'bad': None}

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
        if a.get('code') == 'BAD_COMMAND' and box['sent']:
            box['bad'] = a

    box['sent'] = False
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
    hold = box['status'].get('loc_hold', '')
    if hold == '':
        print('REFUSED: no mission is waiting for a localization decision')
        return finish(1)
    if hold == 'LOST':
        print('REFUSED: the position is not OK yet: wait for LOCALIZATION_RECOVERED')
        return finish(1)
    pub.publish(String(data=json.dumps({'cmd': choice})))
    box['sent'] = True
    print('%s sent' % choice, flush=True)
    end = time.monotonic() + 5.0
    while time.monotonic() < end:
        rclpy.spin_once(node, timeout_sec=0.2)
        if box['bad'] is not None:
            print('REFUSED: %s' % box['bad'].get('text'))
            return finish(1)
        if box['status'].get('loc_hold', 'x') == '':
            print('OK: %s accepted (mission %s, state %s)' % (
                choice, box['status'].get('mission_id'), box['status'].get('state')))
            return finish(0)
    print('no answer from the route executor (is it still running?)')
    return finish(1)


def main():
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawTextHelpFormatter)
    p.add_argument('choice', choices=['resume', 'home'])
    p.add_argument('--robot', default='',
                   help='robot name (default: default_robot of interfaces.yaml)')
    args, _ = p.parse_known_args(sys.argv[1:])
    nm = names_for(args.robot)
    return _decide(args.choice, nm.topic('mission_cmd'), nm.topic('mission_status'),
                   nm.topic('alarm'))


if __name__ == '__main__':
    raise SystemExit(main())
