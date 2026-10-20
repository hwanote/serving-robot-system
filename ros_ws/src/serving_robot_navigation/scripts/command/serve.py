#!/usr/bin/env python3
"""Serve one table through the route executor (test client for the serving sequence).

    ros2 run serving_robot_navigation serve.py table_01
        kitchen -> waits for the load -> table -> waits for the handover -> home
    ros2 run serving_robot_navigation serve.py --confirm load       (at the kitchen)
    ros2 run serving_robot_navigation serve.py --confirm handover   (at the table,
                                         or after a failed delivery to release the hold)

It prints the alarms and the state of the mission. Ctrl-C cancels the mission.
Exit code: 0 the mission succeeded (or the confirmation was accepted),
1 failed / returned to the kitchen / refused, 2 executor not running.
"""
import argparse
import json
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions
from std_msgs.msg import String
from std_srvs.srv import Trigger

from serving_robot_navigation.interfaces import names_for


def _confirm(which, service):
    # no rclpy SIGINT handler: it shuts the context down before our
    # KeyboardInterrupt handler can send the cancel (context invalid)
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = Node('serve_confirm')
    client = node.create_client(Trigger, service)
    if not client.wait_for_service(timeout_sec=5.0):
        print('route executor not running (no service %s)' % service)
        node.destroy_node()
        rclpy.shutdown()
        return 2
    fut = client.call_async(Trigger.Request())
    rclpy.spin_until_future_complete(node, fut, timeout_sec=5.0)
    res = fut.result()
    node.destroy_node()
    rclpy.shutdown()
    if res is None:
        print('no answer from the route executor')
        return 2
    print('%s: %s' % ('OK' if res.success else 'REFUSED', res.message))
    return 0 if res.success else 1


def _serve(table, cmd_topic, status_topic, alarm_topic):
    # no rclpy SIGINT handler: it shuts the context down before our
    # KeyboardInterrupt handler can send the cancel (context invalid)
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = Node('serve_cli')
    pub = node.create_publisher(String, cmd_topic, 10)
    mission_id = 'serve-%d' % int(time.time())
    box = {'result': None, 'state': '', 'rejected': None}

    def on_status(msg):
        try:
            s = json.loads(msg.data)
        except ValueError:
            return
        if s.get('mission_id') != mission_id:
            return
        if s.get('state') != box['state']:
            box['state'] = s.get('state')
            print('   state %s (destination %s)' % (box['state'], s.get('destination')),
                  flush=True)
        if s.get('result'):
            box['result'] = s

    def on_alarm(msg):
        try:
            a = json.loads(msg.data)
        except ValueError:
            return
        print('   ALARM %s: %s' % (a.get('code'), a.get('text')), flush=True)
        if a.get('rejected_id') == mission_id:
            box['rejected'] = a             # our order was refused: nothing runs

    node.create_subscription(String, status_topic, on_status, 10)
    node.create_subscription(String, alarm_topic, on_alarm, 10)
    end = time.monotonic() + 10.0
    while pub.get_subscription_count() == 0 and time.monotonic() < end:
        rclpy.spin_once(node, timeout_sec=0.2)
    if pub.get_subscription_count() == 0:
        print('route executor not running (no listener on %s)' % cmd_topic)
        node.destroy_node()
        rclpy.shutdown()
        return 2
    pub.publish(String(data=json.dumps(
        {'cmd': 'serve', 'id': mission_id, 'table': table})))
    print('serve %s sent (%s)' % (table, mission_id), flush=True)
    try:
        while box['result'] is None and box['rejected'] is None:
            rclpy.spin_once(node, timeout_sec=0.5)
    except KeyboardInterrupt:
        print('   cancelling the mission', flush=True)
        # with our id: never cancels another client's mission
        pub.publish(String(data=json.dumps({'cmd': 'cancel', 'id': mission_id})))
        rclpy.spin_once(node, timeout_sec=0.5)
        node.destroy_node()
        rclpy.shutdown()
        return 1
    if box['rejected'] is not None:
        print('   REJECTED %s (nothing was started)' % box['rejected'].get('code'),
              flush=True)
        node.destroy_node()
        rclpy.shutdown()
        return 1
    s = box['result']
    print('   %s %s' % (s['result'], (s.get('code') or '') + ' ' + (s.get('text') or '')),
          flush=True)
    node.destroy_node()
    rclpy.shutdown()
    return 0 if s['result'] == 'SUCCEEDED' else 1


def main():
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawTextHelpFormatter)
    p.add_argument('table', nargs='?', help='table name, e.g. table_01')
    p.add_argument('--confirm', choices=['load', 'handover'])
    p.add_argument('--robot', default='',
                   help='robot name (default: default_robot of interfaces.yaml)')
    p.add_argument('--cmd-topic', default=None)
    p.add_argument('--status-topic', default=None)
    p.add_argument('--alarm-topic', default=None)
    args, _ = p.parse_known_args(sys.argv[1:])
    nm = names_for(args.robot)
    args.cmd_topic = args.cmd_topic or nm.topic('mission_cmd')
    args.status_topic = args.status_topic or nm.topic('mission_status')
    args.alarm_topic = args.alarm_topic or nm.topic('alarm')
    if args.confirm:
        return _confirm(args.confirm, nm.service('confirm_' + args.confirm))
    if not args.table:
        p.print_help()
        return 2
    return _serve(args.table, args.cmd_topic, args.status_topic, args.alarm_topic)


if __name__ == '__main__':
    raise SystemExit(main())
