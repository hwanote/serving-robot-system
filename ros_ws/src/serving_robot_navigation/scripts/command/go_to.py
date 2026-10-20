#!/usr/bin/env python3
"""Send the robot to named destinations (Nav2 NavigateToPose).

    ros2 run serving_robot_navigation go_to.py table_03
    ros2 run serving_robot_navigation go_to.py table_01 kitchen standby_01
    ros2 run serving_robot_navigation go_to.py --direct table_03
    ros2 run serving_robot_navigation go_to.py --list

Several names are visited one after another (stops at the first failure).
Names and aliases come from config/common/waypoints.yaml.

The robot follows the route graph of the site: it drives a straight leg to the
next node, stops, turns in place to the heading of the next leg, and so on
(no cutting corners). The route starts at the graph node nearest to the robot.
If the robot is not near any node, or --direct is given, the destination is
sent to Nav2 as one goal (Nav2 plans its own, possibly curved, path).

With --executor the robot-side route executor (route_executor.py) does the
driving instead: it waits when something blocks the way (alarm after 15 s,
detour after 30 s, see docs/RUN_GUIDE.md) and this command only sends the
mission, prints the alarms and waits for the result.

Without --executor this is the interim client-side version (no blockage rules).
Exit code: 0 all reached, 1 a goal failed, 2 bad name / no Nav2 / no executor.
"""
import argparse
import json
import math
import sys
import time

import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions
from rclpy.time import Time
from tf2_ros import Buffer, TransformException, TransformListener

from serving_robot_navigation.interfaces import names_for
from serving_robot_navigation.site_config import SiteConfigError, load_site


def _goal(x, y, yaw_rad, frame):
    msg = NavigateToPose.Goal()
    msg.pose = PoseStamped()
    msg.pose.header.frame_id = frame
    msg.pose.pose.position.x = x
    msg.pose.pose.position.y = y
    msg.pose.pose.orientation.z = math.sin(yaw_rad / 2.0)
    msg.pose.pose.orientation.w = math.cos(yaw_rad / 2.0)
    return msg


def _robot_xy(node, tf_buffer, frame, timeout_s=5.0):
    """Robot position in the map frame, or None if it cannot be read."""
    end = time.monotonic() + timeout_s
    while time.monotonic() < end:
        rclpy.spin_once(node, timeout_sec=0.2)
        try:
            t = tf_buffer.lookup_transform(frame, 'base_footprint', Time())
            return t.transform.translation.x, t.transform.translation.y
        except TransformException:
            continue
    return None


def _drive(node, client, site, x, y, yaw_rad):
    """One Nav2 goal, waits for the result. True when it succeeded."""
    send = client.send_goal_async(_goal(x, y, yaw_rad, site.frame_id))
    rclpy.spin_until_future_complete(node, send)
    handle = send.result()
    if handle is None or not handle.accepted:
        print('   goal rejected by Nav2', flush=True)
        return False
    result = handle.get_result_async()
    last = 0.0
    try:
        while not result.done():
            rclpy.spin_once(node, timeout_sec=0.5)
            now = time.monotonic()
            if now - last > 3.0:
                last = now
                print('   ...running', flush=True)
    except KeyboardInterrupt:
        print('   cancelling', flush=True)
        rclpy.spin_until_future_complete(node, handle.cancel_goal_async(),
                                         timeout_sec=3.0)
        return False
    status = result.result().status
    ok = status == GoalStatus.STATUS_SUCCEEDED
    print('   %s' % ('SUCCEEDED' if ok else 'FAILED (status %d)' % status),
          flush=True)
    return ok


def _go(node, client, site, tf_buffer, pose, direct, max_node_dist):
    zone = site.zone_at(pose.x, pose.y)
    print('-> %s  (x=%.2f y=%.2f yaw=%.0fdeg, node %s, zone %s)'
          % (pose.name, pose.x, pose.y, pose.yaw_deg, pose.node, zone),
          flush=True)

    reason = None
    if direct:
        reason = '--direct'
    else:
        xy = _robot_xy(node, tf_buffer, site.frame_id)
        if xy is None:
            reason = 'robot position unknown (no %s->base_footprint)' \
                % site.frame_id
        else:
            start, dist = site.nearest_node(*xy)
            if dist > max_node_dist:
                reason = ('robot is %.2f m from the nearest node %s '
                          '(limit %.2f m)' % (dist, start, max_node_dist))
    if reason is not None:
        print('   direct goal to Nav2 (%s)' % reason, flush=True)
        return _drive(node, client, site, pose.x, pose.y, pose.yaw_rad)

    legs = site.route_legs(start, pose)
    if legs is None:
        print('   no route from node %s to %s on the route graph'
              % (start, pose.node), flush=True)
        return False
    print('   route from node %s: %s'
          % (start, ' -> '.join(l.node for l in legs)), flush=True)
    for i, leg in enumerate(legs, 1):
        print('   leg %d/%d: drive to node %s (x=%.2f y=%.2f), then turn to '
              '%.0fdeg' % (i, len(legs), leg.node, leg.x, leg.y, leg.yaw_deg),
              flush=True)
        if not _drive(node, client, site, leg.x, leg.y, leg.yaw_rad):
            return False
    return True


def _via_executor(names, cmd_topic, status_topic, alarm_topic):
    """Send the mission to the route executor, print alarms, wait for the end."""
    from std_msgs.msg import String
    # no rclpy SIGINT handler: it shuts the context down before our
    # KeyboardInterrupt handler can send the cancel (context invalid)
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = Node('go_to_cli')
    pub = node.create_publisher(String, cmd_topic, 10)
    mission_id = 'cli-%d' % int(time.time())
    box = {'result': None, 'rejected': None}

    def on_status(msg):
        try:
            s = json.loads(msg.data)
        except ValueError:
            return
        if s.get('mission_id') == mission_id and s.get('result'):
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
        {'cmd': 'go', 'id': mission_id, 'names': names})))
    print('mission %s sent: %s' % (mission_id, ' -> '.join(names)), flush=True)
    last = 0.0
    try:
        while box['result'] is None and box['rejected'] is None:
            rclpy.spin_once(node, timeout_sec=0.5)
            now = time.monotonic()
            if now - last > 3.0:
                last = now
                print('   ...running', flush=True)
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
    p.add_argument('names', nargs='*', help='destination names or aliases')
    p.add_argument('--list', action='store_true', help='list known names')
    p.add_argument('--config-dir', default=None)
    p.add_argument('--action', default='/navigate_to_pose')
    p.add_argument('--direct', action='store_true',
                   help='send the destination to Nav2 as one goal '
                        '(no route graph, Nav2 plans its own path)')
    p.add_argument('--executor', action='store_true',
                   help='let the robot-side route executor drive (waits and '
                        'detours when something blocks the way)')
    p.add_argument('--robot', default='',
                   help='robot name (default: default_robot of interfaces.yaml)')
    p.add_argument('--cmd-topic', default=None)
    p.add_argument('--status-topic', default=None)
    p.add_argument('--alarm-topic', default=None)
    p.add_argument('--max-node-dist', type=float, default=0.6,
                   help='robot must be this close (m) to a route node to '
                        'follow the route graph (default 0.6)')
    args, _ = p.parse_known_args(sys.argv[1:])
    nm = names_for(args.robot, args.config_dir)
    args.cmd_topic = args.cmd_topic or nm.topic('mission_cmd')
    args.status_topic = args.status_topic or nm.topic('mission_status')
    args.alarm_topic = args.alarm_topic or nm.topic('alarm')

    try:
        site = load_site(args.config_dir)
    except SiteConfigError as e:
        print('config error: %s' % e)
        return 2
    print('site config revision %s (%s)' % (site.revision, site.config_dir))

    if args.list or not args.names:
        for name in sorted(site.poses):
            pose = site.poses[name]
            print('  %-12s x=%6.2f y=%6.2f yaw=%5.0f  zone=%s'
                  % (name, pose.x, pose.y, pose.yaw_deg,
                     site.zone_at(pose.x, pose.y)))
        print('names/aliases:', ', '.join(site.names()))
        return 0

    try:
        poses = [site.resolve(n) for n in args.names]
    except SiteConfigError as e:
        print('error: %s' % e)
        return 2

    if args.executor:
        return _via_executor([p.name for p in poses], args.cmd_topic,
                             args.status_topic, args.alarm_topic)

    # no rclpy SIGINT handler: it shuts the context down before our
    # KeyboardInterrupt handler can send the cancel (context invalid)
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = Node('go_to')
    client = ActionClient(node, NavigateToPose, args.action)
    tf_buffer = Buffer()
    TransformListener(tf_buffer, node)
    if not client.wait_for_server(timeout_sec=10.0):
        print('Nav2 action server %s not available (is operation_mode running?)'
              % args.action)
        node.destroy_node()
        rclpy.shutdown()
        return 2
    code = 0
    for pose in poses:
        if not _go(node, client, site, tf_buffer, pose, args.direct,
                   args.max_node_dist):
            code = 1
            break
    node.destroy_node()
    rclpy.shutdown()
    return code


if __name__ == '__main__':
    sys.exit(main())
