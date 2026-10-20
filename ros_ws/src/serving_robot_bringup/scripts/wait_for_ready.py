#!/usr/bin/env python3
"""Block until the requested conditions hold, then exit.

Used by operation_mode.launch.py to start each stage only when the previous
one is really ready (no fixed sleeps, so slow and fast machines both work).

Exit code: 0 = every condition met, 1 = timeout, 2 = tool missing.

Conditions (all given ones must hold; each option may be repeated):
  --clock                     /clock is being published (simulation time)
  --gz-topic TOPIC            Gazebo topic exists (`gz topic -l`)
  --topic TOPIC               a ROS publisher exists on TOPIC
  --tf PARENT CHILD           transform PARENT -> CHILD is available
  --lifecycle NODE            lifecycle node NODE is in the `active` state

Example:
  wait_for_ready.py --name robot --clock --tf odom base_footprint
"""
import argparse
import os
import subprocess
import sys
import time

import rclpy
from lifecycle_msgs.msg import State
from lifecycle_msgs.srv import GetState
from rclpy.node import Node
from rclpy.time import Time
from rosgraph_msgs.msg import Clock
from tf2_ros import Buffer, TransformListener


class Waiter(Node):
    def __init__(self, args):
        super().__init__('wait_for_ready_%s_%d' % (args.name, os.getpid()))
        self.args = args
        self.clock_seen = False
        self.gz_ok = set()
        self.gz_missing_tool = False
        self.lifecycle_active = set()
        self.lifecycle_pending = {}

        if args.clock:
            self.create_subscription(Clock, '/clock', self._on_clock, 10)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.state_clients = {
            n: self.create_client(GetState, '/%s/get_state' % n)
            for n in args.lifecycle
        }

    def _on_clock(self, _msg):
        self.clock_seen = True

    def _check_gz(self):
        todo = [t for t in self.args.gz_topic if t not in self.gz_ok]
        if not todo:
            return
        try:
            out = subprocess.run(
                ['gz', 'topic', '-l'], capture_output=True, text=True,
                timeout=5).stdout.split()
        except FileNotFoundError:
            self.gz_missing_tool = True
            return
        except subprocess.TimeoutExpired:
            return
        for t in todo:
            if t in out:
                self.gz_ok.add(t)

    def _check_lifecycle(self):
        for node, client in self.state_clients.items():
            if node in self.lifecycle_active or not client.service_is_ready():
                continue
            fut = self.lifecycle_pending.get(node)
            if fut is None:
                self.lifecycle_pending[node] = client.call_async(
                    GetState.Request())
            elif fut.done():
                res = fut.result()
                if res is not None and res.current_state.id == \
                        State.PRIMARY_STATE_ACTIVE:
                    self.lifecycle_active.add(node)
                self.lifecycle_pending.pop(node, None)

    def unmet(self):
        """Return a list of human readable, still unmet conditions."""
        a = self.args
        missing = []
        if a.clock and not self.clock_seen:
            missing.append('/clock')
        self._check_gz()
        missing += ['gz topic %s' % t for t in a.gz_topic
                    if t not in self.gz_ok]
        for t in a.topic:
            if self.count_publishers(t) == 0:
                missing.append('topic %s' % t)
        for parent, child in a.tf:
            if not self.tf_buffer.can_transform(parent, child, Time()):
                missing.append('tf %s->%s' % (parent, child))
        self._check_lifecycle()
        missing += ['active %s' % n for n in a.lifecycle
                    if n not in self.lifecycle_active]
        return missing


def main():
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawTextHelpFormatter)
    p.add_argument('--name', default='ready')
    p.add_argument('--timeout', type=float, default=180.0,
                   help='seconds (wall clock) before giving up')
    p.add_argument('--clock', action='store_true')
    p.add_argument('--gz-topic', action='append', default=[])
    p.add_argument('--topic', action='append', default=[])
    p.add_argument('--tf', action='append', nargs=2, default=[],
                   metavar=('PARENT', 'CHILD'))
    p.add_argument('--lifecycle', action='append', default=[])
    args, _ = p.parse_known_args(sys.argv[1:])

    rclpy.init()
    node = Waiter(args)
    start = last_print = time.monotonic()
    code = 1
    try:
        while True:
            rclpy.spin_once(node, timeout_sec=0.5)
            missing = node.unmet()
            if node.gz_missing_tool:
                print('[wait:%s] `gz` command not found (Gazebo installed?)'
                      % args.name, flush=True)
                code = 2
                break
            if not missing:
                print('[wait:%s] ready (%.0fs)'
                      % (args.name, time.monotonic() - start), flush=True)
                code = 0
                break
            now = time.monotonic()
            if now - start > args.timeout:
                print('[wait:%s] TIMEOUT after %.0fs, still missing: %s'
                      % (args.name, args.timeout, ', '.join(missing)),
                      flush=True)
                code = 1
                break
            if now - last_print >= 5.0:
                print('[wait:%s] waiting for: %s'
                      % (args.name, ', '.join(missing)), flush=True)
                last_print = now
    except KeyboardInterrupt:
        code = 1
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    sys.exit(code)


if __name__ == '__main__':
    main()
