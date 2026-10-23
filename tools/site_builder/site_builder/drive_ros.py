"""Drive client: runs tour steps through the route executor and watches them.

Everything that decides something lives in BusDriver and works on a `Bus` (no ROS, tested with a
fake bus). RosBus at the bottom is the thin ROS 2 glue; it imports rclpy only when created, so this
module can be imported anywhere. Real driving can only be done on the machine that runs ROS.

Bus (what BusDriver needs):
  bus.sink = object with  status(d,t) alarm(d,t) person(d,t) latched(v,t) hardstop(active,t) scan(f,l,r,t)
  bus.now()  bus.spin(timeout_s)  bus.publish_cmd(dict)  bus.pose() -> (x,y,yaw_rad)|None
  bus.clear_collision() -> bool   bus.place(pose_name) -> bool   bus.listeners() -> int
"""
import json
import math
import os
import subprocess
import time

from .monitor import StepMonitor
from .runner import Driver

SETTLE_S = 3.0          # wait after a cancel for the final result
LOCALIZE_S = 15.0
RESET_WAIT_S = 10.0
CLEAR_WAIT_S = 8.0
PLACE_SETTLE_S = 3.0     # after placing: wait, then check that localization stayed on the node
PLACE_TOL_M = 0.30
POSE_WAIT_S = 10.0       # a fresh ROS node needs a moment before the TF pose can be read
LISTEN_WAIT_S = 15.0     # how long to wait for the executor to be discovered


def node_poses(waypoints, start):
    """node -> pose name to reset to. Home uses the standby pose, others the probe pose ending there."""
    out = {}
    for name, p in (waypoints.get('poses') or {}).items():
        if p.get('kind') == 'standby' and p.get('node') == start:
            out.setdefault(start, name)
    for name, p in (waypoints.get('poses') or {}).items():
        if p.get('kind') == 'probe':
            out.setdefault(p['node'], name)
    return out


class BusDriver(Driver):
    def __init__(self, bus, tour, waypoints, par, mode='sim', ask=input, log=print):
        self.bus, self.tour, self.par, self.mode, self.ask, self.log = bus, tour, par, mode, ask, log
        self.node_pose = node_poses(waypoints, tour['start'])
        self.poses = waypoints['poses']
        self.nodes = waypoints['nodes']
        self.mon = None
        self.is_latched = False
        self.state = None
        self.alarm_log = []          # (t, code)
        self.loc = None
        self.seq = 0
        bus.sink = self

    # ---- sink (called by the bus while spinning)
    def status(self, d, t):
        self.state = d.get('state', self.state)
        if d.get('localization') is not None:
            self.loc = d.get('localization')
        if self.mon:
            self.mon.on_status(d, t)

    def alarm(self, d, t):
        self.alarm_log.append((t, d.get('code')))
        if self.mon:
            self.mon.on_alarm(d, t)

    def latched(self, v, t):
        self.is_latched = bool(v)
        if self.mon:
            self.mon.on_latched(v, t)

    def person(self, d, t):
        if self.mon:
            self.mon.on_person(d.get('state', 'NORMAL'), t)

    def hardstop(self, a, t):
        if self.mon:
            self.mon.on_hardstop(a, t)

    def scan(self, f, l, r, t):
        if self.mon:
            self.mon.on_scan(f, l, r, t)

    # ---- helpers
    def _wait(self, cond, timeout_s):
        end = self.bus.now() + timeout_s
        while self.bus.now() < end:
            if cond():
                return True
            self.bus.spin(0.1)
        return cond()

    def _alarm_since(self, code, t0):
        return any(c == code and t >= t0 for t, c in self.alarm_log)

    def _clear(self):
        if not self.is_latched:
            return True
        end = self.bus.now() + CLEAR_WAIT_S
        while self.bus.now() < end:
            self.bus.clear_collision()
            self.bus.spin(0.5)
            if not self.is_latched:
                return True
        return not self.is_latched

    # ---- Driver interface
    def start(self, node):
        # a new ROS node needs a moment to discover the executor: wait, do not look once
        if not self._wait(lambda: self.bus.listeners() >= 1, LISTEN_WAIT_S):
            self.log('The route executor is not listening on mission_cmd. Start the robot side first '
                     '(with SERVING_ROBOT_CONFIG_DIR=%s).' % self.tour.get('test_config_dir'))
            return False
        if self.mode == 'manual':
            a = self.ask('First real-robot test: a person must stand next to the robot holding the '
                         'emergency stop. Speed limits stay as in runtime_limits.yaml. Ready? [y/N] ')
            if str(a).strip().lower() not in ('y', 'yes'):
                return False
        return self.recover(node, 'start')

    def recover(self, node, reason):
        pose = self.node_pose.get(node)
        if not pose:
            self.log('no pose is defined for node %s' % node)
            return False
        if not self._clear():
            self.log('the collision latch did not clear (check the robot)')
            return False
        t0 = self.bus.now()
        self.bus.publish_cmd({'cmd': 'cancel'})
        self.bus.spin(0.3)
        self.bus.publish_cmd({'cmd': 'reset', 'node': pose})
        if not self._wait(lambda: self._alarm_since('ROBOT_RESET', t0) or self._alarm_since('BAD_COMMAND', t0),
                          RESET_WAIT_S) or self._alarm_since('BAD_COMMAND', t0):
            self.log('the executor did not accept the reset to %s' % pose)
            return False
        if self.mode == 'manual':
            x, y = self.nodes[node]['x'], self.nodes[node]['y']
            a = self.ask('Put the robot on node %s (x=%.2f y=%.2f, facing %s deg), then press Enter '
                         '(q = stop): ' % (node, x, y, self.poses[pose].get('yaw_deg')))
            if str(a).strip().lower() == 'q':
                return False
        else:
            if not self.bus.place(pose):
                self.log('could not place the robot in the simulation')
                return False
        if not self._wait(lambda: self.state == 'IDLE', RESET_WAIT_S):
            self.log('the executor is not IDLE after the reset')
            return False
        self.loc = None
        if not self._wait(lambda: self.loc in ('OK', 'DEGRADED'), LOCALIZE_S):
            self.log('localization is not OK (%s) after %.0f s' % (self.loc, LOCALIZE_S))
            return False
        # Gazebo answers "true" even when it could not move the model, and AMCL was only TOLD the pose:
        # wait a moment and check that the localization still sits on the node.
        end = self.bus.now() + PLACE_SETTLE_S
        while self.bus.now() < end:
            self.bus.spin(0.2)
        p = self.bus.pose()
        end = self.bus.now() + POSE_WAIT_S
        while p is None and self.bus.now() < end:
            self.bus.spin(0.2)
            p = self.bus.pose()
        x, y = self.nodes[node]['x'], self.nodes[node]['y']
        if p is None:
            self.log('no robot pose is available %.0f s after placing it on %s%s'
                     % (POSE_WAIT_S + PLACE_SETTLE_S, node,
                        (' (TF: %s)' % self.bus.last_error) if getattr(self.bus, 'last_error', '') else ''))
            return False
        err = math.hypot(p[0] - x, p[1] - y)
        if err > PLACE_TOL_M:
            self.log('the robot is not on %s: localization says %.2f m away (limit %.2f m). '
                     'It was probably not moved: check the robot name / Gazebo.' % (node, err, PLACE_TOL_M))
            return False
        return True

    def run_step(self, step, timeout_s):
        self.seq += 1
        mid = 'drv-%d-%d' % (int(time.time()), step['i'])
        a, b = self.nodes[step['from']], self.nodes[step['to']]
        t0 = self.bus.now()
        self.mon = StepMonitor(mid, (a['x'], a['y']), (b['x'], b['y']), t0)
        try:
            self.bus.publish_cmd({'cmd': 'go', 'names': [step['probe']], 'id': mid})
            timed_out = False
            cancelled = False
            while True:
                self.bus.spin(0.1)
                now = self.bus.now()
                self.mon.tick(now)
                p = self.bus.pose()
                if p:
                    self.mon.on_pose(p[0], p[1], now)
                if self.mon.finished():
                    break
                why = self.mon.cancel_reason()
                if why:
                    self.log('   cancel: %s' % why)
                    self.bus.publish_cmd({'cmd': 'cancel', 'id': mid})
                    cancelled = True
                    break
                if now - t0 > timeout_s:
                    self.log('   cancel: no result after %.0f s' % timeout_s)
                    self.bus.publish_cmd({'cmd': 'cancel', 'id': mid})
                    timed_out = cancelled = True
                    break
            if cancelled:
                self._wait(lambda: self.mon.finished() and self.state == 'IDLE', SETTLE_S)
            p = self.bus.pose()
            if p:
                self.mon.on_pose(p[0], p[1], self.bus.now())
            return self.mon.observation(self.bus.now(), timed_out)
        finally:
            self.mon = None


# ------------------------------------------------------------------ real ROS glue
class RosBus:
    """Needs ROS 2 (rclpy). Topic names come from interfaces.yaml of the drive-test folder."""

    def __init__(self, robot, config_dir, world='cafe'):
        os.environ['SERVING_ROBOT_CONFIG_DIR'] = config_dir
        import rclpy
        from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy, qos_profile_sensor_data
        from std_msgs.msg import String, Bool
        from std_srvs.srv import Trigger
        from nav2_msgs.msg import CollisionMonitorState
        from sensor_msgs.msg import LaserScan
        import tf2_ros
        from serving_robot_navigation.interfaces import names_for
        from serving_robot_navigation.site_config import load_site

        self.sink, self.world = None, world
        self.last_error = ''
        rclpy.init()
        self.rclpy = rclpy
        self.node = rclpy.create_node('site_builder_drive')
        n = names_for(robot, config_dir)
        self.robot = n.robot            # '' means the default robot of interfaces.yaml, not an empty name
        self.site = load_site(config_dir)
        self.String, self.Trigger = String, Trigger
        latch = QoSProfile(depth=1, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
                           reliability=QoSReliabilityPolicy.RELIABLE)
        self.cmd = self.node.create_publisher(String, n.topic('mission_cmd'), 10)
        self.node.create_subscription(String, n.topic('mission_status'), lambda m: self._json('status', m), 10)
        self.node.create_subscription(String, n.topic('alarm'), lambda m: self._json('alarm', m), 10)
        self.node.create_subscription(String, n.topic('person_limit'), lambda m: self._json('person', m), 10)
        self.node.create_subscription(Bool, n.topic('collision_latched'), self._latched, latch)
        self.node.create_subscription(CollisionMonitorState, n.nav2('collision_monitor_state'), self._cms, 10)
        self.node.create_subscription(LaserScan, n.topic('scan'), self._scan, qos_profile_sensor_data)
        self.cli = self.node.create_client(Trigger, n.service('clear_collision'))
        self.tfb = tf2_ros.Buffer()
        self.tfl = tf2_ros.TransformListener(self.tfb, self.node)

    def _t(self):
        return self.now()

    def _json(self, kind, m):
        try:
            d = json.loads(m.data)
        except ValueError:
            return
        if self.sink:
            getattr(self.sink, kind)(d, self._t())

    def _latched(self, m):
        if self.sink:
            self.sink.latched(m.data, self._t())

    def _cms(self, m):
        if self.sink:
            self.sink.hardstop(m.polygon_name == 'HardStop' and m.action_type == 1, self._t())

    def _scan(self, m):
        from .monitor import sector_mins
        if self.sink:
            f, l, r = sector_mins(m.ranges, m.angle_min, m.angle_increment, m.range_min, m.range_max)
            self.sink.scan(f, l, r, self._t())

    def now(self):
        return time.monotonic()

    def spin(self, timeout_s):
        self.rclpy.spin_once(self.node, timeout_sec=timeout_s)

    def publish_cmd(self, d):
        self.cmd.publish(self.String(data=json.dumps(d)))

    def listeners(self):
        return self.cmd.get_subscription_count()

    def pose(self):
        try:
            tf = self.tfb.lookup_transform(self.site.frame_id, 'base_footprint', self.rclpy.time.Time())
        except Exception as e:      # noqa: BLE001 - kept so the driver can say why there is no pose
            self.last_error = '%s: %s' % (type(e).__name__, e)
            return None
        self.last_error = ''
        t = tf.transform
        yaw = 2 * math.atan2(t.rotation.z, t.rotation.w)
        return t.translation.x, t.translation.y, yaw

    def clear_collision(self):
        if not self.cli.wait_for_service(timeout_sec=1.0):
            return False
        self.cli.call_async(self.Trigger.Request())
        return True

    def place(self, pose_name):
        p = self.site.resolve(pose_name)
        req = 'name: "%s", position: {x: %.3f, y: %.3f, z: 0.02}, orientation: {z: %.5f, w: %.5f}' % (
            self.robot, p.x, p.y, math.sin(p.yaw_rad / 2), math.cos(p.yaw_rad / 2))
        r = subprocess.run(['gz', 'service', '-s', '/world/%s/set_pose' % self.world, '--reqtype',
                            'gz.msgs.Pose', '--reptype', 'gz.msgs.Boolean', '--timeout', '3000',
                            '--req', req], capture_output=True, text=True, timeout=15)
        return 'true' in r.stdout.lower()

    def close(self):
        self.node.destroy_node()
        self.rclpy.shutdown()
