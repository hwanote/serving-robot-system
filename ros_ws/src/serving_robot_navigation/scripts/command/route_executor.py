#!/usr/bin/env python3
"""Route executor: drives the robot along the route graph and reacts to blockages.

    command   /robot_01/mission_cmd     std_msgs/String, JSON:
                  {"cmd": "go", "names": ["table_01", "kitchen"], "id": "x"}
                  {"cmd": "serve", "table": "table_01", "id": "x"}
                      kitchen -> wait for load -> table -> wait for handover -> home
                      ("home": false: the mission ends at the table, the robot stays there)
                  {"cmd": "cancel"}   {"cmd": "detour"}
                  {"cmd": "confirm_load"}   {"cmd": "confirm_handover"}
                  {"cmd": "resume"}   {"cmd": "home"}
                      after LOCALIZATION_RECOVERED: drive on / cancel and drive home
                  {"cmd": "reset", "node": "kitchen"}
                      emergency reset: the robot was put on that node by hand
                  {"cmd": "reload_map"}
                      read config/common again (only while idle; a map that does not
                      load, or lacks the home / kitchen pose, is refused and the old
                      one stays). The robot reports its map in mission_status.config_revision.
    services  /robot_01/request_detour   std_srvs/Trigger  (take another corridor now)
              /robot_01/cancel_mission   std_srvs/Trigger
              /robot_01/confirm_load     std_srvs/Trigger  (the load is on the robot)
              /robot_01/confirm_handover std_srvs/Trigger  (the order was taken)
    status    /robot_01/mission_status  std_msgs/String, JSON, twice a second
    alarms    /robot_01/alarm           std_msgs/String, JSON {code, level, text, ...}

The rules (15 s alarm, 30 s detour, ...) are in serving_robot_navigation/command/mission.py;
this node only connects them to ROS: legs along the route graph go to the Nav2
controller as straight paths (FollowPath, a pose every rail_step_m: no planner,
so no bending around an obstacle), direct goals go to NavigateToPose; TF gives
the robot pose (also its heading: turning in place counts as moving for the
"not moving" rule); and the signals of the soft
stop limiter (/robot_01/obstacle_hold, /robot_01/collision_latched) and the person
limit (/robot_01/person_limit, published by the person speed limiter): a STOP there
(a person seen by the camera or any obstacle seen by the LiDAR, `semantic_source`
tells which) is an obstacle wait too. The executor does not judge by itself, it takes
the decision that is applied to the speed.

Localization: the node reads localization_state from /robot_01/health (the one
place that judges it) and feeds it to the core. LOST while driving: stop and hold
the mission, alarm LOCALIZATION_LOST; not LOST any more (OK or DEGRADED): alarm LOCALIZATION_RECOVERED and
the operator sends "resume" or "home" (the node never moves by itself). No fresh
health message counts as unknown: nothing is stopped because of it.
"""
import json
import math

import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from nav2_msgs.action import FollowPath, NavigateToPose
from nav_msgs.msg import Path
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import (DurabilityPolicy, QoSProfile, ReliabilityPolicy)
from rclpy.time import Time
from std_msgs.msg import Bool, String
from std_srvs.srv import Trigger
from tf2_ros import Buffer, TransformException, TransformListener

from serving_robot_navigation.interfaces import names_for
from serving_robot_navigation.command.mission import (
    ALARM, ARRIVED, CANCEL_NAV, DONE, SEND, MissionCore, MissionParams)
from serving_robot_navigation.command.rail import rail_poses
from serving_robot_navigation.monitoring.detector_watch import CameraWatch, DetectorWatch
from serving_robot_navigation.site_config import SiteConfigError, load_site, map_image_fingerprint


class RouteExecutor(Node):
    def __init__(self):
        super().__init__('route_executor')
        d = self.declare_parameter
        robot = d('robot_name', '').value   # '' = default_robot of interfaces.yaml
        d('config_dir', '')
        d('map_yaml', '')               # map.yaml Nav2 loads (fingerprint -> mission_status.map_image)
        nm = names_for(robot, self.get_parameter('config_dir').value or None)
        d('cmd_topic', nm.topic('mission_cmd'))
        d('status_topic', nm.topic('mission_status'))
        d('alarm_topic', nm.topic('alarm'))
        d('hold_topic', nm.topic('obstacle_hold'))
        d('person_topic', nm.topic('person_detection'))
        d('person_limit_topic', nm.topic('person_limit'))
        d('person_limit_max_age_s', 2.0)
        d('latched_topic', nm.topic('collision_latched'))
        d('health_topic', nm.topic('health'))
        d('health_max_age_s', 3.0)      # older health message = localization unknown
        d('loc_stable_s', 2.0)          # OK this long before the operator is asked
        d('detour_service', nm.service('request_detour'))
        d('cancel_service', nm.service('cancel_mission'))
        d('confirm_load_service', nm.service('confirm_load'))
        d('confirm_handover_service', nm.service('confirm_handover'))
        d('nav_action', nm.nav2('nav_action'))
        d('follow_action', nm.nav2('follow_action'))
        d('initialpose_topic', nm.nav2('initialpose'))
        d('reset_std', 0.1)             # uncertainty given to AMCL on reset (m, rad)
        d('controller_id', 'FollowPath')
        # on in the node (off in the bare MissionParams used by the unit tests)
        d('stall_window_s', 3.0)
        d('stall_move_m', 0.05)
        d('stall_turn_rad', 0.15)
        d('off_rail_m', 0.15)
        d('rail_resume_s', 3.0)
        d('rail_step_m', 0.05)
        d('rate_hz', 10.0)
        defaults = MissionParams()
        for key in ('alarm_after_s', 'detour_after_s', 'blocked_memory_s',
                    'replan_interval_s', 'node_reach_m', 'max_node_dist_m',
                    'delivery_limit_s', 'handover_timeout_s', 'wait_repeat_s',
                    'auto_confirm_s'):
            d(key, float(getattr(defaults, key)))
        d('max_detours', int(defaults.max_detours))
        d('watch_detector', False)        # alarm when person_detection is silent
        d('detector_timeout_s', 1.0)
        d('detector_startup_grace_s', 30.0)
        d('detector_alarm_repeat_s', 30.0)
        d('nav_retries', int(defaults.nav_retries))
        d('kitchen_name', defaults.kitchen_name)
        d('home_name', defaults.home_name)
        g = lambda n: self.get_parameter(n).value  # noqa: E731

        self.site = load_site(g('config_dir') or None)
        params = MissionParams(
            alarm_after_s=float(g('alarm_after_s')),
            detour_after_s=float(g('detour_after_s')),
            blocked_memory_s=float(g('blocked_memory_s')),
            max_detours=int(g('max_detours')),
            replan_interval_s=float(g('replan_interval_s')),
            node_reach_m=float(g('node_reach_m')),
            max_node_dist_m=float(g('max_node_dist_m')),
            nav_retries=int(g('nav_retries')),
            kitchen_name=str(g('kitchen_name')), home_name=str(g('home_name')),
            handover_timeout_s=float(g('handover_timeout_s')),
            wait_repeat_s=float(g('wait_repeat_s')),
            auto_confirm_s=float(g('auto_confirm_s')),
            delivery_limit_s=float(g('delivery_limit_s')),
            stall_window_s=float(g('stall_window_s')),
            stall_move_m=float(g('stall_move_m')),
            stall_turn_rad=float(g('stall_turn_rad')),
            off_rail_m=float(g('off_rail_m')),
            rail_resume_s=float(g('rail_resume_s')),
            loc_stable_s=float(g('loc_stable_s')))
        self.core = MissionCore(self.site, params)
        self.core.map_image = map_image_fingerprint(g('map_yaml')) if g('map_yaml') else ''

        self.hold = False
        self.detector_watch = DetectorWatch(
            float(g('detector_timeout_s')), float(g('detector_startup_grace_s')),
            float(g('detector_alarm_repeat_s')))
        self.camera_watch = CameraWatch(float(g('detector_alarm_repeat_s')))
        self.silent_cameras = []        # from the last person_detection message
        self._limit = {}                # last /robot_01/person_limit message
        self._limit_t = None
        self.latched = False
        self.loc_state = None           # localization_state of the last health message
        self.loc_t = None
        self.last = {'mission_id': '', 'result': '', 'code': '', 'text': ''}
        self.nav_handle = None          # running Nav2 goal
        self.counter = 0

        latch_qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                               reliability=ReliabilityPolicy.RELIABLE)
        self.status_pub = self.create_publisher(String, g('status_topic'), 10)
        self.alarm_pub = self.create_publisher(String, g('alarm_topic'), 10)
        self.create_subscription(String, g('cmd_topic'), self._on_cmd, 10)
        self.initialpose_pub = self.create_publisher(
            PoseWithCovarianceStamped, g('initialpose_topic'), 1)
        self.create_subscription(Bool, g('hold_topic'), self._on_hold, latch_qos)
        self.create_subscription(String, g('person_topic'), self._on_person, 10)
        self.create_subscription(String, g('person_limit_topic'),
                                 self._on_person_limit, 10)
        self.create_subscription(Bool, g('latched_topic'), self._on_latched, latch_qos)
        self.create_subscription(String, g('health_topic'), self._on_health, 10)
        self.create_service(Trigger, g('detour_service'), self._on_detour)
        self.create_service(Trigger, g('cancel_service'), self._on_cancel)
        self.create_service(Trigger, g('confirm_load_service'), self._on_confirm_load)
        self.create_service(Trigger, g('confirm_handover_service'),
                            self._on_confirm_handover)
        self.nav = ActionClient(self, NavigateToPose, g('nav_action'))
        self.follow = ActionClient(self, FollowPath, g('follow_action'))
        self.tf = Buffer()
        TransformListener(self.tf, self)

        self.create_timer(1.0 / float(g('rate_hz')), self._tick)
        self.create_timer(0.5, self._publish_status)
        self.get_logger().info(
            'route executor ready: site config %s, alarm after %.0f s, detour '
            'after %.0f s' % (self.site.revision, params.alarm_after_s,
                              params.detour_after_s))

    # ----- helpers --------------------------------------------------------------
    def _now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def _pose(self):
        """(x, y, yaw) of base_footprint in the site frame, None if unknown."""
        try:
            t = self.tf.lookup_transform(self.site.frame_id, 'base_footprint',
                                         Time(), timeout=Duration(seconds=0.0))
        except TransformException:
            return None
        q = t.transform.rotation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                         1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        return t.transform.translation.x, t.transform.translation.y, yaw

    def _xy(self):
        pose = self._pose()
        return None if pose is None else pose[:2]

    def _publish_status(self):
        status = self.core.status()
        if not status['mission_id']:
            status['mission_id'] = self.last['mission_id']
        status.update(result=self.last['result'], code=self.last['code'],
                      text=self.last['text'], hold=self.hold,
                      semantic_stop=self.semantic_stop,
                      semantic_source=self.semantic_source,
                      latched=self.latched, time=round(self._now(), 1))
        self.status_pub.publish(String(data=json.dumps(status)))

    def _alarm(self, code, level, text, extra=None):
        xy = self._xy()
        body = {'code': code, 'level': level, 'text': text,
                'mission_id': self.core.mission_id or self.last['mission_id'],
                'destination': self.core.dest.name if self.core.dest else '',
                'zone': (self.site.zone_at(*xy) or '') if xy else '',
                'time': round(self._now(), 1)}
        if extra:
            body.update(extra)
        self.alarm_pub.publish(String(data=json.dumps(body)))
        # one call site per severity: rclpy forbids changing it between calls
        if level == 'warn':
            self.get_logger().warning('ALARM %s: %s' % (code, text))
        else:
            self.get_logger().info('ALARM %s: %s' % (code, text))

    # ----- inputs ------------------------------------------------------------------
    def _on_hold(self, msg):
        self.hold = bool(msg.data)

    def _on_person(self, msg):
        self.detector_watch.heard(self._now())      # only to tell the operator
        try:
            body = json.loads(msg.data)
        except ValueError:
            return
        cams = body.get('silent_cameras', []) if isinstance(body, dict) else []
        self.silent_cameras = [str(c) for c in cams] if isinstance(cams, list) else []

    def _on_person_limit(self, msg):
        try:
            body = json.loads(msg.data)
        except ValueError:
            return
        if isinstance(body, dict):
            self._limit, self._limit_t = body, self._now()

    @property
    def semantic_stop(self):
        """The person speed limiter is applying STOP right now (camera or LiDAR).
        One decision for everybody: this node does not re-derive it. No fresh
        message (limiter not running) counts as no STOP."""
        if self._limit_t is None or self._now() - self._limit_t > float(
                self.get_parameter('person_limit_max_age_s').value):
            return False
        return self._limit.get('state') == 'STOP'

    @property
    def semantic_source(self):
        return str(self._limit.get('source') or '') if self.semantic_stop else ''

    def _on_latched(self, msg):
        self.latched = bool(msg.data)

    def _on_health(self, msg):
        try:
            body = json.loads(msg.data)
        except ValueError:
            return
        if isinstance(body, dict):
            self.loc_state = str(body.get('localization_state') or '') or None
            self.loc_t = self._now()

    def _localization(self):
        """localization_state from /robot_01/health; None = unknown (health
        silent or older than health_max_age_s)."""
        if self.loc_t is None or self._now() - self.loc_t > float(
                self.get_parameter('health_max_age_s').value):
            return None
        return self.loc_state

    def _on_cmd(self, msg):
        try:
            cmd = json.loads(msg.data)
            kind = cmd['cmd']
        except (ValueError, KeyError, TypeError):
            self._alarm('BAD_COMMAND', 'warn', 'cannot read command: %s' % msg.data)
            return
        if kind == 'go':
            self._go(cmd)
        elif kind == 'serve':
            self._serve(cmd)
        elif kind in ('confirm_load', 'confirm_handover'):
            ok, text = self._confirm(kind)
            if not ok:
                self._alarm('BAD_COMMAND', 'warn', text)
        elif kind == 'cancel':
            # with "id": only that mission (a client cancels only its own)
            self._run(self.core.cancel(str(cmd.get('id') or '') or None))
        elif kind == 'detour':
            ok, text = self.core.request_detour()
            if not ok:
                self._alarm('BAD_COMMAND', 'warn', text)
        elif kind == 'resume':
            self._resume()
        elif kind == 'home':
            self._home()
        elif kind == 'reset':
            self._reset(cmd)
        elif kind == 'reload_map':
            self._reload_map()
        else:
            self._alarm('BAD_COMMAND', 'warn', 'unknown command %r' % kind)

    def _go(self, cmd):
        names = [str(n) for n in cmd.get('names', [])]
        self._launch(cmd, ' -> '.join(names),
                     lambda now, mid, xy: self.core.start(now, mid, names, xy))

    def _serve(self, cmd):
        table = str(cmd.get('table', ''))
        home = cmd.get('home', True) is not False
        self._launch(cmd, 'serve %s' % table,
                     lambda now, mid, xy: self.core.start_serve(
                         now, mid, table, xy, home=home))

    def _launch(self, cmd, label, starter):
        self.counter += 1
        mission_id = str(cmd.get('id') or 'm%d' % self.counter)
        if self.core.state != 'IDLE' or self.core.loc_refuses_orders():
            # the core answers BUSY / HANDOVER_REQUIRED while orders are held,
            # LOCALIZATION_LOST / _UNINITIALIZED while the position is unknown
            self._run(starter(self._now(), mission_id, (0.0, 0.0)))
            return
        xy = self._xy()
        if xy is None:
            self._finish(mission_id, 'FAILED', 'NO_POSE',
                         'robot position unknown (no %s->base_footprint)'
                         % self.site.frame_id)
            return
        if not (self.nav.server_is_ready() and self.follow.server_is_ready()):
            self._finish(mission_id, 'FAILED', 'NAV_UNAVAILABLE',
                         'Nav2 action server not available (%s, %s)'
                         % (self.get_parameter('nav_action').value,
                            self.get_parameter('follow_action').value))
            return
        self.last = {'mission_id': mission_id, 'result': '', 'code': '', 'text': ''}
        self.get_logger().info('mission %s: %s' % (mission_id, label))
        self._run(starter(self._now(), mission_id, xy))

    def _resume(self):
        """After LOCALIZATION_RECOVERED: drive on to the same destination."""
        xy = self._xy()
        if xy is None:
            self._alarm('BAD_COMMAND', 'warn', 'resume refused: robot position unknown')
            return
        if not (self.nav.server_is_ready() and self.follow.server_is_ready()):
            self._alarm('BAD_COMMAND', 'warn', 'resume refused: Nav2 action server '
                                               'not available')
            return
        ok, text, acts = self.core.resume(self._now(), xy)
        if not ok:
            self._alarm('BAD_COMMAND', 'warn', text)
        self._run(acts)

    def _home(self):
        """After LOCALIZATION_RECOVERED: cancel the mission, drive to home."""
        if self.core.loc_hold == 'RECOVERED' and self._xy() is None:
            self._alarm('BAD_COMMAND', 'warn', 'home refused: robot position unknown')
            return
        ok, text, acts = self.core.go_home(self._now())
        if not ok:
            self._alarm('BAD_COMMAND', 'warn', text)
            return
        self._run(acts)
        home = str(self.get_parameter('home_name').value)
        self._launch({'id': '%s_home' % text}, 'home (%s)' % home,
                     lambda now, mid, xy: self.core.start(now, mid, [home], xy))

    def _reset(self, cmd):
        """Operator reset: the robot stands on the named node. Drops the mission
        and every hold, and tells AMCL where the robot is (the node's pose)."""
        name = str(cmd.get('node', '')).strip()
        try:
            pose = self.site.resolve(name)
        except SiteConfigError as e:
            self._alarm('BAD_COMMAND', 'warn', 'reset refused: %s' % e)
            return
        _, acts = self.core.reset(self._now(), pose.name)
        self._run(acts)
        std = float(self.get_parameter('reset_std').value)
        msg = PoseWithCovarianceStamped()
        msg.header.frame_id = self.site.frame_id
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose.pose.position.x = pose.x
        msg.pose.pose.position.y = pose.y
        msg.pose.pose.orientation.z = math.sin(pose.yaw_rad / 2.0)
        msg.pose.pose.orientation.w = math.cos(pose.yaw_rad / 2.0)
        msg.pose.covariance[0] = msg.pose.covariance[7] = std * std
        msg.pose.covariance[35] = std * std
        self.initialpose_pub.publish(msg)
        self.get_logger().info('reset: robot on %s (x=%.2f y=%.2f yaw=%.0f deg), '
                               'pose given to AMCL' % (pose.name, pose.x, pose.y,
                                                       pose.yaw_deg))
        self._publish_status()

    def _reload_map(self):
        """Read config/common again (it is read once at start otherwise)."""
        if self.core.state != 'IDLE' or self.core.loc_hold:
            self._alarm('MAP_RELOAD_REFUSED', 'warn',
                        'map not reloaded: the robot is not idle')
            return
        try:
            new = load_site(self.get_parameter('config_dir').value or None)
            for key in ('home_name', 'kitchen_name'):
                new.resolve(str(self.get_parameter(key).value))
        except Exception as e:      # noqa: BLE001  any failure (bad YAML, missing file,
            # wrong shape): the old map stays, the robot keeps working
            self._alarm('MAP_RELOAD_FAILED', 'warn',
                        'map not reloaded, the old one stays: %s' % e)
            return
        old = self.site.revision
        ok, text = self.core.replace_site(new)
        if not ok:
            self._alarm('MAP_RELOAD_REFUSED', 'warn', text)
            return
        self.site = new
        self.get_logger().info('map reloaded: %s -> %s' % (old, new.revision))
        self._alarm('MAP_RELOADED', 'info',
                    'map reloaded (%s -> %s)' % (old, new.revision),
                    {'config_revision': new.revision, 'old_revision': old})
        self._publish_status()

    def _confirm(self, kind):
        fn = (self.core.confirm_load if kind == 'confirm_load'
              else self.core.confirm_handover)
        ok, text, acts = fn(self._now(), self._xy() or (0.0, 0.0))
        self._run(acts)
        return ok, text

    def _on_confirm_load(self, _req, res):
        res.success, res.message = self._confirm('confirm_load')
        return res

    def _on_confirm_handover(self, _req, res):
        res.success, res.message = self._confirm('confirm_handover')
        return res

    def _on_detour(self, _req, res):
        res.success, res.message = self.core.request_detour()
        return res

    def _on_cancel(self, _req, res):
        state = self.core.state
        acts = self.core.cancel()
        res.success = state != 'IDLE' and self.core.state == 'IDLE'
        res.message = ('canceling' if res.success else 'no mission is running'
                       if state == 'IDLE' else 'staff must confirm the handover first')
        self._run(acts)
        return res

    # ----- periodic ----------------------------------------------------------------
    def _watch_detector(self):
        """Tell the operator when person_detection is silent: person_detector
        (camera) or person_state (judge) is down (the robot keeps
        driving: LiDAR and the collision monitor still protect)."""
        for event in self.detector_watch.step(self._now()):
            if event == 'RECOVERED':
                self._alarm('DETECTOR_RECOVERED', 'info',
                            'the person detector speaks again')
            else:
                self._alarm(
                    'DETECTOR_SILENT', 'warn',
                    'the person detector is silent: the robot keeps driving with '
                    'the LiDAR and the collision monitor as protection (a held '
                    'STOP is released when the way ahead is clear); check the '
                    'camera and the person_detector / person_state nodes')
        if self.detector_watch.silent:
            return              # no fresh camera list: DETECTOR_SILENT covers it
        for event, cam in self.camera_watch.update(self._now(), self.silent_cameras):
            if event == 'RECOVERED':
                self._alarm('CAMERA_RECOVERED', 'info',
                            'camera %s sends images again' % cam, {'camera': cam})
            else:
                self._alarm('CAMERA_SILENT', 'warn',
                            'camera %s sends no images: people on that side are '
                            'not seen (the LiDAR still protects)%s' % (
                                cam, '; the front camera also limits the speed '
                                '(SENSOR_STALE)' if cam == 'camera_front' else ''),
                            {'camera': cam})

    def _tick(self):
        if self.get_parameter('watch_detector').value:
            self._watch_detector()
        self._run(self.core.on_localization(self._now(), self._localization()))
        pose = self._pose()
        if pose is None:
            return
        self._run(self.core.step(
            self._now(), pose[:2], self.hold or self.semantic_stop, self.latched,
            yaw=pose[2]))

    # ----- actions -------------------------------------------------------------------
    def _run(self, actions):
        for a in actions:
            if a.kind == SEND:
                self._send_goal(a.leg, a.gen, a.start)
            elif a.kind == ALARM:
                self._alarm(a.code, a.level, a.text, a.data or None)
            elif a.kind == ARRIVED:
                self.get_logger().info('arrived: %s' % a.text)
            elif a.kind == CANCEL_NAV:
                self._cancel_goal()
            elif a.kind == DONE:
                self._finish(self.core.mission_id or self.last['mission_id'],
                             a.result, a.code, a.text)

    def _finish(self, mission_id, result, code, text):
        self.last = {'mission_id': mission_id, 'result': result, 'code': code,
                     'text': text}
        self.get_logger().info('mission %s %s %s %s' % (mission_id, result, code, text))
        self._publish_status()

    def _cancel_goal(self):
        if self.nav_handle is not None:
            self.nav_handle.cancel_goal_async()
            self.nav_handle = None

    def _send_goal(self, leg, gen, start=None):
        self._cancel_goal()
        if start is not None:
            self._send_rail(leg, gen, start)
            return
        goal = NavigateToPose.Goal()
        goal.pose = PoseStamped()
        goal.pose.header.frame_id = self.site.frame_id
        goal.pose.pose.position.x = leg.x
        goal.pose.pose.position.y = leg.y
        goal.pose.pose.orientation.z = math.sin(leg.yaw_rad / 2.0)
        goal.pose.pose.orientation.w = math.cos(leg.yaw_rad / 2.0)
        self.get_logger().info('direct goal -> node %s (x=%.2f y=%.2f, then turn to '
                               '%.0f deg)' % (leg.node, leg.x, leg.y, leg.yaw_deg))
        fut = self.nav.send_goal_async(goal)
        fut.add_done_callback(lambda f, g=gen: self._on_accepted(f, g))

    def _send_rail(self, leg, gen, start):
        """The leg as a straight path for the controller (no planner)."""
        path = Path()
        path.header.frame_id = self.site.frame_id
        for x, y, yaw in rail_poses(start[0], start[1], leg.x, leg.y, leg.yaw_rad,
                                    float(self.get_parameter('rail_step_m').value)):
            ps = PoseStamped()
            ps.header.frame_id = self.site.frame_id
            ps.pose.position.x = x
            ps.pose.position.y = y
            ps.pose.orientation.z = math.sin(yaw / 2.0)
            ps.pose.orientation.w = math.cos(yaw / 2.0)
            path.poses.append(ps)
        goal = FollowPath.Goal()
        goal.path = path
        goal.controller_id = str(self.get_parameter('controller_id').value)
        self.get_logger().info(
            'rail -> node %s: straight %.2f m from (%.2f, %.2f), %d poses, then '
            'turn to %.0f deg' % (leg.node, math.hypot(leg.x - start[0], leg.y - start[1]),
                                  start[0], start[1], len(path.poses), leg.yaw_deg))
        fut = self.follow.send_goal_async(goal)
        fut.add_done_callback(lambda f, g=gen: self._on_accepted(f, g))

    def _on_accepted(self, fut, gen):
        handle = fut.result()
        if handle is None or not handle.accepted:
            self._nav_done(gen, False)
            return
        if gen != self.core.gen:         # replaced while it was being accepted
            handle.cancel_goal_async()
            return
        self.nav_handle = handle
        handle.get_result_async().add_done_callback(
            lambda f, g=gen: self._on_result(f, g))

    def _on_result(self, fut, gen):
        self._nav_done(gen, fut.result().status == GoalStatus.STATUS_SUCCEEDED)

    def _nav_done(self, gen, ok):
        if gen != self.core.gen:
            return                       # a replaced goal
        self.nav_handle = None
        xy = self._xy() or (0.0, 0.0)
        self._run(self.core.on_nav_result(self._now(), gen, ok, xy))


def main():
    rclpy.init()
    try:
        node = RouteExecutor()
    except SiteConfigError as e:
        print('config error: %s' % e)
        rclpy.shutdown()
        return 2
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
