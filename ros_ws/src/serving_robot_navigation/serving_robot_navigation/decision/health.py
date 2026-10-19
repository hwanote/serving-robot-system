"""Robot health: one summary of the robot for the fleet and the panel (no ROS).

The node scripts/decision/robot_health.py feeds the facts it hears and publishes
report() on /{robot}/health (JSON, once a second and at once on a change).
Enums follow the team integration guide (Q3-3, MUST); the three states are kept
apart (robot / safety / localization), the details are in `checks` / `reasons`.

    robot_state (first match)
        ERROR       collision latched, localization LOST, no scan for scan_stale_s,
                    route executor silent, or still not booted after boot_timeout_s
        BOOTING     until the scan, a pose and the route executor have been heard
        BLOCKED     driving and stopped for blocked_after_s (the BLOCKED alarm)
        DEGRADED    person detector silent, a camera silent, localization DEGRADED,
                    person speed limiter silent, odometry stale
        PAUSED      waiting for the load / the handover, held after a failed delivery,
                    mission on hold after a localization loss (reason LOCALIZATION_HOLD)
        NAVIGATING  driving
        IDLE        no mission
        (OFFLINE is decided by the fleet when this message stops)
    safety_state      NORMAL / SLOW / STOP: the applied person limit (SENSOR_STALE ->
                      SLOW), STOP also while the obstacle hold or the collision latch is
                      on, and when the person speed limiter (part of the chain) is silent
    localization_state UNINITIALIZED / OK / DEGRADED / LOST from the AMCL covariance
                      (standard deviation) and the age of the map -> base pose (TF)

It only reports: it changes nothing in the robot's behaviour.
"""
import math
from dataclasses import dataclass

SCHEMA_VERSION = 1


@dataclass
class HealthParams:
    robot_id: str = 'robot_01'
    map_revision: int = 0
    person_expected: bool = True     # person_limit:=true (detector + limiter run)
    boot_timeout_s: float = 60.0
    scan_stale_s: float = 2.0
    odom_stale_s: float = 2.0
    msg_stale_s: float = 2.0         # mission_status / person_limit
    pose_lost_s: float = 2.0         # no map -> base pose for this long: LOST
    ok_std_m: float = 0.25
    ok_std_rad: float = 0.20
    lost_std_m: float = 1.0
    blocked_after_s: float = 15.0


class HealthCore:
    def __init__(self, params=None):
        self.p = params or HealthParams()
        self.t0 = None
        self.booted = False
        self.scan_t = self.odom_t = None
        self.mission, self.mission_t = None, None
        self.limit, self.limit_t = None, None
        self.silent_cameras = []
        self.hold = self.latched = False
        self.std_xy = self.std_yaw = None
        self.pose, self.pose_t = None, None

    # ----- inputs ---------------------------------------------------------------
    def start(self, now):
        if self.t0 is None:
            self.t0 = now

    def on_scan(self, now):
        self.scan_t = now

    def on_odom(self, now):
        self.odom_t = now

    def on_mission(self, now, body):
        self.mission, self.mission_t = body, now

    def on_person_limit(self, now, body):
        self.limit, self.limit_t = body, now

    def on_person_detection(self, now, body):
        cams = body.get('silent_cameras', []) if isinstance(body, dict) else []
        self.silent_cameras = sorted(str(c) for c in cams) if isinstance(cams, list) else []

    def on_hold(self, value):
        self.hold = bool(value)

    def on_latched(self, value):
        self.latched = bool(value)

    def on_amcl(self, cov_xx, cov_yy, cov_yawyaw):
        self.std_xy = math.sqrt(max(cov_xx, cov_yy, 0.0))
        self.std_yaw = math.sqrt(max(cov_yawyaw, 0.0))

    def on_pose(self, now, x, y, yaw):
        self.pose, self.pose_t = (x, y, yaw), now

    # ----- judgement ----------------------------------------------------------------
    def _fresh(self, t, now, limit):
        return t is not None and now - t <= limit

    def localization_state(self, now):
        if self.pose_t is None or self.std_xy is None:
            return 'UNINITIALIZED'
        if now - self.pose_t > self.p.pose_lost_s or self.std_xy >= self.p.lost_std_m:
            return 'LOST'
        if self.std_xy > self.p.ok_std_m or self.std_yaw > self.p.ok_std_rad:
            return 'DEGRADED'
        return 'OK'

    def safety_state(self, now):
        if self.hold or self.latched:
            return 'STOP'
        if not self.p.person_expected:
            return 'NORMAL'
        if not self._fresh(self.limit_t, now, self.p.msg_stale_s):
            # the person speed limiter is part of the velocity chain: when it is
            # silent no command reaches the wheels, the robot stands still
            return 'STOP'
        state = str(self.limit.get('state', ''))
        return {'NORMAL': 'NORMAL', 'STOP': 'STOP'}.get(state, 'SLOW')

    def report(self, now):
        self.start(now)
        p = self.p
        loc = self.localization_state(now)
        scan_ok = self._fresh(self.scan_t, now, p.scan_stale_s)
        odom_ok = self._fresh(self.odom_t, now, p.odom_stale_s)
        mission_ok = self._fresh(self.mission_t, now, p.msg_stale_s)
        limit_ok = self._fresh(self.limit_t, now, p.msg_stale_s)
        if not self.booted and scan_ok and mission_ok and loc != 'UNINITIALIZED':
            self.booted = True
        booting = not self.booted and now - self.t0 < p.boot_timeout_s

        errors, degraded = [], []
        if self.latched:
            errors.append('COLLISION_LATCHED')
        if loc == 'LOST':
            errors.append('LOCALIZATION_LOST')
        if not scan_ok and not booting:
            errors.append('SCAN_STALE')
        if not mission_ok and not booting:
            errors.append('MISSION_STATUS_SILENT')
        if loc == 'UNINITIALIZED' and not booting:
            errors.append('LOCALIZATION_UNINITIALIZED')
        if loc == 'DEGRADED':
            degraded.append('LOCALIZATION_DEGRADED')
        if not odom_ok and not booting:
            degraded.append('ODOM_STALE')
        detector = 'off'
        if p.person_expected:
            if not limit_ok:
                detector = 'unknown'
                if not booting:
                    degraded.append('PERSON_LIMIT_SILENT')
            elif self.limit.get('silent'):
                detector = 'silent'
                degraded.append('DETECTOR_SILENT')
            else:
                detector = 'ok'
            if self.silent_cameras and detector == 'ok':
                degraded.append('CAMERA_SILENT')

        m = self.mission if mission_ok else None
        mstate = str(m.get('state', '')) if m else ''
        stopped = float(m.get('stopped_s', 0.0) or 0.0) if m else 0.0
        # mission on hold after a localization loss: stopped on purpose, waiting
        # for the operator (not an obstacle wait)
        held = bool(m.get('loc_hold')) if m else False
        blocked = mstate == 'DRIVING' and not held and stopped >= p.blocked_after_s
        if errors:
            robot = 'ERROR'
        elif booting:
            robot = 'BOOTING'
        elif blocked:
            robot = 'BLOCKED'
        elif degraded:
            robot = 'DEGRADED'
        elif held or mstate in ('WAIT_LOAD', 'WAIT_HANDOVER', 'PAUSED'):
            robot = 'PAUSED'
        elif mstate == 'DRIVING':
            robot = 'NAVIGATING'
        else:
            robot = 'IDLE'
        reasons = errors + (['BLOCKED'] if blocked else []) + degraded
        if held:
            reasons.append('LOCALIZATION_HOLD')
        if booting and not errors:
            reasons = ['BOOTING'] + reasons

        pose = None
        if self.pose is not None:
            pose = {'frame_id': 'map', 'x_m': round(self.pose[0], 3),
                    'y_m': round(self.pose[1], 3), 'yaw_rad': round(self.pose[2], 3),
                    'map_revision': int(p.map_revision)}
        return {
            'schema_version': SCHEMA_VERSION,
            'robot_id': p.robot_id,
            'time': round(now, 1),
            'robot_state': robot,
            'safety_state': self.safety_state(now),
            'localization_state': loc,
            'pose': pose,
            'localization': {
                'std_xy_m': None if self.std_xy is None else round(self.std_xy, 3),
                'std_yaw_rad': None if self.std_yaw is None else round(self.std_yaw, 3)},
            'mission': {
                'state': mstate or None,
                'mission_id': (m.get('mission_id') or None) if m else None,
                'destination': (m.get('destination') or None) if m else None,
                'stopped_s': round(stopped, 1),
                'loc_hold': (m.get('loc_hold') or '') if m else ''},
            'checks': {
                'detector': detector,
                'silent_cameras': list(self.silent_cameras),
                'scan': 'ok' if scan_ok else 'stale',
                'odom': 'ok' if odom_ok else 'stale',
                'obstacle_hold': self.hold,
                'collision_latched': self.latched},
            'reasons': reasons,
        }

    @staticmethod
    def summary(report):
        """What must be published at once when it changes."""
        return (report['robot_state'], report['safety_state'],
                report['localization_state'], tuple(report['reasons']))
