"""Camera observations + LiDAR facts -> the person safety message (no ROS).

The judge node (scripts/decision/person_state.py) feeds every camera
observation (monitoring/person_observation.py) to PersonJudge and publishes the
result on /{robot}/person_detection.

Primary camera (the front one): its observations drive the state, exactly as
the old one-node person_detector did, so the message is the same as before:

    new camera frame  -> core.update(people ahead with a distance, obstacle_hold,
                                     front clearance)       (PersonSafetyCore)
    no new frame      -> core.check_stale (SENSOR_STALE after stale_s)

and the judge answers only when a primary observation comes in: if the camera
node dies, person_detection goes quiet too (detector silent: DETECTOR_SILENT,
STOP kept only while the LiDAR sees something ahead).

Direction rules (step 4, several cameras; bearing = direction seen from the robot):
    ahead  |bearing| <= front_half      the rules above (SLOW 1.5 m, STOP 0.8 m)
    beside front_half < |b| <= side_max a person closer than side_slow_m raises
                                        NORMAL to SLOW (never STOP); released
                                        when nobody is closer than side_slow_m +
                                        hysteresis for side_release_s
    behind |b| > side_max               no effect on the speed (the robot never
                                        reverses); reported as rear_m
A person without a bearing (old observations) counts as ahead.

Camera health: a camera with no new frame for camera_stale_s is listed in
silent_cameras (the route executor raises CAMERA_SILENT). When the primary
camera comes back after a gap longer than camera_stale_s (the camera node was
restarted), every camera gets camera_stale_s again to send its first frame, so a
restart gives DETECTOR_RECOVERED only, not a burst of CAMERA_SILENT. Only the speed rules
of the primary camera change with it (SENSOR_STALE); the others only alarm.
"""
import math

from serving_robot_navigation.decision.person_safety import NORMAL, SLOW
from serving_robot_navigation.monitoring.person_observation import measured


class PersonJudge:
    def __init__(self, core, primary_camera='', cameras=(), front_half_deg=60.0,
                 side_max_deg=120.0, side_slow_m=0.8, side_release_s=1.0,
                 camera_stale_s=2.0):
        self.core = core
        self.primary = primary_camera        # '' = every camera is primary (one camera)
        self.cameras = list(cameras)         # expected cameras (for silent_cameras)
        self.front_half = math.radians(front_half_deg)
        self.side_max = math.radians(side_max_deg)
        self.side_slow_m = side_slow_m
        self.side_release_s = side_release_s
        self.camera_stale_s = camera_stale_s
        self.last = {'confidence': 0.0, 'count': 0, 'bbox': None}
        self.cams = {}                       # name -> {'people': [...], 'frame_t': t}
        self.t0 = None                       # first primary observation (or restart)
        self.last_primary_t = None
        self.side_active = False
        self.side_clear_since = None
        self.side_info = None                # (distance, camera, bearing)

    # ----- helpers -------------------------------------------------------------
    def _sector(self, p):
        b = p.get('bearing_rad')
        if not isinstance(b, (int, float)):
            return 'ahead'
        b = abs(math.atan2(math.sin(b), math.cos(b)))
        if b <= self.front_half:
            return 'ahead'
        return 'beside' if b <= self.side_max else 'behind'

    def _fresh(self, now):
        return {name: c for name, c in self.cams.items()
                if c['frame_t'] is not None and now - c['frame_t'] <= self.camera_stale_s}

    def _nearest(self, now, sector):
        best = None
        for name, c in self._fresh(now).items():
            for p in measured({'people': c['people']}):
                if self._sector(p) == sector and (best is None or p['distance_m'] < best[0]):
                    best = (p['distance_m'], name, p.get('bearing_rad'))
        return best

    def _side_rule(self, now):
        near = self._nearest(now, 'beside')
        if near is not None and near[0] < self.side_slow_m:
            self.side_active, self.side_clear_since, self.side_info = True, None, near
            return
        if not self.side_active:
            return
        if near is not None and near[0] < self.side_slow_m + self.core.hysteresis_m:
            self.side_clear_since, self.side_info = None, near
            return
        if self.side_clear_since is None:
            self.side_clear_since = now
        elif now - self.side_clear_since >= self.side_release_s:
            self.side_active, self.side_clear_since, self.side_info = False, None, None

    def silent_cameras(self, now):
        if self.t0 is None:
            return []
        names = self.cameras or sorted(self.cams)
        out = []
        for name in names:
            c = self.cams.get(name)
            last = c['frame_t'] if c and c['frame_t'] is not None else self.t0
            if now - last > self.camera_stale_s:
                out.append(name)
        return out

    # ----- main ---------------------------------------------------------------------
    def on_observation(self, now, obs, obstacle_hold=False, front_clear_m=None):
        """One observation in -> the person_detection body (dict), or None when
        it was not from the primary camera (only remembered).
        `front_clear_m`: clearance ahead from the scan, None = unknown."""
        name = obs.get('camera', '')
        rec = self.cams.setdefault(name, {'people': [], 'frame_t': None})
        if obs.get('new_frame'):
            rec['people'], rec['frame_t'] = list(obs.get('people', [])), now
        if self.primary and name != self.primary:
            return None
        if self.t0 is None:
            self.t0 = now
        elif now - self.last_primary_t > self.camera_stale_s:
            # the camera node was away (restart): frames from before the gap say
            # nothing about the cameras now; give every camera time again
            for cam_name, c in self.cams.items():
                if (c['frame_t'] is not None and c['frame_t'] <= self.last_primary_t
                        and not (cam_name == name and obs.get('new_frame'))):
                    c['frame_t'], c['people'] = None, []
            self.t0 = now
        self.last_primary_t = now
        core = self.core
        if obs.get('new_frame'):
            valid = [p for p in measured(obs) if self._sector(p) == 'ahead']
            core.update(now, [p['distance_m'] for p in valid], obstacle_hold,
                        front_clear_m)
            best = min(valid, key=lambda p: p['distance_m']) if valid else None
            self.last = {
                'confidence': round(float(best['confidence']), 2) if best else 0.0,
                'count': int(obs.get('count', len(obs.get('people', [])))),
                'bbox': [round(v) for v in best['bbox']] if best else None}
        core.check_stale(now)
        self._side_rule(now)
        state, reason, source = core.state, core.reason, core.source
        if self.side_active and state == NORMAL:
            d, cam, b = self.side_info
            state, source = SLOW, 'camera'
            reason = 'person beside %.2f m (%s%s)' % (
                d, cam, '' if b is None else ', %.0f deg' % math.degrees(b))
        rear = self._nearest(now, 'behind')
        return {'state': state, 'reason': reason,
                'obstacle_hold': bool(obstacle_hold), 'source': source,
                'front_clear_m': (None if front_clear_m in (None, float('inf'))
                                  else round(front_clear_m, 2)),
                'distance_m': None if core.stable is None else round(core.stable, 2),
                'nearest_m': None if core.raw is None else round(core.raw, 2),
                'confidence': self.last['confidence'], 'count': self.last['count'],
                'bbox': self.last['bbox'],
                'frame_age_s': (None if core.last_frame is None
                                else round(now - core.last_frame, 2)),
                'camera': name, 'time': round(now, 1),
                'side_m': None if self.side_info is None else round(self.side_info[0], 2),
                'side_camera': '' if self.side_info is None else self.side_info[1],
                'rear_m': None if rear is None else round(rear[0], 2),
                'silent_cameras': self.silent_cameras(now)}
