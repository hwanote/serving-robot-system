"""What is watched while one tour step is driven. No ROS: events go in, an Observation comes out.

The drive client feeds it: status / alarm messages of the route executor, the contact latch,
the collision monitor's HardStop zone, LiDAR sector minima and the robot pose.
"""
import math

from .verdict import BLOCK_ALARMS, HARDSTOP_FAIL_S

MAX_GAP_S = 0.5          # a gap between two state messages longer than this is not counted as active


def sector_mins(ranges, angle_min, angle_inc, range_min, range_max):
    """Closest return ahead (|a| <= 30 deg), left (60..120) and right (-120..-60), in metres.
    0 deg = straight ahead, + = left (same convention as scripts/dev/scan_min.py). None = no return."""
    best = {'front': None, 'left': None, 'right': None}
    for i, r in enumerate(ranges):
        if math.isnan(r) or math.isinf(r) or not (range_min <= r <= range_max):
            continue
        a = math.degrees(angle_min + i * angle_inc)
        a = (a + 180.0) % 360.0 - 180.0
        if abs(a) <= 30.0:
            k = 'front'
        elif 60.0 <= a <= 120.0:
            k = 'left'
        elif -120.0 <= a <= -60.0:
            k = 'right'
        else:
            continue
        if best[k] is None or r < best[k]:
            best[k] = r
    return best['front'], best['left'], best['right']


def dist_to_segment(p, a, b):
    ax, ay, bx, by = a[0], a[1], b[0], b[1]
    dx, dy = bx - ax, by - ay
    den = dx * dx + dy * dy
    t = 0.0 if den == 0 else max(0.0, min(1.0, ((p[0] - ax) * dx + (p[1] - ay) * dy) / den))
    return math.hypot(p[0] - (ax + t * dx), p[1] - (ay + t * dy))


def _min(a, b):
    return b if a is None else (a if b is None else min(a, b))


class StepMonitor:
    def __init__(self, mission_id, seg_a, seg_b, t0):
        self.mid, self.a, self.b, self.t0 = mission_id, seg_a, seg_b, t0
        self.result = ''
        self.code = None
        self.alarms = []
        self.rejected = None
        self.latched = False
        self.hardstop_s = 0.0
        self._hs_active, self._hs_t = False, None
        self.stopped_max_s = 0.0
        self.person_slow_s = self.person_stop_s = 0.0
        self._pl, self._pl_t = 'NORMAL', None
        self.max_off_rail_m = 0.0
        self.final_error_m = None
        self.min_front_m = self.min_left_m = self.min_right_m = None

    # ---- events
    def on_status(self, s, t):
        if s.get('mission_id') != self.mid:
            return
        self.stopped_max_s = max(self.stopped_max_s, float(s.get('stopped_s') or 0.0))
        if s.get('result'):
            self.result = str(s['result'])
            self.code = s.get('code')

    def on_alarm(self, a, t):
        if a.get('rejected_id') == self.mid:
            self.rejected = a
        code = a.get('code')
        if code and code not in self.alarms:
            self.alarms.append(str(code))

    def on_latched(self, value, t):
        if value:
            self.latched = True

    def on_hardstop(self, active, t):
        if self._hs_active and self._hs_t is not None:
            self.hardstop_s += min(t - self._hs_t, MAX_GAP_S)
        self._hs_active, self._hs_t = bool(active), t

    def _person_tick(self, t):
        if self._pl_t is not None:
            dt = min(t - self._pl_t, 1.0)       # the state is held between messages
            if self._pl == 'SLOW':
                self.person_slow_s += dt
            elif self._pl == 'STOP':
                self.person_stop_s += dt
        self._pl_t = t

    def tick(self, t):
        """Called every loop pass so a held person-limit state keeps counting between messages."""
        self._person_tick(t)

    def on_person(self, state, t):
        """State of /robot_01/person_limit (NORMAL / SLOW / STOP / SENSOR_STALE)."""
        self._person_tick(t)
        self._pl = str(state)

    def on_scan(self, front, left, right, t):
        self.min_front_m = _min(self.min_front_m, front)
        self.min_left_m = _min(self.min_left_m, left)
        self.min_right_m = _min(self.min_right_m, right)

    def on_pose(self, x, y, t):
        self.max_off_rail_m = max(self.max_off_rail_m, dist_to_segment((x, y), self.a, self.b))
        self.final_error_m = math.hypot(x - self.b[0], y - self.b[1])

    # ---- decisions
    def finished(self):
        return bool(self.result) or self.rejected is not None

    def cancel_reason(self):
        """Why the drive client should cancel now (first problem wins), or None."""
        if self.latched:
            return 'collision latched'
        if self.hardstop_s >= HARDSTOP_FAIL_S:
            return 'HardStop active for %.1f s' % self.hardstop_s
        bad = [c for c in self.alarms if c in BLOCK_ALARMS or c == 'OFF_RAIL']
        if bad:
            return 'alarm %s' % bad[0]
        return None

    def observation(self, now, timed_out=False):
        self._person_tick(now)
        hs = self.hardstop_s
        if self._hs_active and self._hs_t is not None:
            hs += min(now - self._hs_t, MAX_GAP_S)
        o = {'result': self.result, 'code': self.code, 'alarms': list(self.alarms),
             'latched': self.latched, 'hardstop_s': round(hs, 2), 'timed_out': timed_out,
             'duration_s': round(now - self.t0, 1), 'stopped_max_s': round(self.stopped_max_s, 1),
             'max_off_rail_m': round(self.max_off_rail_m, 3),
             'person_slow_s': round(self.person_slow_s, 1), 'person_stop_s': round(self.person_stop_s, 1),
             'min_front_m': self.min_front_m, 'min_left_m': self.min_left_m,
             'min_right_m': self.min_right_m,
             'final_error_m': None if self.final_error_m is None else round(self.final_error_m, 3)}
        if self.rejected is not None:
            o['result'], o['code'] = 'FAILED', self.rejected.get('code')
        return o
