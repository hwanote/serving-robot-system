"""Person safety state from the distances of detected people (no ROS, no YOLO).

    NORMAL        no person, or far away
    SLOW          nearest person closer than slow_m
    STOP          nearest person closer than stop_m
    SENSOR_STALE  no camera frame for stale_s (or none yet): be conservative

Getting stricter is immediate. Getting less strict needs a hysteresis margin
(the distance must grow beyond the threshold + hysteresis_m) and that the
condition holds for release_frames frames in a row. The distance used is the
median of the last median_n frames of the nearest person, so a single wrong
depth value does not flip the state.

If a person who was close (SLOW / STOP, last seen nearer than slow_m + hysteresis)
disappears, the state is NOT released at once: the camera often loses a person
who is very close (head and shoulders leave the picture). The state is kept for
at least lost_hold_s, and after that until the LiDAR says the way ahead is clear:
front_clear_m (distance from the body front to the nearest point ahead, from the
scan) must be at least clear_m for clear_hold_s in a row. Unknown clearance
(None: no scan, or too old) counts as blocked. external_hold
(/robot_01/obstacle_hold) also keeps the state. A person who was already far
away when lost is released by the normal frame counting.

The LiDAR clearance ahead (front_clear_m) is judged together with the camera:
both go into the same list of distances and the nearest one decides, so either
sensor alone can raise SLOW / STOP (worst case wins) and there is only one
state machine. While the LiDAR still sees something close, the person lost by
the camera is simply the LiDAR distance.

This is the semantic layer only: it must never be the only safety layer (the
LiDAR / Collision Monitor stay in charge of the final protection).
"""
from statistics import median

NORMAL = 'NORMAL'
SLOW = 'SLOW'
STOP = 'STOP'
STALE = 'SENSOR_STALE'
_RANK = {NORMAL: 0, SLOW: 1, STOP: 2}


class PersonSafetyCore:
    def __init__(self, stop_m=0.8, slow_m=1.5, hysteresis_m=0.2,
                 release_frames=3, median_n=3, stale_s=2.0, lost_hold_s=3.0,
                 clear_m=1.0, clear_hold_s=2.0):
        self.stop_m = stop_m
        self.slow_m = slow_m
        self.hysteresis_m = hysteresis_m
        self.release_frames = release_frames
        self.median_n = median_n
        self.stale_s = stale_s
        self.lost_hold_s = lost_hold_s
        self.clear_m = clear_m
        self.clear_hold_s = clear_hold_s
        self.clear_since = None
        self.lost_since = None
        self.last_seen = None    # last stable distance while the person was seen
        self.state = STALE
        self.reason = 'no camera frame yet'
        self.last_frame = None
        self.hist = []
        self.release = 0
        self.raw = None          # nearest distance of the last frame
        self.source = ''         # which sensor gave the nearest distance
        self.stable = None       # median over the last frames

    @staticmethod
    def _level(d, stop_m, slow_m):
        if d is None:
            return NORMAL
        if d < stop_m:
            return STOP
        if d < slow_m:
            return SLOW
        return NORMAL

    def update(self, now, distances, external_hold=False, front_clear_m=None):
        """A camera frame was processed. `distances`: metres of every detected
        person that has a valid depth (an empty list: nobody seen).
        `external_hold`: the LiDAR side still holds the robot for an obstacle.
        `front_clear_m`: clearance ahead from the scan (None = unknown, inf = free);
        a finite value counts like one more detected person at that distance."""
        self.last_frame = now
        distances = list(distances)
        self.source = 'camera' if distances else ''
        if front_clear_m is not None and front_clear_m != float('inf'):
            # One judge: the LiDAR clearance ahead is just another candidate
            # distance. The nearest of camera and LiDAR decides (worst case).
            if not distances or front_clear_m < min(distances):
                self.source = 'lidar'
            distances.append(front_clear_m)
        if distances:
            self.raw = min(distances)
            self.hist = (self.hist + [self.raw])[-self.median_n:]
            self.stable = median(self.hist)
            self.lost_since = None
            self.clear_since = None
            self.last_seen = self.stable
        else:
            if self.lost_since is None:
                self.lost_since = now
            self.raw = None
            self.hist = []
            self.stable = None
            close = (self.state in (SLOW, STOP) and self.last_seen is not None
                     and self.last_seen < self.slow_m + self.hysteresis_m)
            if close:
                waited = now - self.lost_since
                clear = front_clear_m is not None and front_clear_m >= self.clear_m
                if not clear:
                    self.clear_since = None
                elif self.clear_since is None:
                    self.clear_since = now
                clear_for = 0.0 if self.clear_since is None else now - self.clear_since
                if waited < self.lost_hold_s:
                    why = '%.1f of %.1f s' % (waited, self.lost_hold_s)
                elif external_hold:
                    why = 'obstacle hold'
                elif not clear:
                    why = ('something %.2f m ahead' % front_clear_m
                           if front_clear_m is not None and front_clear_m != float('inf')
                           else 'front clearance unknown')
                elif clear_for < self.clear_hold_s:
                    why = 'front clear for %.1f of %.1f s' % (clear_for, self.clear_hold_s)
                else:
                    why = ''
                if why:
                    self.release = 0
                    self.reason = 'lost while close, holding (%s)' % why
                    return self.state
                self.state, self.release, self.reason = NORMAL, 0, ''
                self.last_seen = None
                self.clear_since = None
                return self.state
        current = NORMAL if self.state == STALE else self.state
        target = self._level(self.stable, self.stop_m, self.slow_m)
        relaxed = self._level(self.stable, self.stop_m + self.hysteresis_m,
                              self.slow_m + self.hysteresis_m)
        if _RANK[target] >= _RANK[current]:
            new, self.release = target, 0
        elif _RANK[relaxed] >= _RANK[current]:
            new, self.release = current, 0          # inside the hysteresis band
        else:
            self.release += 1
            new = relaxed if self.release >= self.release_frames else current
            if new != current:
                self.release = 0
        self.state = new
        self.reason = ''
        if new != NORMAL and self.source == 'lidar':
            self.reason = 'LiDAR: something %.2f m ahead' % self.stable
        return self.state

    def check_stale(self, now):
        """Call regularly; turns the state to SENSOR_STALE when frames stop."""
        if self.last_frame is None:
            self.state, self.reason = STALE, 'no camera frame yet'
        elif now - self.last_frame > self.stale_s:
            self.state = STALE
            self.reason = 'no camera frame for %.1f s' % (now - self.last_frame)
            self.hist, self.release = [], 0
            self.raw = self.stable = None
            self.lost_since = self.last_seen = self.clear_since = None
        return self.state
