"""Speed limit from the person safety state (no ROS).

    NORMAL                          pass the command through
    SLOW                            clamp |linear| and |angular|
    STOP                            no forward driving; backing up and turning
                                    stay allowed at the SLOW caps (like the
                                    obstacle stop of the soft stop limiter), so
                                    the robot can still back off and detour
    SENSOR_STALE / no message /     the same clamp as SLOW (conservative, the
    unknown state text              robot can still go, slowly) plus a warning
    detector silent while the last    STOP stays STOP only while the LiDAR still sees
    state was STOP                    something ahead (or does not know); when the
                                      way ahead is clear it limits like SLOW, so
                                      the robot can still detour. The operator is
                                      told by the route executor (DetectorWatch).

This is a pure clamp on the command: it does not ramp. The soft stop limiter
behind the collision monitor keeps the accelerations gentle. The person state
comes from person_state (camera and LiDAR, worst case wins); this layer
only turns that state into a speed limit and never judges by itself.
"""
import json

NORMAL = 'NORMAL'
SLOW = 'SLOW'
STOP = 'STOP'
STALE = 'SENSOR_STALE'
_KNOWN = (NORMAL, SLOW, STOP, STALE)


def parse_state(text):
    """State text from the detector JSON. None when it cannot be read."""
    try:
        state = json.loads(text).get('state')
    except (ValueError, AttributeError):
        return None
    return state if isinstance(state, str) else None


def parse_message(text):
    """(state, source) from the detector JSON, or None when it cannot be read."""
    try:
        body = json.loads(text)
        state = body.get('state')
        source = body.get('source') or ''
    except (ValueError, AttributeError):
        return None
    if not isinstance(state, str):
        return None
    return state, source if isinstance(source, str) else ''


def _clamp(value, limit):
    return max(-limit, min(limit, value))


class PersonLimitCore:
    def __init__(self, slow_linear_mps, slow_angular_rps, message_timeout_s=1.0,
                 clear_m=1.0, clearance_timeout_s=0.5):
        self.slow_linear = slow_linear_mps
        self.slow_angular = slow_angular_rps
        self.timeout = message_timeout_s
        self.clear_m = clear_m
        self.clearance_timeout = clearance_timeout_s
        self.clearance = None        # LiDAR clearance ahead (m), None = unknown
        self.clearance_t = None
        self.state = None            # last state text received
        self.source = ''             # sensor of the last state (camera / lidar)
        self.last_msg = None         # time of the last readable message

    def on_message(self, now, state, source=''):
        """A readable detector message arrived (`state` may be any text)."""
        self.state = state
        self.source = source
        self.last_msg = now

    def effective_state(self, now):
        """NORMAL / SLOW / STOP / SENSOR_STALE. No message for message_timeout_s
        (detector dead, never started) and unknown texts count as SENSOR_STALE,
        except that a silent detector keeps a STOP while the LiDAR does not say the way
        ahead is clear."""
        if self.last_msg is None:
            return STALE
        if now - self.last_msg > self.timeout:
            if self.state == STOP and not self.front_clear(now):
                return STOP
            return STALE
        return self.state if self.state in _KNOWN else STALE

    def set_clearance(self, now, clearance):
        """LiDAR clearance ahead from the body front (m, inf = free)."""
        self.clearance = clearance
        self.clearance_t = now

    def front_clear(self, now):
        """True only when a fresh LiDAR scan says the way ahead is clear.
        Unknown or old counts as not clear (safe side)."""
        if self.clearance is None or self.clearance_t is None:
            return False
        if now - self.clearance_t > self.clearance_timeout:
            return False
        return self.clearance >= self.clear_m

    def is_silent(self, now):
        """The detector has said something before but not for message_timeout_s."""
        return self.last_msg is not None and now - self.last_msg > self.timeout

    def status(self, now):
        """What is applied right now, for the others (executor, UI):
        {state, silent, source}. `state` is the effective state; the source of a
        STOP held by a silent detector is the LiDAR."""
        state = self.effective_state(now)
        silent = self.is_silent(now)
        source = ''
        if state == STOP:
            source = 'lidar' if silent else self.source
        return {'state': state, 'silent': silent, 'source': source}

    def limit(self, now, lin, ang):
        """Returns (linear, angular, effective_state)."""
        state = self.effective_state(now)
        if state == NORMAL:
            return lin, ang, state
        if state == STOP:
            return (_clamp(min(lin, 0.0), self.slow_linear),
                    _clamp(ang, self.slow_angular), state)
        return (_clamp(lin, self.slow_linear), _clamp(ang, self.slow_angular),
                state)
