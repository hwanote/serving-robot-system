"""Soft stop and resume rules for the velocity command chain (no ROS dependency).

Chain:  ... -> collision_monitor -> cmd_vel_safe -> [soft stop limiter] -> /robot_01/cmd_vel

The limiter does three things:
  1. Rate limit: the speed may not rise faster than `lin_accel` or fall faster
     than `lin_decel` (soft stop for drinks). Same for turning.
  2. Hard stop: when the collision monitor reports the last-defence HardStop,
     the output goes to zero at once (no ramp).
  3. Resume rule: after a stop caused by the stop zone (or the hard stop) the
     robot does not drive forward again until the way ahead has been clear
     (nothing within `clear_distance` of the body front) for `hold_s` seconds.
     Turning in place stays possible while it waits.
  4. Collision latch: when the contact sensor reports that the body touched
     something, the output goes to zero at once and stays zero (whatever is
     commanded) until the restart service clears it. Clearing is refused until
     `release_s` seconds have passed since the last contact. After clearing the
     resume rule (3.) applies, so the robot does not drive forward into the
     thing it touched.
"""
HARD_STOP = 'HardStop'
STOP_ZONE = 'StopZone'
ACTION_STOP = 1


def rate_limit(target, current, dt, up, down):
    """Move `current` towards `target`; |v| grows by at most `up`/s and
    shrinks (or crosses zero) by at most `down`/s."""
    delta = target - current
    growing = abs(target) > abs(current) and target * current >= 0.0
    allowed = (up if growing else down) * dt
    if delta > allowed:
        delta = allowed
    elif delta < -allowed:
        delta = -allowed
    return current + delta


class LimiterCore:
    def __init__(self, lin_accel, lin_decel, ang_accel, ang_decel,
                 cmd_timeout, clear_distance, hold_s, release_s=3.0):
        self.lin_accel = lin_accel
        self.lin_decel = lin_decel
        self.ang_accel = ang_accel
        self.ang_decel = ang_decel
        self.cmd_timeout = cmd_timeout
        self.clear_distance = clear_distance
        self.hold_s = hold_s
        self.release_s = release_s

        self.latched = False        # collision latch (needs the restart service)
        self.collisions = 0         # number of latched collisions so far
        self.last_contact = None
        self.last_contact_name = ''
        self.lin = 0.0              # current output
        self.ang = 0.0
        self.holding = False        # waiting for the way ahead to be clear
        self.clear_since = None
        self._cmd = (0.0, 0.0)
        self._cmd_time = None
        self._clearance = None      # None = unknown (no scan yet / too old)
        self._clearance_time = None

    # ----- inputs ---------------------------------------------------------
    def set_cmd(self, now, lin, ang):
        self._cmd = (lin, ang)
        self._cmd_time = now

    def set_clearance(self, now, clearance):
        self._clearance = clearance
        self._clearance_time = now

    def on_state(self, polygon, action):
        """Collision monitor state (an event): stop zone / hard stop."""
        if action != ACTION_STOP:
            return
        if polygon == HARD_STOP:
            self.lin = 0.0          # last defence: no ramp
            self.ang = 0.0
        elif polygon != STOP_ZONE:
            return
        self.holding = True
        self.clear_since = None

    def on_contact(self, now, name=''):
        """The contact sensor reports a touch. Returns True when this touch
        newly latched the collision (the caller logs it once)."""
        self.last_contact = now
        self.last_contact_name = name
        self.lin = 0.0              # no ramp: the body already touches
        self.ang = 0.0
        if self.latched:
            return False
        self.latched = True
        self.collisions += 1
        return True

    def try_clear(self, now):
        """Restart service. Returns (ok, message)."""
        if not self.latched:
            return True, 'no collision is latched'
        if self.last_contact is not None:
            quiet = now - self.last_contact
            if quiet < self.release_s:
                return False, ('still touching or touched %.1f s ago: wait %.1f'
                               ' s more' % (quiet, self.release_s - quiet))
        self.latched = False
        self.lin = 0.0
        self.ang = 0.0
        self.holding = True         # resume rule: way ahead must be clear first
        self.clear_since = None
        return True, 'collision cleared: waiting for the way ahead to be clear'

    # ----- output ---------------------------------------------------------
    def _way_is_clear(self, now):
        if self._clearance is None or self._clearance_time is None:
            return False
        if now - self._clearance_time > self.cmd_timeout:
            return False            # scan too old: stay on the safe side
        return self._clearance >= self.clear_distance

    def step(self, now, dt):
        """Advance by dt seconds, return the (linear, angular) to publish."""
        if self.latched:
            self.lin = 0.0
            self.ang = 0.0
            return 0.0, 0.0

        if self._cmd_time is None or now - self._cmd_time > self.cmd_timeout:
            cmd_lin, cmd_ang = 0.0, 0.0
        else:
            cmd_lin, cmd_ang = self._cmd

        if self.holding:
            if self._way_is_clear(now):
                if self.clear_since is None:
                    self.clear_since = now
                elif now - self.clear_since >= self.hold_s:
                    self.holding = False
                    self.clear_since = None
            else:
                self.clear_since = None

        target_lin = min(cmd_lin, 0.0) if self.holding else cmd_lin
        self.lin = rate_limit(target_lin, self.lin, dt,
                              self.lin_accel, self.lin_decel)
        self.ang = rate_limit(cmd_ang, self.ang, dt,
                              self.ang_accel, self.ang_decel)
        return self.lin, self.ang
