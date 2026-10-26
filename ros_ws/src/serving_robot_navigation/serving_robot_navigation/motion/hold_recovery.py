"""Way out of an obstacle hold by turning in place (no ROS dependency).

Why: after the stop zone fired, the soft stop limiter keeps the robot from driving
forward until the way ahead has been clear (`scan_dynamic`, resume distance) for a
few seconds. A robot that stands close to a wall never gets a clear way ahead by
itself, so the hold never ends. This core waits `hold_s` seconds in the hold, then
turns the robot in place towards the nearest heading with enough room, stops as
soon as the way ahead is clear, and waits for the limiter to release.

Rules (the robot carries drinks):
  * only turns in place: never drives, never reverses;
  * only acts while NO navigation goal is active (the controller owns the motion
    then, mapping cancels the goal on a hold);
  * looks at the same scan the limiter uses, so "clear" here means "the limiter
    will release";
  * gives up after `max_rotate_s` (a full turn finds nothing) and tries again
    after `giveup_s`.
The command goes into the START of the velocity chain (cmd_vel_nav), so the
collision monitor and the soft stop limiter still apply.
"""
import math

import numpy as np

from serving_robot_navigation.monitoring.clearance import forward_clearance

IDLE = 'IDLE'
ROTATING = 'ROTATING'
SETTLING = 'SETTLING'


def heading_options(ranges, angle_min, angle_increment, range_min, range_max,
                    front, half_width, step_deg=15.0):
    """[(heading rad, clearance m)] for headings around the robot.

    The clearance of a heading is the forward clearance the robot would have after
    turning by that angle (counter-clockwise positive), seen in the same corridor
    as the soft stop limiter.
    """
    out = []
    n = int(round(360.0 / step_deg))
    for i in range(n):
        theta = -math.pi + 2.0 * math.pi * (i + 0.5) / n
        c = forward_clearance(ranges, angle_min - theta, angle_increment,
                              range_min, range_max, front, half_width)
        out.append((theta, c))
    return out


class HoldRecoveryCore:
    def __init__(self, hold_s=5.0, target_clear_m=1.3, ang_speed_rps=0.4,
                 max_rotate_s=30.0, settle_s=6.0, giveup_s=10.0,
                 data_timeout_s=0.5):
        self.hold_s = hold_s
        self.target_clear_m = target_clear_m
        self.ang_speed = abs(ang_speed_rps)
        self.max_rotate_s = max_rotate_s
        self.settle_s = settle_s
        self.giveup_s = giveup_s
        self.data_timeout_s = data_timeout_s

        self.state = IDLE
        self.hold = False
        self.goal_active = False
        self.direction = 0              # +1 counter-clockwise, -1 clockwise
        self.turns = 0                  # recoveries started so far
        self.events = []                # text lines for the node to log
        self._hold_since = None
        self._rotate_since = None
        self._settle_until = None
        self._next_ok = 0.0
        self._front = None
        self._options = ()
        self._scan_time = None

    # ----- inputs ---------------------------------------------------------
    def set_hold(self, hold):
        self.hold = bool(hold)

    def set_goal_active(self, active):
        self.goal_active = bool(active)

    def set_scan(self, now, front_clear, options):
        self._front = front_clear
        self._options = tuple(options)
        self._scan_time = now

    # ----- helpers --------------------------------------------------------
    def _fresh(self, now):
        return (self._scan_time is not None
                and now - self._scan_time <= self.data_timeout_s)

    def _pick_direction(self):
        """(direction, heading rad) towards the nearest heading with room, or
        towards the roomiest one when none has enough."""
        enough = [(t, c) for t, c in self._options if c >= self.target_clear_m]
        if enough:
            theta = min(enough, key=lambda tc: (abs(tc[0]), -tc[0]))[0]
        else:
            theta = max(self._options, key=lambda tc: (tc[1], -abs(tc[0])))[0]
        return (1 if theta > 0.0 else -1), theta

    def _stop(self, new_state):
        self.state = new_state
        self.direction = 0
        self._rotate_since = None

    # ----- output ---------------------------------------------------------
    def step(self, now):
        """Return the angular velocity (rad/s) to send; 0.0 = send nothing."""
        if self.state == ROTATING:
            if not self.hold:
                self.events.append('hold released while turning: done')
                self._stop(IDLE)
                self._hold_since = None
                return 0.0
            if self.goal_active:
                self.events.append('a navigation goal became active: stop turning')
                self._stop(IDLE)
                self._hold_since = None
                return 0.0
            if not self._fresh(now):
                self.events.append('scan too old: stop turning')
                self._stop(IDLE)
                self._next_ok = now + self.giveup_s
                return 0.0
            if self._front >= self.target_clear_m:
                self.events.append(
                    'way ahead is clear (%.2f m): waiting for the limiter to release'
                    % self._front)
                self._stop(SETTLING)
                self._settle_until = now + self.settle_s
                return 0.0
            if now - self._rotate_since > self.max_rotate_s:
                self.events.append(
                    'no clear heading after %.0f s of turning: giving up for %.0f s'
                    % (self.max_rotate_s, self.giveup_s))
                self._stop(IDLE)
                self._next_ok = now + self.giveup_s
                self._hold_since = None
                return 0.0
            return self.direction * self.ang_speed

        if self.state == SETTLING:
            if not self.hold:
                self._stop(IDLE)
                self._hold_since = None
            elif now >= self._settle_until:
                self.events.append('still holding after the turn: trying again')
                self._stop(IDLE)
                self._hold_since = now
            return 0.0

        # IDLE: count the time in a hold with no goal active
        if not self.hold or self.goal_active:
            self._hold_since = None
            return 0.0
        if self._hold_since is None:
            self._hold_since = now
            return 0.0
        if now - self._hold_since < self.hold_s or now < self._next_ok:
            return 0.0
        if not self._fresh(now) or not self._options:
            return 0.0
        if self._front >= self.target_clear_m:
            self._hold_since = now          # the limiter will release by itself
            return 0.0
        direction, theta = self._pick_direction()
        self.direction = direction
        self.state = ROTATING
        self._rotate_since = now
        self.turns += 1
        self.events.append(
            'hold for %.1f s with %.2f m ahead: turning %s towards %.0f deg '
            '(recovery #%d)' % (now - self._hold_since, self._front,
                                'left' if direction > 0 else 'right',
                                math.degrees(theta), self.turns))
        return self.direction * self.ang_speed

    def pop_events(self):
        out, self.events = self.events, []
        return out
