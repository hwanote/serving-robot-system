"""HoldRecoveryCore: turning in place out of an obstacle hold (no ROS).

    python3 tests/robot/test_hold_recovery.py
"""
import math
import os
import sys

import numpy as np

ROOT = os.path.join(os.path.dirname(__file__), '..', '..')
sys.path.insert(0, os.path.join(ROOT, 'ros_ws', 'src', 'serving_robot_navigation'))
from serving_robot_navigation.motion.hold_recovery import (  # noqa: E402
    HoldRecoveryCore, IDLE, ROTATING, SETTLING, heading_options)

FRONT, HALF_W = 0.26, 0.20


def _scan(wall_dir_deg=0.0, wall_dist=0.55, n=720, rmax=12.0):
    """Open room with a wall wall_dist m away in the direction wall_dir_deg
    (a 60 degree wide wall patch)."""
    a = -math.pi + 2.0 * math.pi * np.arange(n) / n
    r = np.full(n, rmax - 0.1)
    d = np.degrees(np.angle(np.exp(1j * (a - math.radians(wall_dir_deg)))))
    r[np.abs(d) <= 30.0] = wall_dist
    return r, -math.pi, 2.0 * math.pi / n


def _feed(core, now, wall_dir=0.0, wall_dist=0.55):
    r, amin, inc = _scan(wall_dir, wall_dist)
    args = (r, amin, inc, 0.05, 12.0, FRONT, HALF_W)
    from serving_robot_navigation.monitoring.clearance import forward_clearance
    core.set_scan(now, forward_clearance(*args), heading_options(*args))


def _run(core, t0, t1, dt=0.05, wall_dir=0.0, wall_dist=0.55):
    """Step from t0 to t1 with the same scan; return (last time, last output)."""
    t, w = t0, 0.0
    while t < t1:
        _feed(core, t, wall_dir, wall_dist)
        w = core.step(t)
        t += dt
    return t, w


def test_heading_options_see_the_wall_ahead_and_the_room_behind():
    r, amin, inc = _scan(0.0, 0.55)
    opts = dict((round(math.degrees(t)), c) for t, c in
                heading_options(r, amin, inc, 0.05, 12.0, FRONT, HALF_W))
    assert min(opts.values()) < 0.5                     # wall straight ahead
    ahead = min(opts.items(), key=lambda kv: abs(kv[0]))[1]
    behind = min(opts.items(), key=lambda kv: abs(abs(kv[0]) - 180))[1]
    assert ahead < 0.5 and behind > 5.0


def test_no_turn_before_hold_s():
    c = HoldRecoveryCore(hold_s=5.0)
    c.set_hold(True)
    t, w = _run(c, 0.0, 4.5)
    assert w == 0.0 and c.state == IDLE and c.turns == 0


def test_turns_after_hold_s_towards_the_nearest_clear_heading():
    c = HoldRecoveryCore(hold_s=5.0, ang_speed_rps=0.4)
    c.set_hold(True)
    t, w = _run(c, 0.0, 5.3, wall_dir=0.0)
    assert c.state == ROTATING and c.turns == 1
    assert abs(w) == 0.4
    assert any('turning' in e for e in c.pop_events())


def test_direction_is_towards_the_open_side():
    # wall straight ahead and on the left: the nearest open heading is on the right
    c = HoldRecoveryCore(hold_s=1.0)
    c.set_hold(True)
    r, amin, inc = _scan(0.0, 0.55)
    a = amin + inc * np.arange(len(r))
    r[(a > 0.0) & (a < math.radians(120))] = 0.55       # left side blocked too
    args = (r, amin, inc, 0.05, 12.0, FRONT, HALF_W)
    from serving_robot_navigation.monitoring.clearance import forward_clearance
    t = 0.0
    while t < 1.5:
        c.set_scan(t, forward_clearance(*args), heading_options(*args))
        w = c.step(t)
        t += 0.05
    assert c.state == ROTATING and w < 0.0              # clockwise = right


def test_stops_turning_when_the_way_ahead_is_clear_then_settles():
    c = HoldRecoveryCore(hold_s=1.0, settle_s=3.0)
    c.set_hold(True)
    t, w = _run(c, 0.0, 1.3, wall_dir=0.0)
    assert c.state == ROTATING
    t, w = _run(c, t, t + 0.5, wall_dir=180.0)          # now the wall is behind
    assert c.state == SETTLING and w == 0.0
    c.set_hold(False)                                   # limiter released
    _feed(c, t, 180.0)
    assert c.step(t + 0.1) == 0.0 and c.state == IDLE


def test_never_turns_while_a_goal_is_active():
    c = HoldRecoveryCore(hold_s=1.0)
    c.set_hold(True)
    c.set_goal_active(True)
    t, w = _run(c, 0.0, 5.0)
    assert w == 0.0 and c.turns == 0
    c.set_goal_active(False)
    t, w = _run(c, t, t + 1.5)
    assert c.state == ROTATING


def test_stops_when_a_goal_becomes_active_or_hold_ends():
    c = HoldRecoveryCore(hold_s=1.0)
    c.set_hold(True)
    t, w = _run(c, 0.0, 1.3)
    assert c.state == ROTATING
    c.set_goal_active(True)
    _feed(c, t)
    assert c.step(t) == 0.0 and c.state == IDLE

    c2 = HoldRecoveryCore(hold_s=1.0)
    c2.set_hold(True)
    t, w = _run(c2, 0.0, 1.3)
    c2.set_hold(False)
    _feed(c2, t)
    assert c2.step(t) == 0.0 and c2.state == IDLE


def test_gives_up_after_max_rotate_s_and_waits_before_trying_again():
    c = HoldRecoveryCore(hold_s=1.0, max_rotate_s=4.0, giveup_s=10.0)
    c.set_hold(True)
    # walls everywhere: no heading is clear
    r = np.full(720, 0.55)
    args = (r, -math.pi, 2.0 * math.pi / 720, 0.05, 12.0, FRONT, HALF_W)
    from serving_robot_navigation.monitoring.clearance import forward_clearance
    t, outs = 0.0, []
    while t < 8.0:
        c.set_scan(t, forward_clearance(*args), heading_options(*args))
        outs.append(c.step(t))
        t += 0.05
    assert c.turns == 1 and c.state == IDLE and outs[-1] == 0.0
    assert any('giving up' in e for e in c.pop_events())


def test_stale_scan_stops_the_turn():
    c = HoldRecoveryCore(hold_s=1.0, data_timeout_s=0.5)
    c.set_hold(True)
    t, w = _run(c, 0.0, 1.3)
    assert c.state == ROTATING
    assert c.step(t + 2.0) == 0.0 and c.state == IDLE   # no new scan for 2 s


def test_front_already_clear_means_no_turn():
    c = HoldRecoveryCore(hold_s=1.0)
    c.set_hold(True)
    t, w = _run(c, 0.0, 3.0, wall_dir=180.0)            # wall behind only
    assert c.turns == 0 and w == 0.0


if __name__ == '__main__':
    for name, fn in sorted(globals().items()):
        if name.startswith('test_') and callable(fn):
            fn()
            print('ok', name)
