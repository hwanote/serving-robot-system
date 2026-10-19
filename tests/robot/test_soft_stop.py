"""Scenario tests for the soft stop limiter core (no ROS needed).

    python3 tests/robot/test_soft_stop.py
"""
import math
import os
import sys

sys.path.insert(0, os.path.join(
    os.path.dirname(__file__), '..', '..', 'ros_ws', 'src',
    'serving_robot_navigation'))
from serving_robot_navigation.monitoring.clearance import (  # noqa: E402
    forward_clearance, touched_name)
from serving_robot_navigation.motion.soft_stop import (  # noqa: E402
    HARD_STOP, STOP_ZONE, LimiterCore)

DT = 0.02


def make():
    return LimiterCore(lin_accel=0.4, lin_decel=0.3, ang_accel=0.8,
                       ang_decel=0.8, cmd_timeout=0.5, clear_distance=1.0,
                       hold_s=2.0)


def run(core, t, seconds, cmd=(0.0, 0.0), clearance=None):
    """Advance `seconds`, feeding the command and clearance every tick."""
    out = []
    for _ in range(int(round(seconds / DT))):
        core.set_cmd(t, *cmd)
        if clearance is not None:
            core.set_clearance(t, clearance)
        t += DT
        out.append(core.step(t, DT))
    return t, out


def test_soft_stop_profile():
    c, t = make(), 0.0
    t, _ = run(c, t, 2.0, cmd=(0.25, 0.0))
    assert abs(c.lin - 0.25) < 1e-9
    t, out = run(c, t, 1.5, cmd=(0.0, 0.0))        # monitor says stop
    speeds = [o[0] for o in out]
    steps = [a - b for a, b in zip([0.25] + speeds, speeds)]
    assert max(abs(s) for s in steps) <= 0.3 * DT + 1e-9, 'decel too hard'
    assert speeds[-1] == 0.0
    stop_time = next(i for i, s in enumerate(speeds) if s == 0.0) * DT
    assert 0.78 < stop_time < 0.88, stop_time        # 0.25 / 0.3 = 0.83 s
    dist = sum(s * DT for s in speeds)
    assert 0.09 < dist < 0.12, dist                  # ~10 cm


def test_hard_stop_is_immediate():
    c, t = make(), 0.0
    t, _ = run(c, t, 2.0, cmd=(0.25, 0.0))
    c.on_state(HARD_STOP, 1)
    c.set_cmd(t, 0.0, 0.0)
    t += DT
    assert c.step(t, DT) == (0.0, 0.0)


def test_resume_rule():
    c, t = make(), 0.0
    t, _ = run(c, t, 2.0, cmd=(0.25, 0.0))
    c.on_state(STOP_ZONE, 1)
    # Nav2 keeps asking to drive; obstacle still 0.7 m ahead -> stays at zero
    t, out = run(c, t, 3.0, cmd=(0.10, 0.0), clearance=0.7)
    assert out[-1][0] == 0.0 and c.holding
    # obstacle leaves the stop zone but is only 0.9 m away: still waiting
    t, out = run(c, t, 3.0, cmd=(0.10, 0.0), clearance=0.9)
    assert out[-1][0] == 0.0 and c.holding
    # 1.2 m: must stay clear for 2 s first
    t, out = run(c, t, 1.9, cmd=(0.10, 0.0), clearance=1.2)
    assert out[-1][0] == 0.0 and c.holding, 'resumed before 2 s'
    t, out = run(c, t, 0.5, cmd=(0.10, 0.0), clearance=1.2)
    assert not c.holding and out[-1][0] > 0.0
    # a blink (clearance drops again) restarts the 2 s timer
    c2, t2 = make(), 0.0
    c2.on_state(STOP_ZONE, 1)
    t2, _ = run(c2, t2, 1.5, cmd=(0.1, 0.0), clearance=1.5)
    t2, _ = run(c2, t2, 0.1, cmd=(0.1, 0.0), clearance=0.6)
    t2, _ = run(c2, t2, 1.5, cmd=(0.1, 0.0), clearance=1.5)
    assert c2.holding, 'timer was not restarted'


def test_resume_ramps_with_acceleration_limit():
    c, t = make(), 0.0
    c.on_state(STOP_ZONE, 1)
    t, _ = run(c, t, 2.5, cmd=(0.25, 0.0), clearance=2.0)
    assert not c.holding
    t, out = run(c, t, 1.0, cmd=(0.25, 0.0), clearance=2.0)
    speeds = [o[0] for o in out]
    steps = [b - a for a, b in zip([c.lin - 0.0] + speeds, speeds)]
    assert max(speeds) <= 0.25 + 1e-9
    assert all(s <= 0.4 * DT + 1e-9 for s in
               [b - a for a, b in zip(speeds, speeds[1:])]), 'accel too hard'


def test_turning_allowed_while_waiting():
    c, t = make(), 0.0
    c.on_state(STOP_ZONE, 1)
    t, out = run(c, t, 3.0, cmd=(0.0, 0.5), clearance=0.7)
    assert c.holding and abs(out[-1][1] - 0.5) < 1e-9 and out[-1][0] == 0.0


def test_unknown_or_old_scan_keeps_waiting():
    c, t = make(), 0.0
    c.on_state(STOP_ZONE, 1)
    t, out = run(c, t, 5.0, cmd=(0.1, 0.0), clearance=None)
    assert c.holding and out[-1][0] == 0.0
    c.set_clearance(t, 5.0)                          # one good scan, then silence
    t, out = run(c, t, 5.0, cmd=(0.1, 0.0), clearance=None)
    assert c.holding


def test_command_timeout_stops_softly():
    c, t = make(), 0.0
    t, _ = run(c, t, 2.0, cmd=(0.25, 0.0))
    for _ in range(100):                              # no more commands
        t += DT
        c.step(t, DT)
    assert c.lin == 0.0


def test_forward_clearance():
    n = 720
    ang_min, inc = -math.pi, 2 * math.pi / n
    rng = [math.inf] * n
    assert forward_clearance(rng, ang_min, inc, 0.1, 12, 0.26, 0.20) == math.inf
    i0 = int(round((0.0 - ang_min) / inc))            # straight ahead
    rng[i0] = 1.5                                      # 1.5 m from the centre
    assert abs(forward_clearance(rng, ang_min, inc, 0.1, 12, 0.26, 0.20)
               - (1.5 - 0.26)) < 1e-6
    rng2 = [math.inf] * n
    i1 = int(round((math.radians(60) - ang_min) / inc))  # far to the side
    rng2[i1] = 1.0
    assert forward_clearance(rng2, ang_min, inc, 0.1, 12, 0.26, 0.20) == math.inf


def test_contact_latches_and_zeroes_at_once():
    c, t = make(), 0.0
    t, _ = run(c, t, 2.0, cmd=(0.25, 0.0))
    assert c.on_contact(t, 'wall') is True           # newly latched
    c.set_cmd(t, 0.25, 0.0)
    t += DT
    assert c.step(t, DT) == (0.0, 0.0)               # no ramp
    assert c.collisions == 1
    t, out = run(c, t, 5.0, cmd=(0.25, 0.3), clearance=5.0)
    assert all(o == (0.0, 0.0) for o in out), 'latched robot must not move'
    assert c.latched


def test_second_contact_does_not_count_twice():
    c, t = make(), 0.0
    assert c.on_contact(t, 'a') is True
    t += 0.1
    assert c.on_contact(t, 'a') is False
    assert c.collisions == 1


def test_clear_refused_while_touching_then_allowed_after_3s():
    c, t = make(), 0.0
    c.on_contact(t, 'box')
    ok, msg = c.try_clear(t + 1.0)
    assert not ok and c.latched, msg
    c.on_contact(t + 2.0, 'box')                     # still touching
    ok, msg = c.try_clear(t + 4.0)                   # only 2 s since last touch
    assert not ok and c.latched, msg
    ok, msg = c.try_clear(t + 5.01)                  # 3.01 s since last touch
    assert ok and not c.latched, msg


def test_clear_when_not_latched_is_harmless():
    c = make()
    ok, msg = c.try_clear(1.0)
    assert ok and not c.latched and not c.holding


def test_after_clear_resume_rule_applies():
    c, t = make(), 0.0
    c.on_contact(t, 'box')
    t += 3.5
    assert c.try_clear(t)[0]
    # box still right in front: do not drive forward, but reversing is fine
    t, out = run(c, t, 3.0, cmd=(0.2, 0.0), clearance=0.05)
    assert all(o[0] == 0.0 for o in out), 'must not drive into the touched thing'
    t, out = run(c, t, 2.0, cmd=(-0.1, 0.0), clearance=0.05)
    assert out[-1][0] < 0.0, 'backing away must be possible'
    # way ahead clear for hold_s -> forward allowed again
    t, out = run(c, t, 3.0, cmd=(0.2, 0.0), clearance=3.0)
    assert out[-1][0] > 0.0
    # a new touch latches again and counts
    assert c.on_contact(t, 'wall') is True and c.collisions == 2


class _E:
    def __init__(self, name):
        self.name = name


class _C:
    def __init__(self, a, b):
        self.collision1, self.collision2 = _E(a), _E(b)


def test_touched_name_ignores_self_contacts():
    own = 'robot_01::base_footprint::base_link_fixed_joint_lump__front_collision'
    wheel = 'robot_01::left_wheel_link::collision'
    assert touched_name([], 'robot_01') == ''
    assert touched_name([_C(own, wheel)], 'robot_01') == ''
    assert touched_name([_C(own, 'cafe_walls::link::wall_1')], 'robot_01') \
        == 'cafe_walls::link::wall_1'
    assert touched_name([_C('test_box::link::collision', own)], 'robot_01') \
        == 'test_box::link::collision'
    assert touched_name([_C(own, wheel), _C(own, 'test_box::link::collision')],
                        'robot_01') == 'test_box::link::collision'
    # another robot counts as the outside world
    assert touched_name([_C(own, 'robot_02::link::collision')], 'robot_01') \
        == 'robot_02::link::collision'


if __name__ == '__main__':
    for name, fn in sorted(globals().items()):
        if name.startswith('test_') and callable(fn):
            fn()
            print('ok  ', name)
    print('all passed')
