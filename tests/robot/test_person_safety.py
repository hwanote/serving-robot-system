"""Tests of the person safety state (no ROS, no YOLO).

    python3 tests/robot/test_person_safety.py
"""
import os
import sys

sys.path.insert(0, os.path.join(
    os.path.dirname(__file__), '..', '..', 'ros_ws', 'src',
    'serving_robot_navigation'))
from serving_robot_navigation.decision.person_safety import (  # noqa: E402
    NORMAL, SLOW, STALE, STOP, PersonSafetyCore)


INF = float('inf')


def frames(core, distances, t0=0.0, dt=0.25, clear=INF):
    t, state = t0, None
    for d in distances:
        state = core.update(t, [] if d is None else [d], front_clear_m=clear)
        t += dt
    return state


def test_starts_stale_until_the_first_frame():
    c = PersonSafetyCore()
    assert c.state == STALE and c.check_stale(0.0) == STALE
    assert c.update(0.0, []) == NORMAL


def test_far_person_and_nobody_are_normal():
    c = PersonSafetyCore()
    assert frames(c, [3.0, 2.5, None, 4.0]) == NORMAL


def test_getting_closer_is_immediate():
    c = PersonSafetyCore()
    assert c.update(0.0, [1.4]) == SLOW
    assert c.update(0.25, [0.6]) == SLOW           # median of 1.4, 0.6 = 1.0
    assert c.update(0.5, [0.6]) == STOP            # one more frame (0.25 s)
    c = PersonSafetyCore()
    assert c.update(0.0, [0.6]) == STOP            # first sight: immediate


def test_hysteresis_keeps_the_state_near_the_threshold():
    c = PersonSafetyCore()
    frames(c, [1.4])                       # SLOW
    assert frames(c, [1.6] * 6, 1.0) == SLOW      # 1.5 < 1.6 < 1.7: stays
    assert frames(c, [1.8] * 3, 3.0) == SLOW      # the median needs 2 frames, then 3 in a row
    assert frames(c, [1.8], 4.0) == NORMAL


def test_leaving_stop_goes_through_slow():
    c = PersonSafetyCore()
    frames(c, [0.5])
    assert c.state == STOP
    assert frames(c, [0.9] * 5, 1.0) == STOP      # 0.8 < 0.9 < 1.0: stays
    assert frames(c, [1.2] * 4, 3.0) == SLOW
    assert frames(c, [2.5] * 4, 5.0) == NORMAL


def test_person_gone_from_far_is_released_by_frame_counting():
    c = PersonSafetyCore()
    c.update(0.0, [1.4])                           # SLOW
    c.update(0.25, [1.9])                          # median 1.65: still close
    c.update(0.5, [1.9])                           # median 1.9: far
    assert c.state == SLOW
    assert frames(c, [None, None], 0.75) == NORMAL
    assert 'holding' not in c.reason


def test_person_lost_while_close_is_held_then_released_when_front_is_clear():
    c = PersonSafetyCore(lost_hold_s=3.0, clear_hold_s=2.0)
    frames(c, [1.0])
    assert c.state == SLOW
    assert frames(c, [None] * 8, 1.0) == SLOW      # 2 s lost: held
    assert 'holding' in c.reason
    # 3 s passed, front clear but not for 2 s yet
    assert frames(c, [None] * 4, 3.0) == SLOW
    assert frames(c, [None] * 8, 4.0) == NORMAL    # clear for 2 s: released
    assert c.reason == ''


def test_stop_lost_while_close_stays_stop_when_something_is_ahead():
    c = PersonSafetyCore(lost_hold_s=3.0)
    frames(c, [0.6])
    assert c.state == STOP
    # 20 s lost, the scan sees something 0.35 m ahead the whole time
    assert frames(c, [None] * 80, 1.0, clear=0.35) == STOP
    assert 'LiDAR: something 0.35 m ahead' in c.reason


def test_lidar_alone_can_raise_stop_and_slow():
    c = PersonSafetyCore()
    assert frames(c, [None] * 4, 0.0, clear=0.39) == STOP
    assert c.source == 'lidar'
    c = PersonSafetyCore()
    assert frames(c, [None] * 4, 0.0, clear=1.2) == SLOW


def test_worst_case_of_camera_and_lidar_wins():
    c = PersonSafetyCore()
    t = 0.0
    for _ in range(4):                             # camera says SLOW, LiDAR says STOP
        c.update(t, [1.0], front_clear_m=0.39)
        t += 0.25
    assert c.state == STOP and c.source == 'lidar'
    c = PersonSafetyCore()
    for _ in range(4):                             # camera says STOP, LiDAR sees nothing
        c.update(t, [0.6], front_clear_m=INF)
        t += 0.25
    assert c.state == STOP and c.source == 'camera'


def test_lidar_obstacle_removed_is_released_after_the_hold():
    c = PersonSafetyCore(lost_hold_s=3.0, clear_hold_s=2.0)
    frames(c, [None] * 8, 0.0, clear=0.39)
    assert c.state == STOP
    assert frames(c, [None] * 8, 2.0, clear=INF) == STOP   # 2 s after removal: held
    assert frames(c, [None] * 8, 4.0, clear=INF) == NORMAL


def test_unknown_clearance_counts_as_blocked():
    c = PersonSafetyCore(lost_hold_s=3.0)
    frames(c, [0.6])
    assert frames(c, [None] * 40, 1.0, clear=None) == STOP
    assert 'unknown' in c.reason


def test_external_hold_keeps_the_state_after_the_wait():
    c = PersonSafetyCore(lost_hold_s=3.0, clear_hold_s=2.0)
    frames(c, [0.6])
    t = 1.0
    for _ in range(80):                            # 20 s lost, LiDAR still holds
        c.update(t, [], external_hold=True, front_clear_m=INF)
        t += 0.25
    assert c.state == STOP and 'obstacle hold' in c.reason
    for _ in range(12):                            # hold ends, then 2 s clear
        c.update(t, [], external_hold=False, front_clear_m=INF)
        t += 0.25
    assert c.state == NORMAL


def test_person_seen_again_resets_the_clear_timer():
    c = PersonSafetyCore(lost_hold_s=3.0, clear_hold_s=2.0)
    frames(c, [0.6])
    frames(c, [None] * 12, 1.0, clear=0.4)         # 3 s lost, blocked
    frames(c, [None] * 2, 4.0)                     # clear for 0.25 s
    assert c.clear_since is not None
    frames(c, [0.9], 5.0)                          # seen again
    assert c.clear_since is None and c.state == STOP
    assert frames(c, [None] * 4, 5.5) == STOP      # new loss: held again


def test_person_seen_again_far_during_the_hold_follows_the_distance():
    c = PersonSafetyCore(lost_hold_s=3.0)
    frames(c, [0.6])
    frames(c, [None] * 4, 1.0)
    assert c.state == STOP
    assert frames(c, [2.5] * 4, 2.0) == NORMAL     # seen far: normal release rules


def test_stale_forgets_the_lost_person():
    c = PersonSafetyCore(stale_s=2.0)
    frames(c, [0.6])
    assert c.check_stale(5.0) == STALE
    assert c.update(6.0, []) == NORMAL             # nothing to hold any more


def test_one_wrong_depth_value_does_not_flip_the_state():
    c = PersonSafetyCore()
    assert frames(c, [2.5, 2.5]) == NORMAL
    assert c.update(1.0, [0.5]) == NORMAL          # median of 2.5, 2.5, 0.5
    assert c.update(1.25, [0.5]) == STOP           # median of 2.5, 0.5, 0.5


def test_nearest_person_counts():
    c = PersonSafetyCore()
    assert c.update(0.0, [3.0, 0.7, 2.0]) == STOP
    assert c.raw == 0.7


def test_sensor_stale_and_recovery():
    c = PersonSafetyCore(stale_s=2.0)
    frames(c, [0.6])
    assert c.check_stale(1.5) == STOP              # still fresh (last frame at 0.0)
    assert c.check_stale(2.5) == STALE
    assert 'no camera frame' in c.reason
    assert c.update(3.0, []) == NORMAL             # frames are back, nobody there
    c.check_stale(9.0)
    assert c.update(9.5, [0.6]) == STOP            # strict state is immediate again


if __name__ == '__main__':
    for name, fn in sorted(globals().items()):
        if name.startswith('test_') and callable(fn):
            fn()
            print('ok  ', name)
    print('all passed')
