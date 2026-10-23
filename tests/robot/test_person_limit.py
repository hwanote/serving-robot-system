"""Tests of the speed limit from the person state (no ROS).

    python3 tests/robot/test_person_limit.py
"""
import os
import sys

sys.path.insert(0, os.path.join(
    os.path.dirname(__file__), '..', '..', 'ros_ws', 'src',
    'serving_robot_navigation'))
from serving_robot_navigation.monitoring.detector_watch import (  # noqa: E402
    CameraWatch, DetectorWatch)
from serving_robot_navigation.motion.person_limit import (  # noqa: E402
    NORMAL, SLOW, STALE, STOP, PersonLimitCore, parse_message, parse_state)


def core():
    return PersonLimitCore(0.10, 0.40, 1.0)


def test_normal_passes_through():
    c = core()
    c.on_message(0.0, NORMAL)
    assert c.limit(0.1, 0.25, 0.8) == (0.25, 0.8, NORMAL)
    assert c.limit(0.1, -0.2, -0.5) == (-0.2, -0.5, NORMAL)


def test_slow_clamps_both_directions():
    c = core()
    c.on_message(0.0, SLOW)
    assert c.limit(0.1, 0.25, 0.8) == (0.10, 0.40, SLOW)
    assert c.limit(0.1, -0.25, -0.8) == (-0.10, -0.40, SLOW)
    assert c.limit(0.1, 0.05, 0.1) == (0.05, 0.1, SLOW)       # lower commands stay


def test_stop_blocks_forward_but_allows_backing_and_turning():
    c = core()
    c.on_message(0.0, STOP)
    assert c.limit(0.1, 0.25, 0.0) == (0.0, 0.0, STOP)         # forward: zero
    assert c.limit(0.1, -0.25, 0.0) == (-0.10, 0.0, STOP)      # backing up: capped
    assert c.limit(0.1, -0.05, 0.0) == (-0.05, 0.0, STOP)
    assert c.limit(0.1, 0.0, 0.8) == (0.0, 0.40, STOP)         # turning: capped
    assert c.limit(0.1, 0.0, -0.8) == (0.0, -0.40, STOP)
    assert c.limit(0.1, 0.25, 0.8) == (0.0, 0.40, STOP)        # forward + turn: turn only


def test_no_message_yet_counts_as_stale_slow():
    c = core()
    assert c.limit(0.0, 0.25, 0.8) == (0.10, 0.40, STALE)


def test_old_message_counts_as_stale_slow():
    c = core()
    c.on_message(0.0, NORMAL)
    assert c.limit(1.0, 0.25, 0.8)[2] == NORMAL               # exactly the timeout
    assert c.limit(1.2, 0.25, 0.8) == (0.10, 0.40, STALE)     # detector silent


def test_silent_detector_keeps_a_stop():
    c = core()
    c.on_message(0.0, STOP)
    assert c.limit(0.5, 0.25, 0.0) == (0.0, 0.0, STOP)
    assert c.limit(5.0, 0.25, 0.0) == (0.0, 0.0, STOP)        # silent for 5 s
    assert c.is_silent(5.0)
    c.on_message(5.5, NORMAL)                                  # it speaks again
    assert c.limit(5.6, 0.25, 0.8) == (0.25, 0.8, NORMAL)
    assert not c.is_silent(5.6)


def test_silent_stop_is_held_only_while_the_lidar_sees_something_ahead():
    c = core()
    c.on_message(0.0, STOP)
    c.set_clearance(4.9, 0.5)                                  # something 0.5 m ahead
    assert c.limit(5.0, 0.25, 0.8) == (0.0, 0.40, STOP)
    c.set_clearance(5.0, 1.2)                                  # the way ahead is clear
    assert c.limit(5.1, 0.25, 0.8) == (0.10, 0.40, STALE)      # drives on, slowly
    c.set_clearance(5.2, float('inf'))
    assert c.limit(5.3, 0.25, 0.8)[2] == STALE
    c.set_clearance(5.4, 0.9)                                  # blocked again
    assert c.limit(5.5, 0.25, 0.8)[2] == STOP


def test_silent_stop_with_unknown_or_old_scan_stays_stop():
    c = core()
    c.on_message(0.0, STOP)
    assert c.limit(5.0, 0.25, 0.0)[2] == STOP                  # no scan at all
    c.set_clearance(5.0, 3.0)
    assert c.limit(5.2, 0.25, 0.0)[2] == STALE                 # fresh and clear
    assert c.limit(5.7, 0.25, 0.0)[2] == STOP                  # scan older than 0.5 s


def test_live_stop_ignores_the_lidar_clearance():
    c = core()
    c.on_message(0.0, STOP)
    c.set_clearance(0.0, 5.0)                                  # camera sees a person, LiDAR not
    assert c.limit(0.1, 0.25, 0.0)[2] == STOP


def test_detector_watch_alarms_once_repeats_and_recovers():
    w = DetectorWatch(timeout_s=1.0, startup_grace_s=30.0, repeat_s=30.0)
    w.heard(0.0)
    assert w.step(0.5) == []
    assert w.step(1.2) == ['SILENT']
    assert w.step(20.0) == []
    assert w.step(31.5) == ['SILENT_REPEAT']
    w.heard(32.0)
    assert w.step(32.1) == ['RECOVERED']
    assert w.step(40.0) == ['SILENT']                          # silent again


def test_detector_watch_gives_a_never_started_detector_a_grace_time():
    w = DetectorWatch(timeout_s=1.0, startup_grace_s=30.0, repeat_s=30.0)
    assert w.step(0.0) == []
    assert w.step(29.0) == []
    assert w.step(30.5) == ['SILENT']
    w.heard(31.0)
    assert w.step(31.1) == ['RECOVERED']


def test_silent_slow_is_still_the_stale_limit():
    c = core()
    c.on_message(0.0, SLOW)
    assert c.limit(5.0, 0.25, 0.8) == (0.10, 0.40, STALE)


def test_detector_stale_state_counts_as_slow():
    c = core()
    c.on_message(0.0, STALE)
    assert c.limit(0.1, 0.25, 0.8) == (0.10, 0.40, STALE)


def test_unknown_state_text_counts_as_stale():
    c = core()
    c.on_message(0.0, 'BANANA')
    assert c.limit(0.1, 0.25, 0.8) == (0.10, 0.40, STALE)


def test_recovery_after_a_new_message():
    c = core()
    c.on_message(0.0, STOP)
    assert c.limit(0.1, 0.25, 0.0)[2] == STOP
    c.on_message(0.5, NORMAL)
    assert c.limit(0.6, 0.25, 0.0) == (0.25, 0.0, NORMAL)


def test_status_is_the_applied_decision():
    c = core()
    assert c.status(0.0) == {'state': STALE, 'silent': False, 'source': ''}
    c.on_message(0.0, STOP, 'camera')
    assert c.status(0.5) == {'state': STOP, 'silent': False, 'source': 'camera'}
    c.on_message(0.6, SLOW, 'lidar')
    assert c.status(0.7) == {'state': SLOW, 'silent': False, 'source': ''}
    c.on_message(1.0, STOP, 'camera')
    # the detector dies: STOP held only while the LiDAR sees something ahead
    c.set_clearance(5.0, 0.5)
    assert c.status(5.0) == {'state': STOP, 'silent': True, 'source': 'lidar'}
    c.set_clearance(5.1, 2.0)                                  # the way ahead is clear
    assert c.status(5.2) == {'state': STALE, 'silent': True, 'source': ''}


def test_parse_message():
    assert parse_message('{"state": "STOP", "source": "lidar"}') == (STOP, 'lidar')
    assert parse_message('{"state": "SLOW"}') == (SLOW, '')
    assert parse_message('{"state": "SLOW", "source": null}') == (SLOW, '')
    assert parse_message('nope') is None
    assert parse_message('{"state": 3}') is None


def test_parse_state():
    assert parse_state('{"state": "SLOW", "reason": ""}') == SLOW
    assert parse_state('not json') is None
    assert parse_state('{"state": 3}') is None
    assert parse_state('[1, 2]') is None



def test_camera_watch_alarms_once_repeats_and_recovers():
    w = CameraWatch(repeat_s=30.0)
    assert w.update(0.0, []) == []
    assert w.update(1.0, ['camera_rear_left']) == [('SILENT', 'camera_rear_left')]
    assert w.update(20.0, ['camera_rear_left']) == []
    assert w.update(31.0, ['camera_rear_left', 'camera_front']) == [
        ('SILENT', 'camera_front'), ('SILENT_REPEAT', 'camera_rear_left')]
    assert w.update(32.0, ['camera_front']) == [('RECOVERED', 'camera_rear_left')]
    assert w.update(33.0, []) == [('RECOVERED', 'camera_front')]
    assert w.update(34.0, []) == []


if __name__ == '__main__':
    for name, fn in sorted(globals().items()):
        if name.startswith('test_') and callable(fn):
            fn()
            print('ok  ', name)
    print('all passed')
