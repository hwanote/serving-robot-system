"""Robot health summary (no ROS).

    python3 tests/robot/test_health.py
"""
import os
import sys

sys.path.insert(0, os.path.join(
    os.path.dirname(__file__), '..', '..', 'ros_ws', 'src',
    'serving_robot_navigation'))
from serving_robot_navigation.decision.health import (  # noqa: E402
    SCHEMA_VERSION, HealthCore, HealthParams)

ROBOT_STATES = {'BOOTING', 'IDLE', 'NAVIGATING', 'PAUSED', 'BLOCKED', 'DEGRADED',
                'ERROR'}           # OFFLINE is the fleet's (team guide Q3-3)


def mission(state='IDLE', stopped=0.0):
    return {'state': state, 'mission_id': 'm1' if state != 'IDLE' else '',
            'destination': 'kitchen' if state != 'IDLE' else '', 'stopped_s': stopped}


def limit(state='NORMAL', silent=False):
    return {'state': state, 'silent': silent, 'source': '', 'time': 0.0}


def healthy(t=0.0, **kw):
    """A booted robot: everything heard at time t."""
    h = HealthCore(HealthParams(**kw))
    h.start(t)
    feed(h, t)
    h.report(t)          # the node reports 5 times a second: boot is noticed
    return h


def feed(h, t, m=None, lim=None, std=(0.05, 0.05), det=None):
    h.on_scan(t)
    h.on_odom(t)
    h.on_mission(t, m or mission())
    h.on_person_limit(t, lim or limit())
    h.on_person_detection(t, det or {'silent_cameras': []})
    h.on_amcl(std[0] ** 2, std[0] ** 2, std[1] ** 2)
    h.on_pose(t, -3.2, -2.4, 0.0)


def test_message_follows_the_team_contract():
    r = healthy(map_revision=1).report(0.5)
    assert r['schema_version'] == SCHEMA_VERSION == 1 and r['robot_id'] == 'robot_01'
    assert r['robot_state'] in ROBOT_STATES
    assert r['safety_state'] in {'NORMAL', 'SLOW', 'STOP'}
    assert r['localization_state'] in {'UNINITIALIZED', 'OK', 'DEGRADED', 'LOST'}
    assert r['pose'] == {'frame_id': 'map', 'x_m': -3.2, 'y_m': -2.4, 'yaw_rad': 0.0,
                         'map_revision': 1}
    assert r['reasons'] == [] and r['robot_state'] == 'IDLE'
    assert all(c == c.upper() for c in r['reasons'])


def test_booting_until_everything_is_heard_then_error_after_the_timeout():
    h = HealthCore(HealthParams(boot_timeout_s=60.0))
    r = h.report(0.0)
    assert r['robot_state'] == 'BOOTING' and r['reasons'] == ['BOOTING']
    assert r['localization_state'] == 'UNINITIALIZED' and r['pose'] is None
    h.on_scan(10.0)
    assert h.report(10.0)['robot_state'] == 'BOOTING'
    r = h.report(61.0)                                    # never got a pose
    assert r['robot_state'] == 'ERROR'
    assert 'LOCALIZATION_UNINITIALIZED' in r['reasons']
    assert 'MISSION_STATUS_SILENT' in r['reasons'] and 'SCAN_STALE' in r['reasons']


def test_mission_states_map_to_robot_states():
    for mstate, robot in (('IDLE', 'IDLE'), ('DRIVING', 'NAVIGATING'),
                          ('WAIT_LOAD', 'PAUSED'), ('WAIT_HANDOVER', 'PAUSED'),
                          ('PAUSED', 'PAUSED')):
        h = healthy()
        feed(h, 1.0, m=mission(mstate))
        r = h.report(1.0)
        assert r['robot_state'] == robot, (mstate, r)
        assert r['mission']['state'] == mstate        # task state kept apart


def test_blocked_from_15_s_stopped_while_driving():
    h = healthy()
    feed(h, 1.0, m=mission('DRIVING', 14.9))
    assert h.report(1.0)['robot_state'] == 'NAVIGATING'
    feed(h, 2.0, m=mission('DRIVING', 15.0))
    r = h.report(2.0)
    assert r['robot_state'] == 'BLOCKED' and r['reasons'] == ['BLOCKED']
    feed(h, 3.0, m=mission('DRIVING', 20.0), lim=limit('STOP', silent=True))
    r = h.report(3.0)                                      # blocked AND degraded
    assert r['robot_state'] == 'BLOCKED' and r['reasons'] == ['BLOCKED', 'DETECTOR_SILENT']


def test_detector_and_camera_problems_are_degraded():
    h = healthy()
    feed(h, 1.0, lim=limit('SLOW', silent=True))
    r = h.report(1.0)
    assert r['robot_state'] == 'DEGRADED' and r['reasons'] == ['DETECTOR_SILENT']
    assert r['checks']['detector'] == 'silent' and r['safety_state'] == 'SLOW'
    h = healthy()
    feed(h, 1.0, det={'silent_cameras': ['camera_rear_right']})
    r = h.report(1.0)
    assert r['robot_state'] == 'DEGRADED' and r['reasons'] == ['CAMERA_SILENT']
    assert r['checks']['silent_cameras'] == ['camera_rear_right']


def test_errors_win_over_everything():
    h = healthy()
    feed(h, 1.0, m=mission('DRIVING', 20.0), lim=limit('STOP', silent=True))
    h.on_latched(True)
    r = h.report(1.0)
    assert r['robot_state'] == 'ERROR' and r['reasons'][0] == 'COLLISION_LATCHED'
    assert 'BLOCKED' in r['reasons'] and 'DETECTOR_SILENT' in r['reasons']
    assert r['safety_state'] == 'STOP'


def test_scan_and_mission_silence_after_boot_are_errors():
    h = healthy()
    h.on_mission(5.0, mission())
    h.on_person_limit(5.0, limit())
    h.on_pose(5.0, 0.0, 0.0, 0.0)
    h.on_odom(5.0)
    r = h.report(5.0)                                      # scan last heard at 0
    assert r['robot_state'] == 'ERROR' and r['reasons'] == ['SCAN_STALE']
    assert r['checks']['scan'] == 'stale'


def test_localization_thresholds():
    for std, yaw, want in ((0.10, 0.05, 'OK'), (0.25, 0.20, 'OK'),
                           (0.30, 0.05, 'DEGRADED'), (0.10, 0.30, 'DEGRADED'),
                           (1.0, 0.05, 'LOST')):
        h = healthy()
        feed(h, 1.0, std=(std, yaw))
        r = h.report(1.0)
        assert r['localization_state'] == want, (std, yaw, r)
    h = healthy()
    feed(h, 1.0, std=(0.4, 0.05))
    assert h.report(1.0)['robot_state'] == 'DEGRADED'
    feed(h, 2.0, std=(1.5, 0.05))
    assert h.report(2.0)['robot_state'] == 'ERROR'


def test_no_pose_for_2_s_is_lost():
    h = healthy()
    for t in (1.0, 2.0, 3.0):                              # everything but the pose
        h.on_scan(t)
        h.on_odom(t)
        h.on_mission(t, mission())
        h.on_person_limit(t, limit())
    r = h.report(3.0)
    assert r['localization_state'] == 'LOST' and 'LOCALIZATION_LOST' in r['reasons']


def test_safety_state():
    h = healthy()
    for lim_state, want in (('NORMAL', 'NORMAL'), ('SLOW', 'SLOW'), ('STOP', 'STOP'),
                            ('SENSOR_STALE', 'SLOW')):
        feed(h, 1.0, lim=limit(lim_state))
        assert h.report(1.0)['safety_state'] == want, lim_state
    feed(h, 2.0)
    h.on_hold(True)
    assert h.report(2.0)['safety_state'] == 'STOP'
    h.on_hold(False)
    for f in (h.on_scan, h.on_odom):                       # all but the limiter
        f(4.5)
    h.on_mission(4.5, mission())
    h.on_pose(4.5, 0.0, 0.0, 0.0)
    r = h.report(4.5)                                      # limiter silent since 2.0
    assert r['safety_state'] == 'STOP' and r['reasons'] == ['PERSON_LIMIT_SILENT']
    assert r['robot_state'] == 'DEGRADED' and r['checks']['detector'] == 'unknown'


def test_without_the_person_layer():
    h = healthy(person_expected=False)
    r = h.report(5.0 - 4.5)
    assert r['checks']['detector'] == 'off' and r['safety_state'] == 'NORMAL'
    assert r['reasons'] == []


def test_summary_changes_only_with_the_states():
    h = healthy()
    a = HealthCore.summary(h.report(0.5))
    feed(h, 0.8)
    assert HealthCore.summary(h.report(0.8)) == a          # time moves: same summary
    feed(h, 1.0, m=mission('DRIVING'))
    assert HealthCore.summary(h.report(1.0)) != a


def test_a_mission_on_hold_after_a_localization_loss_is_paused_not_blocked():
    h = healthy()
    held = dict(mission('DRIVING', stopped=40.0), loc_hold='RECOVERED')
    feed(h, 1.0, m=held)
    r = h.report(1.0)
    assert r['robot_state'] == 'PAUSED' and r['reasons'] == ['LOCALIZATION_HOLD']
    assert r['mission']['loc_hold'] == 'RECOVERED'
    # the same mission, stopped, but not on hold, is an obstacle wait
    feed(h, 2.0, m=mission('DRIVING', stopped=40.0))
    assert h.report(2.0)['robot_state'] == 'BLOCKED'
    # while the position is lost it is an error, whatever the mission does
    feed(h, 3.0, m=dict(held, loc_hold='LOST'), std=(1.2, 0.05))
    r = h.report(3.0)
    assert r['robot_state'] == 'ERROR' and 'LOCALIZATION_LOST' in r['reasons']


if __name__ == '__main__':
    for name, fn in sorted(globals().items()):
        if name.startswith('test_') and callable(fn):
            fn()
            print('ok  ', name)
    print('all passed')
