"""Tests of the launch parameter helpers (no ROS).

    python3 tests/robot/test_launch_params.py
"""
import os
import shutil
import sys
import tempfile

ROOT = os.path.join(os.path.dirname(__file__), '..', '..')
sys.path.insert(0, os.path.join(ROOT, 'ros_ws', 'src', 'serving_robot_navigation'))
from serving_robot_navigation.launch_params import (  # noqa: E402
    camera_params, executor_params, health_params, map_revision, footprint_bounds, limiter_params, load_limits,
    monitor_zones, person_params)

NAV2 = os.path.join(ROOT, 'ros_ws', 'src', 'serving_robot_navigation', 'config',
                    'nav2.yaml')


def limits():
    # same layout as the installed share directory: <share>/config/common/...
    share = os.path.join(ROOT, 'ros_ws', 'src', 'serving_robot_navigation')
    path = os.path.join(ROOT, 'config', 'common', 'runtime_limits.yaml')
    import yaml
    with open(path, encoding='utf-8') as f:
        return yaml.safe_load(f), share


def test_footprint_bounds_are_positive():
    front, rear, half = footprint_bounds(NAV2)
    assert front > 0 and rear > 0 and half > 0


def test_load_limits_reads_the_shared_file():
    # the installed layout is <share>/config/common/runtime_limits.yaml
    with tempfile.TemporaryDirectory() as share:
        os.makedirs(os.path.join(share, 'config', 'common'))
        shutil.copy(os.path.join(ROOT, 'config', 'common', 'runtime_limits.yaml'),
                    os.path.join(share, 'config', 'common'))
        loaded = load_limits(share)
    assert loaded == limits()[0]


def test_monitor_zones_follow_the_shared_distances():
    lim, _ = limits()
    z = monitor_zones(lim, NAV2)
    assert z['SlowZone.linear_limit'] == lim['slow_linear_mps']
    assert z['SlowZone.angular_limit'] == lim['max_angular_rps']
    for key in ('HardStop.points', 'StopZone.forward.points', 'SlowZone.forward.points'):
        assert z[key].startswith('[[') and z[key].endswith(']]')


def test_limiter_and_person_params_come_from_the_limits():
    lim, _ = limits()
    lp = limiter_params(lim, NAV2)
    assert lp['cmd_timeout_s'] == lim['cmd_vel_timeout_s']
    det, limiter = person_params(lim, NAV2)
    assert det['stop_m'] == lim['person_stop_distance_m']
    assert det['slow_m'] == lim['person_slow_distance_m']
    assert limiter['slow_linear_mps'] == lim['slow_linear_mps']
    assert limiter['slow_angular_rps'] > 0
    ps = lim['person_safety']
    assert det['front_half_deg'] == ps['front_half_angle_deg']
    assert det['side_slow_m'] == ps['side_slow_distance_m']
    src = open(os.path.join(ROOT, 'ros_ws', 'src', 'serving_robot_navigation',
                            'scripts', 'decision', 'person_state.py'),
               encoding='utf-8').read()
    for key in det:                       # every judge key is a node parameter
        assert ("d('%s'" % key) in src, key
    cams = camera_params(lim)
    assert cams['primary_camera'] in cams['cameras'] and len(cams['cameras']) == 3
    assert det['cameras'] == cams['cameras']          # the judge gets the same list
    detector = open(os.path.join(ROOT, 'ros_ws', 'src', 'serving_robot_navigation',
                                 'scripts', 'monitoring', 'person_detector.py'),
                    encoding='utf-8').read()
    for key in cams:
        assert ("d('%s'" % key) in detector, key


def test_executor_params_switch_the_detector_watch():
    lim, _ = limits()
    assert executor_params(lim, True)['watch_detector'] is True
    assert executor_params(lim, False)['watch_detector'] is False
    p = executor_params(lim)
    assert p['alarm_after_s'] == lim['blocked_response']['alarm_after_s']
    assert p['detour_after_s'] == lim['blocked_response']['detour_after_s']
    assert p['max_detours'] == lim['blocked_response']['max_detours']
    assert p['stall_window_s'] == lim['blocked_response']['stall_window_s'] > 0
    assert p['off_rail_m'] == lim['rail']['off_rail_m'] > 0
    assert p['rail_step_m'] == lim['rail']['step_m']
    assert p['rail_resume_s'] == lim['rail']['resume_s']
    assert p['loc_stable_s'] == lim['localization_hold']['recover_stable_s'] > 0
    assert p['health_max_age_s'] == lim['localization_hold']['health_max_age_s'] > 0


def test_executor_params_match_the_node_parameters():
    """Every key the launch file passes is declared by route_executor.py."""
    lim, _ = limits()
    src = open(os.path.join(ROOT, 'ros_ws', 'src', 'serving_robot_navigation',
                            'scripts', 'command', 'route_executor.py'),
               encoding='utf-8').read()
    for key in executor_params(lim, True):
        assert ("d('%s'" % key) in src or ("'%s'" % key) in src, key



def test_health_params_match_the_node_and_the_limits():
    lim, share = limits()
    hp = health_params(lim, True, 3)
    assert hp['person_expected'] is True and hp['map_revision'] == 3
    assert hp['blocked_after_s'] == lim['blocked_response']['alarm_after_s']
    assert hp['ok_std_m'] == lim['health']['ok_std_m']
    src = open(os.path.join(ROOT, 'ros_ws', 'src', 'serving_robot_navigation',
                            'scripts', 'decision', 'robot_health.py'),
               encoding='utf-8').read()
    for key in hp:
        assert ("d('%s'" % key) in src or ("'%s'" % key) in src, key


def test_map_revision_is_read_from_map_meta():
    with tempfile.TemporaryDirectory() as share:
        os.makedirs(os.path.join(share, 'maps', 'stores', 's1'))
        with open(os.path.join(share, 'maps', 'stores', 's1', 'map_meta.yaml'), 'w') as f:
            f.write('map_revision: 4\n')
        assert map_revision(share, 's1') == 4
        assert map_revision(share, 'missing') == 0
    real = os.path.join(ROOT)                        # maps/stores/demo_store in the repo
    assert map_revision(real, 'demo_store') >= 1


if __name__ == '__main__':
    for name, fn in sorted(globals().items()):
        if name.startswith('test_'):
            fn()
    print('all passed')
