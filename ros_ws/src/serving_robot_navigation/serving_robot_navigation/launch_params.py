"""Parameter values for the launch files, computed from the shared files (no ROS).

Everything comes from config/common/runtime_limits.yaml and the Nav2 parameter
file, so no value is copied into a launch file or a node:

    limits = load_limits(share)
    monitor_zones(limits, nav2_params_file)    collision monitor polygons
    limiter_params(limits, nav2_params_file)   soft stop limiter
    person_params(limits, nav2_params_file)    (person_state, person_speed_limiter)
    executor_params(limits, watch_detector)    route executor rules
"""
import ast
import os

import yaml


def load_limits(share):
    path = os.path.join(share, 'config', 'common', 'runtime_limits.yaml')
    with open(path, 'r', encoding='utf-8') as f:
        return yaml.safe_load(f) or {}


def footprint_bounds(nav2_params_file):
    """Body size (front, rear, half_width) from the Nav2 local costmap footprint."""
    with open(nav2_params_file, 'r', encoding='utf-8') as f:
        nav2 = yaml.safe_load(f)
    fp = nav2['local_costmap']['local_costmap']['ros__parameters']['footprint']
    pts = ast.literal_eval(fp) if isinstance(fp, str) else fp
    xs = [p[0] for p in pts]
    ys = [p[1] for p in pts]
    return max(xs), -min(xs), max(abs(y) for y in ys)


def _pts(points):
    return '[' + ', '.join('[%.3f, %.3f]' % (x, y) for x, y in points) + ']'


def monitor_zones(limits, nav2_params_file):
    """Collision monitor polygon points, computed so no value is copied.

    Distances are measured from the body edge (footprint), not from the sensor.
    """
    front, rear, half_w = footprint_bounds(nav2_params_file)
    stop = float(limits['person_stop_distance_m'])
    slow = float(limits['person_slow_distance_m'])
    slow_speed = float(limits['slow_linear_mps'])
    max_ang = float(limits['max_angular_rps'])
    hard = float(limits['obstacle_reaction']['hard_stop_distance_m'])

    hard_pts = [(front + hard, half_w + hard), (front + hard, -half_w - hard),
                (-rear - hard, -half_w - hard), (-rear - hard, half_w + hard)]
    # Zone in front of the body only (starts at the front edge: no self hits).
    stop_pts = [(front + stop, half_w), (front + stop, -half_w),
                (front, -half_w), (front, half_w)]
    slow_pts = [(front + slow, half_w), (front + slow, -half_w),
                (front, -half_w), (front, half_w)]
    tiny = [(0.05, 0.05), (0.05, -0.05), (-0.05, -0.05), (-0.05, 0.05)]
    return {
        'HardStop.points': _pts(hard_pts),
        'StopZone.forward.points': _pts(stop_pts),
        'StopZone.rotate.points': _pts(tiny),
        'StopZone.reverse.points': _pts(tiny),
        'SlowZone.forward.points': _pts(slow_pts),
        'SlowZone.rotate.points': _pts(tiny),
        'SlowZone.reverse.points': _pts(tiny),
        # Speed cap inside the slow zone (turning speed is not reduced).
        'SlowZone.linear_limit': slow_speed,
        'SlowZone.angular_limit': max_ang,
    }


def limiter_params(limits, nav2_params_file):
    """Soft stop limiter settings, all taken from the shared limits."""
    front, _rear, half_w = footprint_bounds(nav2_params_file)
    comfort = limits['motion_comfort']
    react = limits['obstacle_reaction']
    return {
        'lin_accel_mps2': float(comfort['max_linear_accel_mps2']),
        'lin_decel_mps2': float(comfort['soft_stop_decel_mps2']),
        'ang_accel_rps2': float(comfort['max_angular_accel_rps2']),
        'ang_decel_rps2': float(comfort['max_angular_accel_rps2']),
        'cmd_timeout_s': float(limits['cmd_vel_timeout_s']),
        'resume_clear_distance_m': float(react['resume_clear_distance_m']),
        'resume_hold_s': float(react['resume_hold_s']),
        'collision_release_s': float(react['collision_release_s']),
        'front_m': float(front),
        'half_width_m': float(half_w),
    }


def camera_params(limits):
    """Camera list for person_detector and person_state (the same for both)."""
    ps = limits.get('person_safety', {})
    cams = [str(c) for c in ps.get('cameras', ['camera_front'])]
    primary = str(ps.get('primary_camera', cams[0]))
    return {'cameras': cams, 'primary_camera': primary}


def person_params(limits, nav2_params_file):
    """person_state (the judge) and person_speed_limiter settings, from the
    shared limits. person_detector (camera node) needs none of them."""
    front, _rear, half_w = footprint_bounds(nav2_params_file)
    react = limits['obstacle_reaction']
    ps = limits.get('person_safety', {})
    detector = {
        'stop_m': float(limits['person_stop_distance_m']),
        'slow_m': float(limits['person_slow_distance_m']),
        'front_m': float(front),
        'half_width_m': float(half_w),
        'clear_m': float(react['resume_clear_distance_m']),
        'clear_hold_s': float(react['resume_hold_s']),
        'front_half_deg': float(ps.get('front_half_angle_deg', 60.0)),
        'side_max_deg': float(ps.get('side_max_angle_deg', 120.0)),
        'side_slow_m': float(ps.get('side_slow_distance_m', 0.8)),
        'side_release_s': float(ps.get('side_release_s', 1.0)),
    }
    detector.update(camera_params(limits))
    limiter = {
        'slow_linear_mps': float(limits['slow_linear_mps']),
        'slow_angular_rps': float(ps.get('slow_angular_rps', 0.40)),
        'message_timeout_s': float(ps.get('message_timeout_s', 1.0)),
        'front_m': float(front),
        'half_width_m': float(half_w),
        'clear_m': float(react['resume_clear_distance_m']),
    }
    return detector, limiter


def health_params(limits, person_expected, map_revision=0):
    """robot_health settings (thresholds from the shared limits)."""
    hl = limits.get('health', {})
    r = limits['blocked_response']
    out = {'person_expected': bool(person_expected), 'map_revision': int(map_revision),
           'blocked_after_s': float(r['alarm_after_s'])}
    for key in ('boot_timeout_s', 'scan_stale_s', 'odom_stale_s', 'msg_stale_s',
                'pose_lost_s', 'ok_std_m', 'ok_std_rad', 'lost_std_m'):
        if key in hl:
            out[key] = float(hl[key])
    return out


def map_revision(share, store):
    """map_revision of maps/stores/<store>/map_meta.yaml (0 when unknown)."""
    path = os.path.join(share, 'maps', 'stores', store, 'map_meta.yaml')
    try:
        with open(path, encoding='utf-8') as f:
            return int((yaml.safe_load(f) or {}).get('map_revision', 0))
    except (OSError, ValueError, TypeError):
        return 0


def executor_params(limits, watch_detector=False):
    """Route executor settings (alarm / detour rules), from the shared limits."""
    r = limits['blocked_response']
    rail = limits.get('rail', {})
    dl = limits.get('delivery', {})
    ps = limits.get('person_safety', {})
    lh = limits.get('localization_hold', {})
    return {
        'watch_detector': bool(watch_detector),
        'detector_timeout_s': float(ps.get('message_timeout_s', 1.0)),
        'detector_startup_grace_s': float(ps.get('detector_startup_grace_s', 30.0)),
        'detector_alarm_repeat_s': float(ps.get('detector_alarm_repeat_s', 30.0)),
        'delivery_limit_s': float(dl.get('delivery_limit_s', 180.0)),
        'handover_timeout_s': float(dl.get('handover_timeout_s', 60.0)),
        'wait_repeat_s': float(dl.get('wait_repeat_s', 30.0)),
        'auto_confirm_s': float(dl.get('auto_confirm_s', 0.0)),
        'alarm_after_s': float(r['alarm_after_s']),
        'detour_after_s': float(r['detour_after_s']),
        'blocked_memory_s': float(r['blocked_memory_s']),
        'max_detours': int(r['max_detours']),
        'replan_interval_s': float(r['replan_interval_s']),
        'stall_window_s': float(r.get('stall_window_s', 3.0)),
        'stall_move_m': float(r.get('stall_move_m', 0.05)),
        'stall_turn_rad': float(r.get('stall_turn_rad', 0.15)),
        'rail_step_m': float(rail.get('step_m', 0.05)),
        'off_rail_m': float(rail.get('off_rail_m', 0.15)),
        'rail_resume_s': float(rail.get('resume_s', 3.0)),
        'loc_stable_s': float(lh.get('recover_stable_s', 2.0)),
        'health_max_age_s': float(lh.get('health_max_age_s', 3.0)),
    }
