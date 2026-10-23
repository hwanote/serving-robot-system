"""One shared numeric-policy source. No implicit substitution of team design clearance."""
from pathlib import Path
import math
import yaml

DEFAULTS = {
    'navigation': {'max_forward_mps':0.25, 'max_yaw_rps':0.80, 'goal_xy_tolerance_m':0.10, 'goal_yaw_tolerance_rad':0.25},
    'arrival': {'xy_tolerance_m':0.10, 'yaw_tolerance_rad':0.25, 'fine_engage_max_m':0.20, 'fine_timeout_s':45.0},
    'safety': {'design_clearance_m':2.50, 'person_slow_m':1.50, 'person_stop_m':0.80, 'stationary_replan_s':20.0, 'scan_timeout_s':0.50, 'command_timeout_s':0.50},
    'fleet': {'robot_separation_m':0.90, 'zone_approach_margin_m':0.45, 'blocked_alert_s':20.0},
}

def load_limits(path):
    if not path: raise ValueError('runtime_limits_file is required')
    config = yaml.safe_load(Path(path).read_text(encoding='utf8'))
    if not isinstance(config, dict): raise ValueError('Invalid runtime config')
    for section, fields in DEFAULTS.items():
        if not isinstance(config.get(section),dict): raise ValueError('Missing section '+section)
        for key in fields:
            v=config[section].get(key)
            if not isinstance(v,(int,float)) or isinstance(v,bool) or not math.isfinite(v) or v<=0:
                raise ValueError('Invalid positive value: '+section+'.'+key)
    if config['safety']['person_stop_m'] >= config['safety']['person_slow_m']:
        raise ValueError('Person STOP must be smaller than SLOW')
    if config['navigation']['goal_xy_tolerance_m'] > config['arrival']['xy_tolerance_m']:
        raise ValueError('Nav2 tolerance cannot exceed arrival acceptance')
    if config['arrival']['xy_tolerance_m'] > config['arrival']['fine_engage_max_m']:
        raise ValueError('Fine engagement must cover arrival tolerance')
    if 'dynamic_obstacles' in config:
        d=config['dynamic_obstacles']
        if not d['person_stop_m'] < config['safety']['person_slow_m'] < d['response_m'] < d['recognition_m']:
            raise ValueError('Dynamic distance roles must remain STOP < person SLOW < response < recognition')
        if not 0 < d['observe_stopped_s'] < d['blocked_alert_s'] < d['replan_check_s'] < d['detour_after_s']:
            raise ValueError('Dynamic time roles must remain observe < alert < check < detour')
        if d['person_stop_m'] != config['safety']['person_stop_m']:
            raise ValueError('Dynamic person STOP must equal the existing semantic STOP')
    return config
