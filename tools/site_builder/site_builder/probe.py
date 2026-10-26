"""The folder the executor is started on for a drive test.

It is a copy of config/common with the candidate's entities / waypoints / zones laid over
it, plus one extra pose per tour step (probe_01 ...): the executor only takes names of
poses, and a probe pose is "node X, facing the way the step arrives". The candidate folder
itself is never changed.

Start the robot side with:  SERVING_ROBOT_CONFIG_DIR=<this folder>  (executor, reset, go_to)
"""
import copy
import json
import os
import shutil

import yaml

from .tour import plan

HEADER = ('# Drive-test copy made by tools/site_builder. Not a project config: '
          'it only exists so the route executor can be sent to every node.\n')
OVERLAY = ('entities.yaml', 'waypoints.yaml', 'zones.yaml')


class ProbeError(ValueError):
    pass


def _load(path):
    with open(path, 'r', encoding='utf-8') as f:
        d = yaml.safe_load(f)
    if not isinstance(d, dict):
        raise ProbeError('empty or invalid file: %s' % path)
    return d


def home_node(waypoints):
    """Node of the first standby pose (the robot's home)."""
    for name, p in (waypoints.get('poses') or {}).items():
        if p.get('kind') == 'standby':
            return str(p['node'])
    raise ProbeError('the candidate has no standby pose: a tour needs a home node')


def build(candidate_dir, common_dir, out_dir, speed_mps=0.25, omega_rps=0.8):
    """Write the drive-test folder and tour.json into out_dir; returns the tour dict."""
    wp = _load(os.path.join(candidate_dir, 'waypoints.yaml'))
    tour = plan(wp, home_node(wp), speed_mps, omega_rps)
    if os.path.abspath(out_dir) in (os.path.abspath(candidate_dir), os.path.abspath(common_dir)):
        raise ProbeError('out_dir must be a new folder, not the candidate or the common config')
    os.makedirs(out_dir, exist_ok=True)
    for name in os.listdir(common_dir):
        src = os.path.join(common_dir, name)
        if os.path.isfile(src):
            shutil.copy2(src, os.path.join(out_dir, name))
    for name in OVERLAY:
        shutil.copy2(os.path.join(candidate_dir, name), os.path.join(out_dir, name))
    poses = copy.deepcopy(wp.get('poses') or {})
    for s in tour['steps']:
        name = 'probe_%02d' % s['i']
        if name in poses:
            raise ProbeError("pose name '%s' is already used by the candidate" % name)
        poses[name] = {'node': s['to'], 'yaw_deg': s['heading_deg'], 'kind': 'probe'}
        s['probe'] = name
    wp2 = copy.deepcopy(wp)
    wp2['poses'] = poses
    with open(os.path.join(out_dir, 'waypoints.yaml'), 'w', encoding='utf-8') as f:
        f.write(HEADER)
        yaml.safe_dump(wp2, f, sort_keys=False, allow_unicode=True, default_flow_style=None)
    tour['candidate_dir'] = os.path.abspath(candidate_dir)
    tour['test_config_dir'] = os.path.abspath(out_dir)
    with open(os.path.join(out_dir, 'tour.json'), 'w', encoding='utf-8') as f:
        json.dump(tour, f, indent=1, ensure_ascii=False)
    return tour
