"""Waypoints and approach selection. All tables share the same algorithm."""
from pathlib import Path
import math
import yaml
from .policy import distance


def read_waypoints(path):
    data=yaml.safe_load(Path(path).read_text(encoding='utf-8'))
    poses=data.get('destinations',{})
    for key,value in poses.items():
        if not isinstance(value,dict):raise ValueError('Invalid waypoint '+key)
        if all(k in value for k in ('x','y','yaw')):
            if not all(math.isfinite(float(value[k])) for k in ('x','y','yaw')):
                raise ValueError('Invalid coordinate '+key)
        elif 'approaches' not in value:raise ValueError('Missing coordinates '+key)
    return poses


def choose_waypoint(poses,name,xy=None,side='auto'):
    if name not in poses:raise ValueError('Unknown destination '+name)
    p=poses[name]
    if 'approaches' not in p:return {k:float(p[k]) for k in ('x','y','yaw')}
    candidates=p['approaches']
    if side!='auto':
        if side not in candidates:raise ValueError('Unknown table approach '+side)
        candidates={side:candidates[side]}
    if not candidates:raise ValueError('No approach for '+name)
    if xy is None:xy=(0.,0.)
    _,chosen=min(candidates.items(),key=lambda kv: distance(xy,(float(kv[1]['x']),float(kv[1]['y']))))
    return {k:float(chosen[k]) for k in ('x','y','yaw')}
