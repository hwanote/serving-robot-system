"""Offline kinematic world: runs the SAME PersonRules code as manager.tick on the
configured lanes. No Gazebo, no robot, no random pauses (constant lane speed).
Evidence from this world is NOT a Gazebo result."""
import copy
import itertools
import math
import sys
from pathlib import Path

import yaml

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from serving_robot_person_simulator.policy import PersonRules, Walker, validate  # noqa: E402

DEADLOCK_KEYS = {'person_deadlock_timeout_s': 10.0, 'person_hard_min_radius_m': 0.50}


def load_cfg(deadlock=True):
    cfg = yaml.safe_load((ROOT / 'config' / 'policy.yaml').read_text())
    for k in DEADLOCK_KEYS:
        cfg.pop(k, None)
    if deadlock:
        cfg.update(DEADLOCK_KEYS)
    return validate(cfg)


def run(cfg, ids, speeds, seconds=900.0, dt=0.1):
    rules = PersonRules(cfg)
    walkers, pos = {}, {}
    for i in ids:
        a, b = cfg['lanes'][i - 1]['start'], cfg['lanes'][i - 1]['end']
        walkers[i] = Walker(i, f'test_person_{i:02d}', tuple(a), tuple(b), speed_mps=speeds[i], paused=False)
        pos[i] = list(a)
    turns = {i: 0 for i in ids}
    retreats_of = lambda: sum(walkers[i].person_retreats for i in ids)
    dist = {i: 0.0 for i in ids}
    min_sep, stopped_run, max_stopped, passes = math.inf, 0.0, 0.0, 0
    prev_pass = {i: False for i in ids}
    for k in range(int(seconds / dt)):
        now = (k + 1) * dt
        for i in ids:
            w, p = walkers[i], tuple(pos[i])
            peers = [(j, tuple(pos[j]), walkers[j].published_v, walkers[j]) for j in ids if j != i]
            rules.begin(w, now, p, peers)
            tgt = w.end if w.target_end else w.start
            dx, dy = tgt[0] - p[0], tgt[1] - p[1]
            length = math.hypot(dx, dy)
            if length < 0.12:
                w.target_end = not w.target_end
                w.published_v = (0.0, 0.0)
                turns[i] += 1
                continue
            v = (w.speed_mps * dx / length, w.speed_mps * dy / length)
            blocked = rules.hold_blocked(w, p, v, peers) or rules.future_blocked(w, p, v, peers)
            w.published_v = (0.0, 0.0) if blocked else v
            rules.mark(i, w, now, p, peers, blocked)
            if w.person_pass and not prev_pass[i]:
                passes += 1
            prev_pass[i] = w.person_pass
        any_move = False
        for i in ids:
            vx, vy = walkers[i].published_v
            pos[i][0] += vx * dt
            pos[i][1] += vy * dt
            dist[i] += math.hypot(vx, vy) * dt
            any_move |= (vx, vy) != (0.0, 0.0)
        stopped_run = 0.0 if any_move else stopped_run + dt
        max_stopped = max(max_stopped, stopped_run)
        for a, b in itertools.combinations(ids, 2):
            min_sep = min(min_sep, math.dist(pos[a], pos[b]))
    return {'turns': turns, 'distance': dist, 'min_sep': min_sep,
            'max_all_stopped_s': max_stopped, 'passes': passes, 'retreats': retreats_of()}
