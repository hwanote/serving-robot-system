"""map + labels -> route graph, stop poses, HardStop pre-check, temporary zones.

generate(map_yaml, labels, params) returns a Result; write_outputs() writes
waypoints.yaml / entities.yaml / zones.yaml / preview.json (status: provisional).
Pure files in, files out: no ROS import.
"""
import json
import math
import os

import numpy as np
import yaml

from . import sitegraph as sg
from .clearance import clearance_map, exact_sampler, rasterize_rects
from .extract import clearance_sampler, generate_graph
from .labels import load_labels
from .mapio import MapData
from .params import Params


class PipelineError(RuntimeError):
    pass


def _snap_yaw(deg, tol):
    for k in (-180, -90, 0, 90, 180):
        if abs(deg - k) <= tol:
            return 180.0 if k == -180 else float(k)
    return deg


def raycast(blocked_at, start, yaw_rad, max_m=1.5, step=0.01):
    """Distance from start along yaw to the first blocked cell, or None."""
    c, s = math.cos(yaw_rad), math.sin(yaw_rad)
    for i in range(int(max_m / step) + 1):
        d = i * step
        if blocked_at(start[0] + c * d, start[1] + s * d):
            return d
    return None


class Result:
    def __init__(self):
        self.graph = None
        self.poses = {}        # label id -> dict(node, x, y, yaw_deg, kind, faces, aliases, ...)
        self.excluded = []     # [{id, reason}]
        self.warnings = []
        self.node_checks = {}
        self.edge_checks = {}
        self.zones = []
        self.entities = {'tables': {}, 'fixtures': {}}
        self.map_revision = None
        self.params = None
        self.labels = []
        self.adjust_info = None
        self.approach = {}     # table label id -> what the approach branch step did

    def count(self, level):
        return (sum(1 for c in self.node_checks.values() if c['level'] == level)
                + sum(1 for c in self.edge_checks.values() if c['level'] == level)
                + sum(1 for c in self.poses.values() if c['level'] == level))


def table_rect_list(labels, par):
    """{label id: [(x0, x1, y0, y1), ...]} table top and chairs of every table label."""
    rects = {}
    for l in labels:
        if l['kind'] != 'table':
            continue
        top = l.get('top_size_m') or {'x': par.default_top_m, 'y': par.default_top_m}
        sx, sy = top['x'] / 2, top['y'] / 2
        r = [(l['x'] - sx, l['x'] + sx, l['y'] - sy, l['y'] + sy)]
        for c in l.get('chairs') or []:
            h = c['size_m'] / 2
            r.append((l['x'] + c['dx'] - h, l['x'] + c['dx'] + h,
                      l['y'] + c['dy'] - h, l['y'] + c['dy'] + h))
        rects[l['id']] = r
    return rects


def _level(margin, fail, warn):
    return 'FAIL' if margin < fail else ('WARN' if margin < warn else 'OK')


def hardstop_points(x, y, yaw_rad, par, step=0.05):
    """Perimeter of the HardStop rectangle placed at a pose."""
    pts = []
    for u in np.arange(-par.hs_rear_m, par.hs_front_m + 1e-9, step):
        pts += [(u, -par.hs_half_m), (u, par.hs_half_m)]
    for v in np.arange(-par.hs_half_m, par.hs_half_m + 1e-9, step):
        pts += [(-par.hs_rear_m, v), (par.hs_front_m, v)]
    c, s = math.cos(yaw_rad), math.sin(yaw_rad)
    return [(x + u * c - v * s, y + u * s + v * c) for u, v in pts]


class _Ctx:
    """Everything the steps share."""
    pass


def _prepare(map_yaml, labels_src, par):
    c = _Ctx()
    c.par = par
    c.mp = MapData(map_yaml)
    lab = load_labels(labels_src, c.mp.revision)
    c.labels = lab['labels']
    rects = table_rect_list(c.labels, par)
    flat = [r for rs in rects.values() for r in rs]
    mp = c.mp
    extra = rasterize_rects((mp.h, mp.w), mp, flat) if flat else np.zeros((mp.h, mp.w), bool)
    c.obst = mp.blocked | extra
    c.clr = np.where(extra, 0.0, clearance_map(c.obst, mp.res, cap_m=1.0))
    c.clr_cell = clearance_sampler(mp, c.clr)     # for the graph build (fast, cell grid)
    c.clr_at = exact_sampler(mp, c.obst)           # for every HardStop decision
    c.need = par.min_corridor_m / 2.0
    return c


def _blocked_at(c):
    def f(x, y):
        r, col = c.mp.cell_of(x, y)
        return (not c.mp.inside(r, col)) or bool(c.obst[r, col])
    return f


def _stop_for_standby(c, g, l, res):
    click = (l['x'], l['y'])
    best = None
    for e in g.edges:
        foot, _ = sg.foot_on_segment(click, g.nodes[e['a']], g.nodes[e['b']])
        d = math.hypot(foot[0] - click[0], foot[1] - click[1])
        if d <= c.par.snap_max_m and (best is None or d < best[0]):
            best = (d, e, foot)
    if best is None:
        res.excluded.append({'id': l['id'], 'reason': 'no corridor within %.1f m of the point'
                             % c.par.snap_max_m})
        return None
    _, e, foot = best
    nid = sg.nearest_node(g, foot, c.par.node_reuse_m) or sg.split_edge(g, e, foot, c.clr_cell)
    return {'node': nid}


def _axis_yaw(deg, tol=1.0):
    """East / north / west / south (within tol degrees) as -90, 0, 90, 180; otherwise None."""
    for k in (-180, -90, 0, 90, 180):
        if abs((deg - k + 180.0) % 360.0 - 180.0) <= tol:
            return 180.0 if k == -180 else float(k)
    return None


def _stop_for_pin(c, g, l, res, blocked_at):
    """The stop a person placed by hand: the corridor point nearest to it, facing the chosen way.
    Same checks as a generated stop (front gap, facing the target). When they fail the label is
    left out with the reason; it is never swapped for a generated stop."""
    par = c.par
    pin = l['stop']
    want = (pin['x'], pin['y'])
    yaw = None
    if pin['yaw_deg'] is not None:
        yaw = _axis_yaw(pin['yaw_deg'])
        if yaw is None:
            res.excluded.append({'id': l['id'], 'reason': 'pinned direction %.0f deg is not east, '
                                 'north, west or south' % pin['yaw_deg']})
            return None
    best = None
    for e in g.edges:
        foot, _ = sg.foot_on_segment(want, g.nodes[e['a']], g.nodes[e['b']])
        d = math.hypot(foot[0] - want[0], foot[1] - want[1])
        if best is None or d < best[0]:
            best = (d, e, foot)
    if best is None or best[0] > par.snap_max_m:
        res.excluded.append({'id': l['id'], 'reason': 'pinned stop: no corridor within %.1f m'
                             % par.snap_max_m})
        return None
    _, e, foot = best
    near = sg.nearest_node(g, foot, par.node_reuse_m)
    pos = g.nodes[near] if near else foot
    if l['kind'] != 'standby':
        click = (l['x'], l['y'])
        if yaw is None:
            yaw = _snap_yaw(math.degrees(math.atan2(click[1] - pos[1], click[0] - pos[0])),
                            par.yaw_snap_deg)
        yr = math.radians(yaw)
        front = (pos[0] + par.front_m * math.cos(yr), pos[1] + par.front_m * math.sin(yr))
        gap = raycast(blocked_at, front, yr)
        if gap is None:
            res.excluded.append({'id': l['id'], 'reason': 'pinned stop: nothing in front of it '
                                 '(it faces %.0f deg)' % yaw})
            return None
        if not (par.gap_lo_m <= gap <= par.gap_hi_m):
            res.excluded.append({'id': l['id'], 'reason': 'pinned stop: front gap %.2f m is not '
                                 'within %.2f-%.2f m' % (gap, par.gap_lo_m, par.gap_hi_m)})
            return None
        hit = (front[0] + gap * math.cos(yr), front[1] + gap * math.sin(yr))
        if l['kind'] == 'table':
            top = l.get('top_size_m') or {'x': par.default_top_m, 'y': par.default_top_m}
            tol = math.hypot(top['x'], top['y']) / 2 + 0.15
        else:
            tol = 0.7
        if math.hypot(hit[0] - click[0], hit[1] - click[1]) > tol:
            res.excluded.append({'id': l['id'], 'reason': 'pinned stop does not face the target '
                                 '(what is in front is %.2f m from it)'
                                 % math.hypot(hit[0] - click[0], hit[1] - click[1])})
            return None
    nid = near or sg.split_edge(g, e, foot, c.clr_cell)
    return {'node': nid, 'pin_yaw': yaw, 'pinned': True}


def _stop_for_target(c, g, l, res, blocked_at):
    """Best corridor point that faces the labelled table / counter."""
    par = c.par
    click = (l['x'], l['y'])
    if l['kind'] == 'table':
        top = l.get('top_size_m') or {'x': par.default_top_m, 'y': par.default_top_m}
        tol = math.hypot(top['x'], top['y']) / 2 + 0.15
    else:
        tol = 0.7
    scored = []
    good_room = par.corner_radius_m + par.turn_warn_margin_m
    for e in g.edges:
        pa, pb = g.nodes[e['a']], g.nodes[e['b']]
        foot, _ = sg.foot_on_segment(click, pa, pb)
        n = max(1, int(e['length_m'] / 0.05))
        spots = [(pa[0] + (pb[0] - pa[0]) * i / n, pa[1] + (pb[1] - pa[1]) * i / n)
                 for i in range(n + 1)] + [foot]
        for pos in spots:
            if math.hypot(click[0] - pos[0], click[1] - pos[1]) < 0.3:
                continue
            yaw = _snap_yaw(math.degrees(math.atan2(click[1] - pos[1], click[0] - pos[0])),
                            par.yaw_snap_deg)
            yr = math.radians(yaw)
            front = (pos[0] + par.front_m * math.cos(yr), pos[1] + par.front_m * math.sin(yr))
            gap = raycast(blocked_at, front, yr)
            if gap is None or not (par.gap_lo_m <= gap <= par.gap_hi_m):
                continue
            hit = (front[0] + gap * math.cos(yr), front[1] + gap * math.sin(yr))
            if math.hypot(hit[0] - click[0], hit[1] - click[1]) > tol:
                continue
            room = c.clr_at(*pos)
            d_foot = math.hypot(pos[0] - foot[0], pos[1] - foot[1])
            # enough room to turn -> the closest gap wins; otherwise the roomiest spot
            axis = 0 if yaw in (-180.0, -90.0, 0.0, 90.0, 180.0) else 1
            key = ((axis, 0, round(gap / 0.05), d_foot) if room >= good_room
                   else (axis, 1, -round(room / 0.02), gap))
            scored.append((key, e, pos))   # the edge itself: ids of edges split earlier are still ''
    if not scored:
        res.excluded.append({'id': l['id'], 'reason': 'no corridor point faces it with a front gap '
                             'of %.2f-%.2f m' % (par.gap_lo_m, par.gap_hi_m)})
        return None
    scored.sort(key=lambda t: t[0])
    _, e, pos = scored[0]
    nid = sg.nearest_node(g, pos, 1e-6) or sg.split_edge(g, e, pos, c.clr_cell)
    return {'node': nid}


def _approach_extent(c, g, nid, yaw_deg, gap_goal, blocked_at):
    """How far a straight branch can run from node `nid` along the (axis) heading `yaw_deg` so the
    robot front ends `gap_goal` from the table. Returns (length_m, front_gap_now, reason or None)."""
    par = c.par
    x, y = g.nodes[nid]
    yr = math.radians(yaw_deg)
    dx, dy = round(math.cos(yr)), round(math.sin(yr))
    gap_now = raycast(blocked_at, (x + par.front_m * dx, y + par.front_m * dy), yr)
    if gap_now is None:
        return 0.0, None, 'no table ahead'
    t_max = gap_now - gap_goal
    if t_max <= 0.0:
        return 0.0, gap_now, 'already %.0f cm from the table' % (gap_now * 100)
    # other corridors: the dock keeps the corridor half width from them (own edges excepted)
    others = [(g.nodes[e['a']], g.nodes[e['b']]) for e in g.edges if nid not in (e['a'], e['b'])]
    t, last_ok, why = 0.0, 0.0, None
    while t < t_max - 1e-9:
        t = min(t + 0.025, t_max)
        px, py = x + dx * t, y + dy * t
        if c.clr_at(px, py) < c.need - 0.02:
            why = 'not enough room beside the branch (%.2f m)' % c.clr_at(px, py)
            break
        if any(math.hypot(px - sg.foot_on_segment((px, py), a, b)[0][0],
                          py - sg.foot_on_segment((px, py), a, b)[0][1]) < c.need
               for a, b in others):
            why = 'another corridor is in the way'
            break
        last_ok = t
    if why is None and c.clr_at(x + dx * last_ok, y + dy * last_ok) - par.corner_radius_m \
            < par.turn_fail_margin_m:
        return 0.0, gap_now, 'no room to turn at the end of the branch'
    return last_ok, gap_now, why


def _add_approach_branches(c, g, stops, res, blocked_at):
    """For every table stop, a straight spur from the corridor node towards the table.
    The robot turns at the corridor node, drives the spur, serves and turns round at the dock.
    No branch when it would be shorter than min_branch_m: the guest reaches over."""
    par = c.par
    # the dock needs room to turn in place: at least corner radius + the WARN margin from the table
    gap_goal = max(par.serve_gap_m, par.corner_radius_m + par.turn_warn_margin_m - par.front_m)
    docks = {}
    for l in c.labels:
        if l['kind'] != 'table' or l['id'] not in stops:
            continue
        nid = stops[l['id']]['node']
        x, y = g.nodes[nid]
        info = {'node': nid, 'branch_m': 0.0, 'gap_goal_m': round(gap_goal, 3)}
        res.approach[l['id']] = info
        yaw = _snap_yaw(math.degrees(math.atan2(l['y'] - y, l['x'] - x)), par.yaw_snap_deg)
        if yaw not in (-90.0, 0.0, 90.0, 180.0):
            info['reason'] = 'the table is not straight ahead of the corridor node'
            continue
        length, gap_now, why = _approach_extent(c, g, nid, yaw, gap_goal, blocked_at)
        info['gap_now_m'] = None if gap_now is None else round(gap_now, 3)
        if length < par.min_branch_m:
            info['reason'] = why or ('a branch would be only %.0f cm (under %.0f cm)'
                                     % (length * 100, par.min_branch_m * 100))
            continue
        yr = math.radians(yaw)
        dock = (x + round(math.cos(yr)) * length, y + round(math.sin(yr)) * length)
        again = _snap_yaw(math.degrees(math.atan2(l['y'] - dock[1], l['x'] - dock[0])),
                          par.yaw_snap_deg)
        if again != yaw:
            info['reason'] = 'the table would no longer be straight ahead at the dock'
            continue
        if any(math.hypot(dock[0] - v[0], dock[1] - v[1]) < par.node_reuse_m
               for v in list(g.nodes.values()) + list(docks.values())):
            info['reason'] = 'another node is already at the end of the branch'
            continue
        did = sg.new_node_id(g)
        g.nodes[did] = dock
        edge = {'id': '', 'a': nid, 'b': did, 'length_m': 0.0, 'angle_deg': 0.0,
                'min_clearance_m': None, 'branch': True}
        g.edges.append(edge)
        sg.refresh_edge(g, edge, c.clr_at)
        docks[l['id']] = dock
        stops[l['id']] = {**stops[l['id']], 'node': did}
        info.update({'node': did, 'from_node': nid, 'branch_m': round(length, 3),
                     'gap_m': round(gap_now - length, 3)})


def _incident_yaw(g, nid, par):
    best = None
    for e in g.edges:
        if nid in (e['a'], e['b']):
            other = e['b'] if e['a'] == nid else e['a']
            if best is None or e['length_m'] > best[0]:
                best = (e['length_m'], other)
    if best is None:
        return 0.0
    p, q = g.nodes[nid], g.nodes[best[1]]
    return _snap_yaw(math.degrees(math.atan2(q[1] - p[1], q[0] - p[0])), par.yaw_snap_deg)


def generate(map_yaml, labels_src, par=None, adjustments=None):
    par = par or Params()
    c = _prepare(map_yaml, labels_src, par)
    res = Result()
    res.params, res.map_revision, res.labels = par, c.mp.revision, c.labels
    blocked_at = _blocked_at(c)
    g, _, _ = generate_graph(c.mp, c.clr, c.need, par.spur_m, par.dp_tol_m)
    if not g.edges:
        raise PipelineError('no straight corridor found: check the map and the corridor width')

    # tables and counters decide where the corridor points are; standby goes last
    stops = {}
    order = [x for x in c.labels if x['kind'] != 'standby'] + \
            [x for x in c.labels if x['kind'] == 'standby']
    # stops a person placed by hand go last: they join the nodes the others made and change nothing
    # about where the generated stops are
    order = [x for x in order if not x.get('stop')] + [x for x in order if x.get('stop')]
    for l in order:
        s = (_stop_for_pin(c, g, l, res, blocked_at) if l.get('stop')
             else _stop_for_standby(c, g, l, res) if l['kind'] == 'standby'
             else _stop_for_target(c, g, l, res, blocked_at))
        if s:
            stops[l['id']] = s

    # prune what leads nowhere, then drop nodes the HardStop could not turn in
    protected = {s['node'] for s in stops.values()}
    sg.prune(g, protected, c.clr_cell)
    removed, removed_xy = [], {}
    for _ in range(20):
        bad = {k for k, (x, y) in g.nodes.items()
               if c.clr_at(x, y) - par.corner_radius_m < par.turn_fail_margin_m}
        if not bad:
            break
        removed += sorted(bad)
        removed_xy.update({k: g.nodes[k] for k in bad})
        sg.remove_nodes(g, bad)
        protected -= bad
        sg.prune(g, protected, c.clr_cell)
    if removed:
        res.warnings.append('removed %d node(s) that fail the HardStop turn check, with their '
                            'corridors: %s' % (len(removed), ', '.join(
                                '(%.2f, %.2f)' % removed_xy[k] for k in removed)))
    for lid, s in list(stops.items()):
        if s['node'] not in g.nodes:
            res.excluded.append({'id': lid, 'reason': 'its stop node fails the HardStop turn '
                                 'check (needs %.3f m + %.2f m of room)'
                                 % (par.corner_radius_m, par.turn_fail_margin_m)})
            del stops[lid]

    # everything must be reachable from the first standby point
    first = next((stops[l['id']]['node'] for l in c.labels
                  if l['kind'] == 'standby' and l['id'] in stops), None)
    if first is None:
        raise PipelineError('no standby point could be placed on a corridor: '
                            + '; '.join('%s: %s' % (x['id'], x['reason']) for x in res.excluded))
    home = next(comp for comp in sg.components(g) if first in comp)
    for lid, s in list(stops.items()):
        if s['node'] not in home:
            res.excluded.append({'id': lid, 'reason': 'not connected to the first standby point'})
            del stops[lid]
    sg.prune(g, {s['node'] for s in stops.values()}, c.clr_cell)
    ident = sg.renumber(g)
    for s in stops.values():
        s['node'] = ident[s['node']]
    res.adjust_info = None
    if par.approach:
        _add_approach_branches(c, g, stops, res, blocked_at)
        if res.approach and any(v['branch_m'] for v in res.approach.values()):
            ident = sg.renumber(g)
            for s in stops.values():
                s['node'] = ident[s['node']]
            for v in res.approach.values():
                for k in ('node', 'from_node'):
                    if k in v:
                        v[k] = ident[v[k]]
    if adjustments:
        from . import adjust as adj
        a = adj.load(adjustments)
        if adj.approved(a):
            res.adjust_info = adj.apply_to_graph(g, a, c, stops, res)
            for r in res.adjust_info['rejected']:
                res.warnings.append('adjustment %s not applied: %s' % (r['id'], r['reason']))
    res.graph = g
    _poses(c, g, stops, res)
    _checks(c, g, res)
    _zones(g, res)
    return res


def _poses(c, g, stops, res):
    par = c.par
    taken = {l['id'] for l in c.labels}
    n_standby = 0
    for l in c.labels:
        s = stops.get(l['id'])
        if not s:
            continue
        x, y = g.nodes[s['node']]
        if s.get('pin_yaw') is not None:
            yaw = s['pin_yaw']                       # the person chose the direction
        elif l['kind'] == 'standby':
            yaw = _incident_yaw(g, s['node'], par)
        else:   # face the clicked target again from the final node position
            yaw = _snap_yaw(math.degrees(math.atan2(l['y'] - y, l['x'] - x)), par.yaw_snap_deg)
        aliases = list(l['aliases'])
        if not aliases:
            if l['kind'] == 'standby':
                if l['id'].startswith('standby_'):
                    aliases.append('wait_' + l['id'].split('_', 1)[1])
                if n_standby == 0:
                    aliases.append('home')
            elif l['kind'] == 'pickup':
                aliases.append('desk')
            aliases = [a for a in aliases if a not in taken]
        taken.update(aliases)
        n_standby += l['kind'] == 'standby'
        margin = min(c.clr_at(px, py)
                     for px, py in hardstop_points(x, y, math.radians(yaw), par))
        p = {'node': s['node'], 'x': x, 'y': y, 'yaw_deg': yaw, 'aliases': aliases,
             'pinned': bool(s.get('pinned')),
             'kind': {'table': 'table_service', 'pickup': 'pickup', 'standby': 'standby'}[l['kind']],
             'hardstop_margin_m': margin,
             'level': 'FAIL' if margin <= 0.02 else ('WARN' if margin < 0.05 else 'OK')}
        if l['kind'] == 'table':
            p['faces'] = {'table': l['id']}
            top = l.get('top_size_m') or {'x': par.default_top_m, 'y': par.default_top_m}
            t = {'center': {'x': round(l['x'], 3), 'y': round(l['y'], 3)},
                 'top_size_m': dict(top)}
            if l.get('chairs'):
                t['chairs'] = l['chairs']
            res.entities['tables'][l['id']] = t
        elif l['kind'] == 'pickup':
            p['faces'] = {'fixture': l['id']}
            res.entities['fixtures'][l['id']] = {
                'kind': 'box', 'center': {'x': round(l['x'], 3), 'y': round(l['y'], 3)},
                'size_m': {'x': 0.5, 'y': 0.5}, 'estimated': True}
        res.poses[l['id']] = p
    for l in c.labels:
        if l['id'] not in res.poses and not any(x['id'] == l['id'] for x in res.excluded):
            res.excluded.append({'id': l['id'], 'reason': 'its node was pruned'})


def _checks(c, g, res):
    par = c.par
    for nid, (x, y) in g.nodes.items():
        cl = c.clr_at(x, y)
        m = cl - par.corner_radius_m
        res.node_checks[nid] = {'clearance_m': cl, 'margin_m': m,
                                'level': _level(m, par.turn_fail_margin_m, par.turn_warn_margin_m)}
    for e in g.edges:
        cm = e['min_clearance_m'] if e['min_clearance_m'] is not None else 0.0
        chk = {'clearance_m': cm,
               'level': 'FAIL' if cm < c.need - 0.03 else
               ('WARN' if cm < c.need + par.edge_warn_margin_m else 'OK')}
        off_axis = abs(((e['angle_deg'] + 45) % 90) - 45)
        if off_axis > par.yaw_snap_deg:
            if chk['level'] == 'OK':
                chk['level'] = 'WARN'
            chk['note'] = 'slanted (%.0f deg)' % e['angle_deg']
            res.warnings.append('edge %s is slanted (%.0f deg): check it against the walls'
                                % (e['id'], e['angle_deg']))
        res.edge_checks[e['id']] = chk


def _zones(g, res):
    """Temporary zones until the Fleet part delivers the real ones."""
    pad = res.params.zone_pad_m
    for name, p in res.poses.items():
        if p['kind'] in ('standby', 'pickup'):
            h = 0.45
            res.zones.append({'id': '%s_zone' % name, 'rect': {
                'x_min': p['x'] - h, 'x_max': p['x'] + h,
                'y_min': p['y'] - h, 'y_max': p['y'] + h}})
    for e in g.edges:
        (ax, ay), (bx, by) = g.nodes[e['a']], g.nodes[e['b']]
        e['zone'] = 'corridor_%s' % e['id']
        res.zones.append({'id': e['zone'], 'rect': {
            'x_min': min(ax, bx) - pad, 'x_max': max(ax, bx) + pad,
            'y_min': min(ay, by) - pad, 'y_max': max(ay, by) + pad}})


# ------------------------------------------------------------------------- output
HEADER = ('# GENERATED by tools/site_builder (authoring: generated), status: provisional.\n'
          '# Check with: python3 scripts/validate_site_config.py --config-dir <this dir>\n')


def _r(v, n=3):
    return round(float(v), n)


def _round_all(d):
    return {k: (_r(v) if isinstance(v, float) else v) for k, v in d.items()}


def to_waypoints(res):
    g = res.graph
    poses = {}
    for name, p in res.poses.items():
        o = {'node': p['node'], 'yaw_deg': _r(p['yaw_deg'], 1), 'kind': p['kind']}
        if p.get('faces'):
            o['faces'] = p['faces']
        if p['aliases']:
            o['aliases'] = p['aliases']
        poses[name] = o
    return {'schema_version': 1, 'status': 'provisional', 'authoring': 'generated',
            'frame_id': 'map', 'map_revision': res.map_revision,
            'nodes': {k: {'x': _r(v[0]), 'y': _r(v[1])} for k, v in g.nodes.items()},
            'edges': [{'id': e['id'], 'from': e['a'], 'to': e['b'], 'zone': e['zone']}
                      for e in g.edges],
            'poses': poses}


def to_zones(res):
    return {'schema_version': 1, 'status': 'provisional', 'owner': 'fleet',
            'authoring': 'generated', 'frame_id': 'map',
            'zones': [{'id': z['id'], 'rect': _round_all(z['rect'])} for z in res.zones]}


def to_entities(res):
    return {'schema_version': 1, 'status': 'provisional', 'authoring': 'generated',
            'robots': {'robot_01': {}}, 'tables': res.entities['tables'],
            'fixtures': res.entities['fixtures']}


def to_preview(res):
    g = res.graph
    return {
        'schema_version': 1, 'map_revision': res.map_revision, 'params': res.params.as_dict(),
        'nodes': {k: {'x': _r(v[0]), 'y': _r(v[1]), **_round_all(res.node_checks[k])}
                  for k, v in g.nodes.items()},
        'edges': [{'id': e['id'], 'from': e['a'], 'to': e['b'], 'length_m': _r(e['length_m']),
                   'angle_deg': _r(e['angle_deg'], 1), **_round_all(res.edge_checks[e['id']])}
                  for e in g.edges],
        'poses': {n: _round_all(p) for n, p in res.poses.items()},
        'zones': [{'id': z['id'], 'rect': _round_all(z['rect'])} for z in res.zones],
        'excluded': res.excluded, 'warnings': res.warnings, 'adjustments': res.adjust_info,
        'approach': res.approach,
        'summary': {k: res.count(k) for k in ('OK', 'WARN', 'FAIL')},
    }


def write_outputs(res, out_dir):
    os.makedirs(out_dir, exist_ok=True)
    for name, data in (('waypoints.yaml', to_waypoints(res)),
                       ('entities.yaml', to_entities(res)), ('zones.yaml', to_zones(res))):
        with open(os.path.join(out_dir, name), 'w', encoding='utf-8') as f:
            f.write(HEADER)
            yaml.safe_dump(data, f, sort_keys=False, allow_unicode=True, default_flow_style=None)
    with open(os.path.join(out_dir, 'preview.json'), 'w', encoding='utf-8') as f:
        json.dump(to_preview(res), f, indent=1, ensure_ascii=False)
    ap = os.path.join(out_dir, 'adjustments_applied.json')
    if res.adjust_info:
        with open(ap, 'w', encoding='utf-8') as f:
            json.dump(res.adjust_info, f, indent=1, ensure_ascii=False)
    elif os.path.isfile(ap):
        os.remove(ap)          # an unadjusted run must not leave an old record behind
    return out_dir
