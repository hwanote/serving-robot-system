"""Measured drive results -> adjustment proposals -> deterministic application.

Nothing here changes a file by itself. propose() writes proposals (status 'proposed'); a person sets
the status to 'approved' (the screen in C-4 does that); generate(..., adjustments=...) applies only
approved ones. Proposals always refer to ONE base: the graph of an unadjusted generation, named by
base_hash. If the labels or the map change, the hash changes and old proposals are refused.

Kinds
  shift_line   move a whole straight chain of corridors sideways (never slants a corridor), at most
               MAX_SHIFT_M, towards the side with more room (from the LiDAR side distances)
  remove_edge  a corridor that failed in BOTH directions; `loses` lists destinations that become
               unreachable
  review_edge  something a person must look at (failed in one direction, or cannot be shifted
               without slanting); never applied
"""
import hashlib
import json
import math
import time

from .verdict import level

VERDICT_KO = {'COLLISION': '충돌', 'HARDSTOP': 'HardStop', 'BLOCKED': '막힘', 'OFF_RAIL': '경로 이탈',
              'NAV_FAILED': '주행 실패', 'TIMEOUT': '시간 초과', 'FAILED': '실패'}
MAX_SHIFT_M = 0.15
MIN_SHIFT_M = 0.05
MIN_GAIN_M = 0.03       # the tightest section must gain at least this much room
RANK = {'OK': 0, 'WARN': 1, 'FAIL': 2}
AXIS_TOL = 1e-6


class AdjustError(ValueError):
    pass


def _r3(v):
    return round(float(v), 3)


def hash_graph(nodes, edges):
    """nodes {id: (x, y)}, edges [(id, a, b)] -> short hash (same for a graph and its waypoints.yaml)."""
    body = json.dumps([sorted((k, _r3(v[0]), _r3(v[1])) for k, v in nodes.items()),
                       sorted(tuple(e) for e in edges)], sort_keys=True)
    return hashlib.sha1(body.encode()).hexdigest()[:12]


def hash_waypoints(wp):
    return hash_graph({k: (v['x'], v['y']) for k, v in wp['nodes'].items()},
                      [(e['id'], e['from'], e['to']) for e in wp['edges']])


def hash_of_graph(g):
    return hash_graph(g.nodes, [(e['id'], e['a'], e['b']) for e in g.edges])


def _axis(p, q):
    if abs(p[0] - q[0]) < AXIS_TOL:
        return 'v'
    if abs(p[1] - q[1]) < AXIS_TOL:
        return 'h'
    return None


def chain_of(nodes, edges, eid):
    """Nodes and edge ids of the maximal collinear axis-aligned chain through edge eid."""
    e0 = next(e for e in edges if e['id'] == eid)
    ax = _axis(nodes[e0['from']], nodes[e0['to']])
    if ax is None:
        return None
    key = (lambda p: p[1]) if ax == 'h' else (lambda p: p[0])
    ref = key(nodes[e0['from']])
    S, E, todo = set(), set(), [e0]
    while todo:
        e = todo.pop()
        if e['id'] in E:
            continue
        E.add(e['id'])
        S.update((e['from'], e['to']))
        for f in edges:
            if f['id'] in E or not ({f['from'], f['to']} & {e['from'], e['to']}):
                continue
            p, q = nodes[f['from']], nodes[f['to']]
            if _axis(p, q) == ax and abs(key(p) - ref) < 1e-3:
                todo.append(f)
    return ax, S, E


def shift_is_clean(nodes, edges, S, ax):
    """Every corridor leaving the chain must run across it (so it only gets longer/shorter)."""
    for e in edges:
        inside = (e['from'] in S) + (e['to'] in S)
        if inside == 1:
            other = _axis(nodes[e['from']], nodes[e['to']])
            if other is None or other == ax:      # slanted, or in line: it would change direction
                return False
    return True


def _reach(nodes, edges, start, skip=()):
    adj = {}
    for e in edges:
        if e['id'] in skip:
            continue
        adj.setdefault(e['from'], set()).add(e['to'])
        adj.setdefault(e['to'], set()).add(e['from'])
    seen, todo = {start}, [start]
    while todo:
        n = todo.pop()
        for m in adj.get(n, ()):
            if m not in seen:
                seen.add(m)
                todo.append(m)
    return seen


def _sides(rec_fwd, rec_rev):
    """Closest left/right distance in the frame of the edge direction a->b, both runs combined."""
    left, right = [], []
    if rec_fwd:
        left.append(rec_fwd['obs'].get('min_left_m')); right.append(rec_fwd['obs'].get('min_right_m'))
    if rec_rev:     # driven b->a: its left is our right
        left.append(rec_rev['obs'].get('min_right_m')); right.append(rec_rev['obs'].get('min_left_m'))
    l = [x for x in left if x is not None]
    r = [x for x in right if x is not None]
    return (min(l) if l else None), (min(r) if r else None)


def propose(waypoints, report, tour=None, applied_info=None):
    """-> adjustments dict. `report` is the test_report.json content."""
    nodes = {k: (v['x'], v['y']) for k, v in waypoints['nodes'].items()}
    edges = waypoints['edges']
    base = hash_waypoints(waypoints)
    steps = report.get('steps', {})
    by_edge = {}
    for rec in steps.values():
        by_edge.setdefault(rec['edge'], {})['fwd' if rec['from'] == next(
            e['from'] for e in edges if e['id'] == rec['edge']) else 'rev'] = rec
    out = {'schema_version': 1, 'base_hash': base, 'tour_hash': report.get('tour_hash'),
           'created': time.strftime('%Y-%m-%d %H:%M:%S'), 'proposals': [], 'notes': []}
    if applied_info:
        out['notes'].append('이 후보에는 이미 조정이 적용돼 있어요. 옮기기/빼기 제안은 다시 만들지 않아요 '
                            '(조정 전 원본에서 새로 시험해 주세요)')
    n = [0]

    def add(kind, **kw):
        n[0] += 1
        d = {'id': 'adj%02d' % n[0], 'kind': kind, 'status': 'proposed'}
        d.update(kw)
        out['proposals'].append(d)

    failed_both, failed_one = set(), set()
    for eid, d in sorted(by_edge.items()):
        bad = [k for k, r in d.items() if level(r['verdict'], r['flags']) == 'FAIL']
        if len(bad) == 2:
            failed_both.add(eid)
        elif len(bad) == 1:
            failed_one.add(eid)
    for eid in sorted(failed_one):
        d = by_edge[eid]
        r = next(r for r in d.values() if level(r['verdict'], r['flags']) == 'FAIL')
        add('review_edge', edge=eid, reason='한쪽 방향만 실패했어요 (%s): 그 자리에서 로봇을 직접 확인해 주세요'
            % VERDICT_KO.get(r['verdict'], r['verdict']))
    if not applied_info:
        poses = waypoints.get('poses') or {}
        start = tour['start'] if tour else next(
            (p['node'] for p in poses.values() if p.get('kind') == 'standby'), None)
        for eid in sorted(failed_both):
            lost = []
            if start in nodes:
                ok = _reach(nodes, edges, start, skip={eid})
                lost = sorted(nm for nm, p in poses.items() if p['node'] not in ok)
            add('remove_edge', edge=eid, loses=lost,
                reason='양쪽 방향 모두 실패했어요 (%s)' % ', '.join(
                    VERDICT_KO.get(r['verdict'], r['verdict']) for r in by_edge[eid].values()))
        done_chain = set()
        for eid in sorted(by_edge):
            if eid in failed_both or eid in failed_one or eid in done_chain:
                continue
            ch = chain_of(nodes, edges, eid)
            if ch is None:
                continue
            ax, S, E = ch
            rooms, ev = [], {}
            for ce in sorted(E):
                if ce in failed_both or ce in failed_one:
                    continue
                d = by_edge.get(ce)
                if not d:
                    continue
                e = next(x for x in edges if x['id'] == ce)
                p, q = nodes[e['from']], nodes[e['to']]
                l, r = _sides(d.get('fwd'), d.get('rev'))
                if l is None or r is None:
                    continue
                h = math.atan2(q[1] - p[1], q[0] - p[0])
                n_axis = (-math.sin(h)) if ax == 'v' else math.cos(h)   # left normal along +x (v) / +y (h)
                rooms.append((l, r) if n_axis > 0 else (r, l))           # (room on the + side, on the - side)
                ev[ce] = {'left_m': l, 'right_m': r}
            done_chain |= E
            if not rooms:
                continue
            # the shift that leaves the most room at the tightest section of the whole chain
            def worst(t):
                return min(min(rp - t, rm + t) for rp, rm in rooms)
            cand = [i / 100.0 for i in range(-int(MAX_SHIFT_M * 100), int(MAX_SHIFT_M * 100) + 1)]
            avg = max(cand, key=lambda t: (round(worst(t), 4), -abs(t)))
            if abs(avg) < MIN_SHIFT_M - 1e-9 or worst(avg) - worst(0.0) < MIN_GAIN_M:
                continue
            if any(ce in failed_both for ce in E) or not shift_is_clean(nodes, edges, S, ax):
                add('review_edge', edge=sorted(E)[0],
                    reason='옆으로 %.2f m 옮기면 좋지만, 옮기면 이웃한 구간이 비스듬해져서 자동 제안은 하지 않아요' % abs(avg))
                continue
            d = [round(avg, 3), 0.0] if ax == 'v' else [0.0, round(avg, 3)]
            add('shift_line', nodes=sorted(S), edges=sorted(E), d=d, evidence=ev,
                reason='라이다로 잰 좌우 여유가 달라요: %.0f cm 옮기면 가장 좁은 곳 여유가 %.0f cm 늘어요'
                       % (abs(avg) * 100, (worst(avg) - worst(0.0)) * 100))
    return out


def load(path_or_dict):
    if isinstance(path_or_dict, dict):
        return path_or_dict
    with open(path_or_dict, 'r', encoding='utf-8') as f:
        return json.load(f)


def _node_level(c, par, xy):
    """Same rule as pipeline._checks: turning room = clearance - corner radius."""
    m = c.clr_at(*xy) - par.corner_radius_m
    return 'FAIL' if m < par.turn_fail_margin_m else ('WARN' if m < par.turn_warn_margin_m else 'OK')


def _edge_level(c, par, e):
    """Same rule as pipeline._checks for the corridor width (the slant rule does not change when a line is shifted)."""
    cm = e['min_clearance_m'] if e['min_clearance_m'] is not None else 0.0
    return 'FAIL' if cm < c.need - 0.03 else ('WARN' if cm < c.need + par.edge_warn_margin_m else 'OK')


def approved(adj):
    return [p for p in adj.get('proposals', []) if p.get('status') == 'approved']


def apply_to_graph(g, adj, c, stops, res):
    """Applies approved proposals to the graph built by the pipeline (after its first renumber).
    Moves nodes, removes edges, then prunes and renumbers again. Returns the info dict."""
    from . import sitegraph as sg
    par = c.par
    if adj.get('base_hash') != hash_of_graph(g):
        raise AdjustError('the adjustments were made for another graph (base %s, this graph %s): '
                          'the labels or the map changed. Run the drive test again.'
                          % (adj.get('base_hash'), hash_of_graph(g)))
    info = {'base_hash': adj['base_hash'], 'applied': [], 'rejected': []}
    todo = approved(adj)
    for p in [x for x in todo if x['kind'] == 'remove_edge']:
        before = len(g.edges)
        g.edges = [e for e in g.edges if e['id'] != p['edge']]
        if len(g.edges) == before:
            info['rejected'].append({'id': p['id'], 'reason': 'edge %s not found' % p['edge']})
        else:
            info['applied'].append(p['id'])
    for p in [x for x in todo if x['kind'] == 'shift_line']:
        S = set(p['nodes'])
        if not S <= set(g.nodes):
            info['rejected'].append({'id': p['id'], 'reason': 'a node of the chain is gone'})
            continue
        ed = [{'id': e['id'], 'from': e['a'], 'to': e['b']} for e in g.edges]
        ax = 'v' if abs(p['d'][0]) > abs(p['d'][1]) else 'h'
        if not shift_is_clean(g.nodes, ed, S, ax):
            info['rejected'].append({'id': p['id'], 'reason': 'would slant a neighbouring corridor'})
            continue
        old = {k: g.nodes[k] for k in S}
        touched = [e for e in g.edges if e['a'] in S or e['b'] in S]
        lvl_n = {k: _node_level(c, par, old[k]) for k in S}          # levels before the move
        lvl_e = {e['id']: _edge_level(c, par, e) for e in touched}
        for k in S:
            g.nodes[k] = (old[k][0] + p['d'][0], old[k][1] + p['d'][1])
        for e in touched:
            sg.refresh_edge(g, e, c.clr_at)
        bad = [e['id'] for e in touched if (e['min_clearance_m'] or 0.0) < c.need - 0.03]
        bad_n = [k for k in S if c.clr_at(*g.nodes[k]) - par.corner_radius_m < par.turn_fail_margin_m]
        # a move must not make anything worse than it was (OK -> WARN is a worse map even if nothing fails)
        worse = ['%s %.3f m -> %.3f m' % (k, old_m, new_m) for k, old_m, new_m in
                 [(k, c.clr_at(*old[k]) - par.corner_radius_m, c.clr_at(*g.nodes[k]) - par.corner_radius_m)
                  for k in sorted(S) if RANK[_node_level(c, par, g.nodes[k])] > RANK[lvl_n[k]]]]
        worse_e = [e['id'] for e in touched if RANK[_edge_level(c, par, e)] > RANK[lvl_e[e['id']]]]
        if bad or bad_n or worse or worse_e:
            for k in S:
                g.nodes[k] = old[k]
            for e in touched:
                sg.refresh_edge(g, e, c.clr_at)
            if bad or bad_n:
                reason = 'the new position leaves too little room (%s)' % ', '.join(bad + bad_n)
            else:
                reason = ('옮기면 여유가 모자라서 새로 주의가 생겨요: %s'
                          % ', '.join(['노드 ' + w + ' (회전 여유)' for w in worse] + ['구간 ' + i for i in worse_e]))
            info['rejected'].append({'id': p['id'], 'reason': reason})
            continue
        info['applied'].append(p['id'])
    # who is still reachable from the first standby point?
    first = next((s['node'] for l in c.labels if l['kind'] == 'standby'
                  for lid, s in stops.items() if lid == l['id']), None)
    comp = _reach(g.nodes, [{'id': e['id'], 'from': e['a'], 'to': e['b']} for e in g.edges], first)
    for lid, s in list(stops.items()):
        if s['node'] not in comp:
            res.excluded.append({'id': lid, 'reason': 'cut off by an approved adjustment (edge removed)'})
            del stops[lid]
    sg.prune(g, {s['node'] for s in stops.values()}, c.clr_cell)
    ident = sg.renumber(g)
    for s in stops.values():
        s['node'] = ident[s['node']]
    info['adjusted_hash'] = hash_of_graph(g)
    return info
