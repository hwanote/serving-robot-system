"""A drive plan: one continuous tour over the route graph, every corridor once in each
direction, starting and ending at the home node. No teleporting between steps.

An Euler circuit of the graph with each edge replaced by two opposite arcs always exists
for a connected graph (every node has as many arcs in as out).
"""
import hashlib
import json
import math
from collections import defaultdict


class TourError(ValueError):
    pass


def graph_of(waypoints):
    """({node: (x, y)}, [(edge id, a, b)]) from a waypoints.yaml dict."""
    nodes = {str(k): (float(v['x']), float(v['y'])) for k, v in (waypoints.get('nodes') or {}).items()}
    edges = [(str(e['id']), str(e['from']), str(e['to'])) for e in waypoints.get('edges') or []]
    for eid, a, b in edges:
        if a not in nodes or b not in nodes:
            raise TourError("edge '%s' refers to an unknown node" % eid)
    return nodes, edges


def _heading(p, q):
    return math.atan2(q[1] - p[1], q[0] - p[0])


def euler_tour(nodes, edges, start):
    """[(edge id, from node, to node)] covering every edge in both directions."""
    if start not in nodes:
        raise TourError("unknown start node '%s'" % start)
    arcs = defaultdict(list)           # node -> [(to, edge id)] sorted, consumed as we go
    for eid, a, b in sorted(edges):
        arcs[a].append((b, eid))
        arcs[b].append((a, eid))
    for k in arcs:
        arcs[k].sort()
    total = sum(len(v) for v in arcs.values())
    # Hierholzer; prefer not to turn straight back on the arc we arrived by
    stack = [(start, None)]            # (node, arc used to get here (from, to, eid))
    circuit = []
    while stack:
        v, came = stack[-1]
        options = arcs[v]
        if not options:
            stack.pop()
            if came is not None:
                circuit.append(came)
            continue
        pick = 0
        if came is not None and len(options) > 1:
            for i, (to, eid) in enumerate(options):
                if not (eid == came[2] and to == came[0]):
                    pick = i
                    break
        to, eid = options.pop(pick)
        stack.append((to, (v, to, eid)))
    circuit.reverse()
    if len(circuit) != total:
        seen = {e for _, _, e in circuit}
        raise TourError('not every corridor can be reached from %s (missing: %s)'
                        % (start, ', '.join(sorted({e for e, _, _ in edges} - seen))))
    return [(eid, a, b) for a, b, eid in circuit]


def plan(waypoints, start, speed_mps=0.25, omega_rps=0.8):
    """The tour with lengths and expected times. The first step starts facing its direction."""
    nodes, edges = graph_of(waypoints)
    steps, prev_dir = [], None
    for i, (eid, a, b) in enumerate(euler_tour(nodes, edges, start), 1):
        L = math.hypot(nodes[b][0] - nodes[a][0], nodes[b][1] - nodes[a][1])
        d = _heading(nodes[a], nodes[b])
        turn = 0.0 if prev_dir is None else abs(math.atan2(math.sin(d - prev_dir), math.cos(d - prev_dir)))
        steps.append({'i': i, 'edge': eid, 'from': a, 'to': b, 'length_m': round(L, 3),
                      'heading_deg': round(math.degrees(d), 1), 'turn_deg': round(math.degrees(turn), 1),
                      'expected_s': round(L / speed_mps + turn / omega_rps + 2.0, 1)})
        prev_dir = d
    tour = {'schema_version': 1, 'start': start, 'steps': steps}
    tour['hash'] = tour_hash(tour)
    return tour


def tour_hash(tour):
    core = [(s['edge'], s['from'], s['to']) for s in tour['steps']]
    return hashlib.sha1(json.dumps([tour['start'], core]).encode()).hexdigest()[:12]
