"""Editing a Graph: split an edge to put a stop on it, prune, renumber."""
import math
from collections import defaultdict

import numpy as np

from .graph import Graph, _drop_passthrough, _edge_angle


def new_node_id(g):
    nums = [int(k[1:]) for k in g.nodes if k[0] == 'n' and k[1:].isdigit()]
    return 'n%02d' % (max(nums, default=0) + 1)


def foot_on_segment(p, a, b):
    ax, ay, bx, by = a[0], a[1], b[0], b[1]
    dx, dy = bx - ax, by - ay
    den = dx * dx + dy * dy
    t = 0.0 if den == 0 else max(0.0, min(1.0, ((p[0] - ax) * dx + (p[1] - ay) * dy) / den))
    return (ax + t * dx, ay + t * dy), t


def refresh_edge(g, e, clr_at=None):
    p, q = g.nodes[e['a']], g.nodes[e['b']]
    e['length_m'] = math.hypot(q[0] - p[0], q[1] - p[1])
    e['angle_deg'] = _edge_angle(p, q)
    if clr_at is not None:
        n = max(2, int(e['length_m'] / 0.025) + 1)
        e['min_clearance_m'] = min(
            clr_at(p[0] + (q[0] - p[0]) * t, p[1] + (q[1] - p[1]) * t)
            for t in np.linspace(0.0, 1.0, n))


def nearest_node(g, xy, within):
    best, bd = None, within
    for k, v in g.nodes.items():
        d = math.hypot(v[0] - xy[0], v[1] - xy[1])
        if d <= bd:
            best, bd = k, d
    return best


def split_edge(g, edge, point, clr_at=None):
    """Put a new node on `edge` at `point`; returns its id."""
    nid = new_node_id(g)
    g.nodes[nid] = point
    old_b = edge['b']
    edge['b'] = nid
    new = {'id': '', 'a': nid, 'b': old_b, 'length_m': 0.0, 'angle_deg': 0.0,
           'min_clearance_m': None}
    g.edges.append(new)
    refresh_edge(g, edge, clr_at)
    refresh_edge(g, new, clr_at)
    return nid


def degrees(g):
    d = defaultdict(int)
    for e in g.edges:
        d[e['a']] += 1
        d[e['b']] += 1
    return d


def components(g):
    adj = defaultdict(set)
    for k in g.nodes:
        adj[k]
    for e in g.edges:
        adj[e['a']].add(e['b'])
        adj[e['b']].add(e['a'])
    seen, comps = set(), []
    for k in sorted(adj):
        if k in seen:
            continue
        stack, comp = [k], set()
        seen.add(k)
        while stack:
            u = stack.pop()
            comp.add(u)
            for v in adj[u]:
                if v not in seen:
                    seen.add(v)
                    stack.append(v)
        comps.append(comp)
    return comps


def prune(g, protected, clr_at=None):
    """Drop what leads nowhere: components without a protected node, dead-end branches
    without one, and protected-free nodes that are only a straight pass-through."""
    for comp in components(g):
        if not (comp & set(protected)):
            g.edges = [e for e in g.edges if e['a'] not in comp]
            for k in comp:
                g.nodes.pop(k, None)
    while True:
        deg = degrees(g)
        leaf = next((k for k in sorted(g.nodes) if deg.get(k, 0) == 1 and k not in protected), None)
        if leaf is None:
            break
        g.edges = [e for e in g.edges if leaf not in (e['a'], e['b'])]
        del g.nodes[leaf]
    for k in [k for k in g.nodes if degrees(g).get(k, 0) == 0 and k not in protected]:
        del g.nodes[k]
    pts = dict(g.nodes)
    edges = {(min(e['a'], e['b']), max(e['a'], e['b'])) for e in g.edges}
    pts, edges = _drop_passthrough(pts, edges, keep=set(protected))
    g.nodes = pts
    g.edges = [{'id': '', 'a': a, 'b': b, 'length_m': 0.0, 'angle_deg': 0.0,
                'min_clearance_m': None} for a, b in sorted(edges)]
    for e in g.edges:
        refresh_edge(g, e, clr_at)


def renumber(g, extra_ids=()):
    """Ids n01.. in a fixed order (x, then y); edge ids e01.. -> returns {old: new}."""
    order = sorted(g.nodes, key=lambda k: (round(g.nodes[k][0], 3), round(g.nodes[k][1], 3)))
    ident = {k: 'n%02d' % (i + 1) for i, k in enumerate(order)}
    g.nodes = {ident[k]: g.nodes[k] for k in order}
    for e in g.edges:
        e['a'], e['b'] = sorted((ident[e['a']], ident[e['b']]))
    g.edges.sort(key=lambda e: (e['a'], e['b']))
    for i, e in enumerate(g.edges):
        e['id'] = 'e%02d' % (i + 1)
    return ident


def remove_nodes(g, bad):
    g.edges = [e for e in g.edges if e['a'] not in bad and e['b'] not in bad]
    for k in bad:
        g.nodes.pop(k, None)
