"""Centre line pixels -> a graph of straight corridors (nodes and edges).

Pipeline (all lengths in metres, map frame):
  1. pixel graph of the thin centre line (diagonal links dropped where a
     4-neighbour path exists, so a staircase is not mistaken for a junction)
  2. nodes = junctions (clusters of junction pixels) and dead ends; chains of
     pixels between them are polylines
  3. spur pruning: a dead end shorter than `spur_m` that hangs on a junction is
     a skeleton barb, not a corridor
  4. nodes of degree 2 are joined into one polyline, then Douglas-Peucker cuts
     every polyline into straight pieces (`dp_tol_m`); the cut points become nodes
  5. near-horizontal / near-vertical edges are made exactly horizontal /
     vertical (`snap_deg`): the two ends of such an edge get the same y / x
  5b. a slanted edge between a horizontal and a vertical corridor (a cut corner)
     is replaced by the point where the two lines meet, if that spot is drivable
  6. nodes closer than `merge_m` are merged, pass-through nodes (degree 2, no
     bend) are removed, every straight horizontal / vertical line is slid to the
     middle of its corridor, ids are assigned in a fixed order
"""
import math
from collections import defaultdict

import numpy as np

NB8 = [(-1, 0), (-1, 1), (0, 1), (1, 1), (1, 0), (1, -1), (0, -1), (-1, -1)]


class Graph:
    def __init__(self):
        self.nodes = {}     # id -> (x, y)
        self.edges = []     # dicts: id, a, b, length_m, angle_deg, min_clearance_m

    def to_dict(self):
        return {
            'nodes': {k: {'x': round(v[0], 3), 'y': round(v[1], 3)}
                      for k, v in self.nodes.items()},
            'edges': [{'id': e['id'], 'from': e['a'], 'to': e['b'],
                       'length_m': round(e['length_m'], 3),
                       'angle_deg': round(e['angle_deg'], 1),
                       'min_clearance_m': (None if e.get('min_clearance_m') is None
                                           else round(e['min_clearance_m'], 3))}
                      for e in self.edges],
        }


# ----------------------------------------------------------------- pixel graph
def _pixel_adjacency(skel):
    h, w = skel.shape
    rows, cols = np.nonzero(skel)
    pix = set(zip(rows.tolist(), cols.tolist()))
    adj = {p: set() for p in pix}
    for r, c in pix:
        for dr, dc in NB8:
            q = (r + dr, c + dc)
            if q not in pix:
                continue
            if dr != 0 and dc != 0 and ((r + dr, c) in pix or (r, c + dc) in pix):
                continue      # diagonal shortcut next to an orthogonal path
            adj[(r, c)].add(q)
    return adj


def _chains_from_pixels(skel, xy_of):
    """-> (nodes {id: (x, y)}, chains [{a, b, pts[(x, y)]}])."""
    adj = _pixel_adjacency(skel)
    deg = {p: len(n) for p, n in adj.items()}
    is_node = {p: deg[p] != 2 for p in adj}
    # clusters of junction pixels (deg >= 3 that touch each other)
    cluster = {}
    cid = 0
    for p in sorted(adj):
        if not is_node[p] or p in cluster:
            continue
        stack = [p]
        cluster[p] = cid
        while stack:
            u = stack.pop()
            for v in adj[u]:
                if is_node[v] and deg[v] >= 3 and deg[u] >= 3 and v not in cluster:
                    cluster[v] = cid
                    stack.append(v)
        cid += 1
    members = defaultdict(list)
    for p, k in cluster.items():
        members[k].append(p)
    nodes = {}
    for k, ps in members.items():
        xs = [xy_of(*p) for p in ps]
        nodes[k] = (sum(x for x, _ in xs) / len(xs), sum(y for _, y in xs) / len(xs))

    chains = []
    seen = set()

    def mark(u, v):
        seen.add((u, v))
        seen.add((v, u))

    for p in sorted(adj):
        if not is_node[p]:
            continue
        for q in sorted(adj[p]):
            if cluster[p] == cluster.get(q, -1) or (p, q) in seen:
                continue
            path = [p, q]
            mark(p, q)
            prev, cur = p, q
            while not is_node[cur]:
                nxt = [n for n in adj[cur] if n != prev]
                if not nxt:
                    break
                mark(cur, nxt[0])
                prev, cur = cur, nxt[0]
                path.append(cur)
            if cur not in cluster:       # defensive: the walk ended on a plain pixel
                continue
            pts = [xy_of(*u) for u in path]
            pts[0], pts[-1] = nodes[cluster[p]], nodes[cluster[cur]]
            chains.append({'a': cluster[p], 'b': cluster[cur], 'pts': pts})
    # closed loops: only degree 2 pixels, nothing walked yet
    for p in sorted(adj):
        if is_node[p] or any((p, q) in seen for q in adj[p]):
            continue
        k = max(nodes, default=-1) + 1
        nodes[k] = xy_of(*p)
        q = sorted(adj[p])[0]
        path = [p, q]
        mark(p, q)
        prev, cur = p, q
        while cur != p:
            nxt = [n for n in adj[cur] if n != prev]
            if not nxt:
                break
            mark(cur, nxt[0])
            prev, cur = cur, nxt[0]
            path.append(cur)
        pts = [xy_of(*u) for u in path]
        pts[0] = pts[-1] = nodes[k]
        chains.append({'a': k, 'b': k, 'pts': pts})
    return nodes, chains


def _polyline_length(pts):
    return sum(math.hypot(pts[i + 1][0] - pts[i][0], pts[i + 1][1] - pts[i][1])
               for i in range(len(pts) - 1))


def _degree(chains):
    d = defaultdict(int)
    for c in chains:
        d[c['a']] += 1
        d[c['b']] += 1
    return d


def _prune_spurs(nodes, chains, spur_m):
    while True:
        deg = _degree(chains)
        drop = None
        for i, c in enumerate(chains):
            if c['a'] == c['b']:
                continue
            la = deg[c['a']] == 1
            lb = deg[c['b']] == 1
            if (la != lb) and _polyline_length(c['pts']) < spur_m:
                drop = i
                break
            if la and lb and _polyline_length(c['pts']) < spur_m:   # tiny island
                drop = i
                break
        if drop is None:
            return
        chains.pop(drop)
        deg = _degree(chains)
        for k in [k for k in nodes if k not in deg]:
            del nodes[k]


def _join_degree2(nodes, chains):
    while True:
        deg = _degree(chains)
        target = None
        for k, d in deg.items():
            if d == 2:
                inc = [i for i, c in enumerate(chains) if k in (c['a'], c['b'])]
                if len(inc) == 2:           # two different chains (not a loop on k)
                    target = (k, inc)
                    break
        if target is None:
            return
        k, (i, j) = target
        c1, c2 = chains[i], chains[j]
        p1 = c1['pts'] if c1['b'] == k else c1['pts'][::-1]
        a1 = c1['a'] if c1['b'] == k else c1['b']
        p2 = c2['pts'] if c2['a'] == k else c2['pts'][::-1]
        b2 = c2['b'] if c2['a'] == k else c2['a']
        merged = {'a': a1, 'b': b2, 'pts': p1 + p2[1:]}
        for idx in sorted((i, j), reverse=True):
            chains.pop(idx)
        chains.append(merged)
        del nodes[k]


# ------------------------------------------------------------------- polylines
def douglas_peucker(pts, tol):
    if len(pts) < 3:
        return list(pts)
    (x0, y0), (x1, y1) = pts[0], pts[-1]
    dx, dy = x1 - x0, y1 - y0
    norm = math.hypot(dx, dy)
    best, idx = -1.0, 0
    for i in range(1, len(pts) - 1):
        px, py = pts[i]
        d = (math.hypot(px - x0, py - y0) if norm < 1e-9
             else abs(dy * (px - x0) - dx * (py - y0)) / norm)
        if d > best:
            best, idx = d, i
    if best <= tol:
        return [pts[0], pts[-1]]
    left = douglas_peucker(pts[:idx + 1], tol)
    right = douglas_peucker(pts[idx:], tol)
    return left[:-1] + right


class _DSU:
    def __init__(self, items):
        self.p = {i: i for i in items}

    def find(self, x):
        while self.p[x] != x:
            self.p[x] = self.p[self.p[x]]
            x = self.p[x]
        return x

    def union(self, a, b):
        ra, rb = self.find(a), self.find(b)
        if ra != rb:
            self.p[max(ra, rb)] = min(ra, rb)


def _edge_angle(p, q):
    return math.degrees(math.atan2(q[1] - p[1], q[0] - p[0]))


def _axis_kind(p, q, snap_deg):
    a = abs(_edge_angle(p, q)) % 180.0
    if a <= snap_deg or a >= 180.0 - snap_deg:
        return 'h'
    if abs(a - 90.0) <= snap_deg:
        return 'v'
    return None


def _simplify_to_graph(nodes, chains, dp_tol):
    """Cut every chain into straight pieces. -> (pts {id: (x, y)}, edges {(a, b)})."""
    pts = dict(nodes)
    edges = set()
    nxt = max(pts, default=-1) + 1
    for c in chains:
        poly = douglas_peucker(c['pts'], dp_tol)
        ids = [c['a']]
        for p in poly[1:-1]:
            pts[nxt] = p
            ids.append(nxt)
            nxt += 1
        ids.append(c['b'])
        for u, v in zip(ids[:-1], ids[1:]):
            if u != v:
                edges.add((min(u, v), max(u, v)))
    return pts, edges


def _snap_axes(pts, edges, snap_deg):
    xs, ys = _DSU(pts), _DSU(pts)
    for a, b in edges:
        kind = _axis_kind(pts[a], pts[b], snap_deg)
        if kind == 'v':
            xs.union(a, b)
        elif kind == 'h':
            ys.union(a, b)
    gx, gy = defaultdict(list), defaultdict(list)
    for k in pts:
        gx[xs.find(k)].append(k)
        gy[ys.find(k)].append(k)
    new = dict(pts)
    for ks in gx.values():
        if len(ks) > 1:
            mx = sum(pts[k][0] for k in ks) / len(ks)
            for k in ks:
                new[k] = (mx, new[k][1])
    for ks in gy.values():
        if len(ks) > 1:
            my = sum(pts[k][1] for k in ks) / len(ks)
            for k in ks:
                new[k] = (new[k][0], my)
    return new


def _kinds_at(pts, edges, node, skip, snap_deg):
    kinds = set()
    for a, b in edges:
        if (a, b) == skip or node not in (a, b):
            continue
        k = _axis_kind(pts[a], pts[b], snap_deg)
        if k:
            kinds.add(k)          # slanted neighbours do not decide the corner
    return kinds


def _segment_ok(clr_at, p, q, need, slack=0.03):
    n = max(2, int(math.hypot(q[0] - p[0], q[1] - p[1]) / 0.025) + 1)
    return all(clr_at(p[0] + (q[0] - p[0]) * t, p[1] + (q[1] - p[1]) * t) >= need - slack
               for t in np.linspace(0.0, 1.0, n))


def _straighten_corners(pts, edges, clr_at, need, snap_deg, max_len):
    """A skeleton cuts the inside of a bend, which leaves a short slanted edge between a
    horizontal and a vertical corridor. Replace both of its ends by the point where the
    two corridor lines meet, if the robot can really stand and drive there."""
    changed = True
    while changed:
        changed = False
        for u, v in sorted(edges):
            if _axis_kind(pts[u], pts[v], snap_deg) is not None:
                continue
            if math.hypot(pts[u][0] - pts[v][0], pts[u][1] - pts[v][1]) > max_len:
                continue
            ku = _kinds_at(pts, edges, u, (u, v), snap_deg)
            kv = _kinds_at(pts, edges, v, (u, v), snap_deg)
            if len(ku) != 1 or len(kv) != 1 or ku == kv:
                continue
            if 'v' in ku:
                corner = (pts[u][0], pts[v][1])
            else:
                corner = (pts[v][0], pts[u][1])
            if clr_at is not None and not (_segment_ok(clr_at, pts[u], corner, need)
                                           and _segment_ok(clr_at, pts[v], corner, need)):
                continue
            pts[u] = corner
            new_edges = set()
            for a, b in edges:
                a, b = (u if a == v else a), (u if b == v else b)
                if a != b:
                    new_edges.add((min(a, b), max(a, b)))
            edges.clear()
            edges.update(new_edges)
            del pts[v]
            changed = True
            break
    return pts, edges


def _recenter(pts, edges, clr_at, snap_deg, reach=0.20, step=0.025, passes=2):
    """Slide every straight horizontal / vertical line sideways (up to `reach`) to where the
    smallest clearance along it is largest. A thinned centre line leans to one side in
    corridors of even width; this puts it back in the middle."""
    if clr_at is None:
        return pts
    offsets = [round(i * step, 6) for i in range(-int(reach / step), int(reach / step) + 1)]
    for _ in range(passes):
        for axis, kind in ((0, 'v'), (1, 'h')):
            dsu = _DSU(pts)
            for a, b in edges:
                if _axis_kind(pts[a], pts[b], snap_deg) == kind:
                    dsu.union(a, b)
            groups = defaultdict(list)
            for k in pts:
                groups[dsu.find(k)].append(k)
            for ks in groups.values():
                ke = [(a, b) for a, b in edges
                      if a in ks and b in ks and _axis_kind(pts[a], pts[b], snap_deg) == kind]
                if not ke:
                    continue
                best_score, best_d = None, 0.0
                for d in offsets:
                    worst = 1e9
                    for a, b in ke:
                        p = list(pts[a])
                        q = list(pts[b])
                        p[axis] += d
                        q[axis] += d
                        n = max(2, int(math.hypot(q[0] - p[0], q[1] - p[1]) / 0.05) + 1)
                        for t in np.linspace(0.0, 1.0, n):
                            worst = min(worst, clr_at(p[0] + (q[0] - p[0]) * t,
                                                      p[1] + (q[1] - p[1]) * t))
                    score = (round(worst, 3), -abs(d))
                    if best_score is None or score > best_score:
                        best_score, best_d = score, d
                for k in ks:
                    pt = list(pts[k])
                    pt[axis] += best_d
                    pts[k] = tuple(pt)
    return pts


def _merge_close(pts, edges, merge_m):
    dsu = _DSU(pts)
    ids = sorted(pts)
    for i, a in enumerate(ids):
        for b in ids[i + 1:]:
            if math.hypot(pts[a][0] - pts[b][0], pts[a][1] - pts[b][1]) <= merge_m:
                dsu.union(a, b)
    groups = defaultdict(list)
    for k in ids:
        groups[dsu.find(k)].append(k)
    newpts = {g: (sum(pts[k][0] for k in ks) / len(ks), sum(pts[k][1] for k in ks) / len(ks))
              for g, ks in groups.items()}
    newedges = set()
    for a, b in edges:
        u, v = dsu.find(a), dsu.find(b)
        if u != v:
            newedges.add((min(u, v), max(u, v)))
    return newpts, newedges


def _drop_passthrough(pts, edges, straight_deg=6.0, keep=()):
    """Remove degree-2 nodes whose two edges continue in (nearly) the same direction."""
    changed = True
    while changed:
        changed = False
        nbr = defaultdict(set)
        for a, b in edges:
            nbr[a].add(b)
            nbr[b].add(a)
        for k in sorted(pts):
            if k in keep or len(nbr[k]) != 2:
                continue
            u, v = sorted(nbr[k])
            a1 = _edge_angle(pts[u], pts[k])
            a2 = _edge_angle(pts[k], pts[v])
            turn = abs((a2 - a1 + 180.0) % 360.0 - 180.0)
            if turn <= straight_deg and (min(u, v), max(u, v)) not in edges:
                edges.discard((min(u, k), max(u, k)))
                edges.discard((min(v, k), max(v, k)))
                edges.add((min(u, v), max(u, v)))
                del pts[k]
                changed = True
                break
    return pts, edges


def build_graph(skel, xy_of, clr_at=None, spur_m=0.40, dp_tol_m=0.10, snap_deg=8.0,
                merge_m=0.15, min_edge_m=0.10, need=0.40, corner_max_m=1.2):
    """skel: bool array of the centre line; xy_of(row, col) -> world (x, y);
    clr_at(x, y) -> clearance in metres (optional, only to fill min_clearance_m)."""
    nodes, chains = _chains_from_pixels(skel, xy_of)
    _prune_spurs(nodes, chains, spur_m)
    _join_degree2(nodes, chains)
    pts, edges = _simplify_to_graph(nodes, chains, dp_tol_m)
    pts = _snap_axes(pts, edges, snap_deg)
    pts, edges = _straighten_corners(pts, edges, clr_at, need, snap_deg, corner_max_m)
    pts = _snap_axes(pts, edges, snap_deg)
    pts, edges = _merge_close(pts, edges, merge_m)
    pts = _snap_axes(pts, edges, snap_deg)
    pts, edges = _drop_passthrough(pts, edges)
    # a second pass: removing a node can leave a piece that is axis aligned again
    pts = _snap_axes(pts, edges, snap_deg)
    pts = _recenter(pts, edges, clr_at, snap_deg)
    edges = {e for e in edges if math.hypot(pts[e[0]][0] - pts[e[1]][0],
                                            pts[e[0]][1] - pts[e[1]][1]) >= min_edge_m}
    used = {k for e in edges for k in e}
    order = sorted(used, key=lambda k: (round(pts[k][0], 3), round(pts[k][1], 3)))
    ident = {k: 'n%02d' % (i + 1) for i, k in enumerate(order)}
    g = Graph()
    for k in order:
        g.nodes[ident[k]] = pts[k]
    for e in sorted(edges, key=lambda e: (ident[e[0]], ident[e[1]])):
        a, b = sorted((ident[e[0]], ident[e[1]]))
        p, q = g.nodes[a], g.nodes[b]
        edge = {'id': 'e%02d' % (len(g.edges) + 1), 'a': a, 'b': b,
                'length_m': math.hypot(q[0] - p[0], q[1] - p[1]),
                'angle_deg': _edge_angle(p, q), 'min_clearance_m': None}
        if clr_at is not None:
            n = max(2, int(edge['length_m'] / 0.025) + 1)
            edge['min_clearance_m'] = min(
                clr_at(p[0] + (q[0] - p[0]) * t, p[1] + (q[1] - p[1]) * t)
                for t in np.linspace(0.0, 1.0, n))
        g.edges.append(edge)
    return g
