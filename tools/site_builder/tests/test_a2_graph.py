#!/usr/bin/env python3
"""A-2: centre line -> straight corridors on synthetic maps of different shapes.
Run: python3 tools/site_builder/tests/test_a2_graph.py"""
import math
import os
import sys
import tempfile

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, '..'))
from site_builder.cli import generate_graph   # noqa: E402
from site_builder.clearance import clearance_map   # noqa: E402
from site_builder.mapio import MapData   # noqa: E402

REPO = os.path.abspath(os.path.join(HERE, '..', '..', '..'))
RES = 0.05
NEED = 0.40


def build(tmp, free_rects=(), block_rects=(), img=None, h=120, w=160):
    if img is None:
        img = np.zeros((h, w), np.uint8)               # everything is wall
        for r0, r1, c0, c1 in free_rects:
            img[r0:r1, c0:c1] = 254
        for r0, r1, c0, c1 in block_rects:
            img[r0:r1, c0:c1] = 0
    h, w = img.shape
    with open(os.path.join(tmp, 'm.pgm'), 'wb') as f:
        f.write(b'P5\n%d %d\n255\n' % (w, h) + img.tobytes())
    with open(os.path.join(tmp, 'm.yaml'), 'w') as f:
        f.write('image: m.pgm\nmode: trinary\nresolution: %s\norigin: [0.0, 0.0, 0.0]\n'
                'negate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.196\n' % RES)
    return MapData(os.path.join(tmp, 'm.yaml'))


def run(mp):
    clr = clearance_map(mp.blocked, mp.res, 1.0)
    g, _drv, _skel = generate_graph(mp, clr, NEED, 0.40, 0.10)
    return g


def xy(col, row, mp):
    return mp.xy_of(row, col)


def near(p, q, tol):
    return math.hypot(p[0] - q[0], p[1] - q[1]) <= tol


def axis_aligned(g):
    for e in g.edges:
        a = abs(e['angle_deg']) % 180.0
        if min(a, abs(a - 90.0), abs(a - 180.0)) > 1e-6:
            return False
    return True


def has_node_near(g, p, tol=0.12):
    return any(near(v, p, tol) for v in g.nodes.values())


def test_l_shape():
    with tempfile.TemporaryDirectory() as tmp:
        mp = build(tmp, [(50, 74, 20, 140), (10, 74, 116, 140)])
        g = run(mp)
        assert len(g.nodes) == 3 and len(g.edges) == 2, (len(g.nodes), len(g.edges))
        assert axis_aligned(g)
        assert has_node_near(g, xy(128, 62, mp)), 'the bend must sit where the centre lines meet'


def test_cross():
    with tempfile.TemporaryDirectory() as tmp:
        mp = build(tmp, [(50, 74, 10, 150), (10, 110, 68, 92)])
        g = run(mp)
        assert len(g.nodes) == 5 and len(g.edges) == 4, (len(g.nodes), len(g.edges))
        assert axis_aligned(g)
        assert has_node_near(g, xy(80, 62, mp))


def test_ring():
    with tempfile.TemporaryDirectory() as tmp:
        mp = build(tmp, [(20, 100, 20, 140)], [(44, 76, 44, 116)])
        g = run(mp)
        assert len(g.nodes) == 4 and len(g.edges) == 4, (len(g.nodes), len(g.edges))
        assert axis_aligned(g)
        for c, r in ((32, 32), (128, 32), (128, 88), (32, 88)):
            assert has_node_near(g, xy(c, r, mp)), (c, r)


def test_small_niche_is_not_a_corridor():
    with tempfile.TemporaryDirectory() as tmp:
        mp = build(tmp, [(50, 74, 20, 140), (40, 50, 60, 84)])
        g = run(mp)
        assert len(g.nodes) == 2 and len(g.edges) == 1, (len(g.nodes), len(g.edges))
        assert axis_aligned(g)


def test_slanted_corridor_stays_slanted():
    with tempfile.TemporaryDirectory() as tmp:
        h, w = 120, 160
        rr, cc = np.mgrid[0:h, 0:w]
        p0, p1 = np.array([90.0, 30.0]), np.array([30.0, 130.0])      # (row, col)
        d = p1 - p0
        t = np.clip(((rr - p0[0]) * d[0] + (cc - p0[1]) * d[1]) / (d @ d), 0, 1)
        dist = np.hypot(rr - (p0[0] + t * d[0]), cc - (p0[1] + t * d[1]))
        img = np.where(dist <= 13, 254, 0).astype(np.uint8)
        mp = build(tmp, img=img)
        g = run(mp)
        assert len(g.edges) == 1 and len(g.nodes) == 2, (len(g.nodes), len(g.edges))
        ang = g.edges[0]['angle_deg']
        assert 25.0 <= abs(ang) <= 37.0 or 143.0 <= abs(ang) <= 155.0, ang   # ~31 deg, not snapped


def test_every_edge_is_drivable():
    with tempfile.TemporaryDirectory() as tmp:
        mp = build(tmp, [(50, 74, 10, 150), (10, 110, 68, 92)])
        g = run(mp)
        for e in g.edges:
            assert e['min_clearance_m'] >= NEED - 0.03, e


def seg_dist(p, a, b):
    ax, ay, bx, by = a[0], a[1], b[0], b[1]
    dx, dy = bx - ax, by - ay
    t = 0.0 if dx == dy == 0 else max(0.0, min(1.0, ((p[0] - ax) * dx + (p[1] - ay) * dy) / (dx * dx + dy * dy)))
    return math.hypot(p[0] - (ax + t * dx), p[1] - (ay + t * dy))


def test_demo_map_recovers_the_hand_made_grid():
    import yaml
    from site_builder.cli import prepare

    class A:
        map = os.path.join(REPO, 'maps/stores/demo_store/map.yaml')
        entities = os.path.join(REPO, 'config/common/entities.yaml')
    if not os.path.exists(A.map):
        return
    mp, _extra, clr = prepare(A)
    g, _d, _s = generate_graph(mp, clr, NEED, 0.40, 0.10)
    wp = yaml.safe_load(open(os.path.join(REPO, 'config/common/waypoints.yaml')))
    nodes = {k: (v['x'], v['y']) for k, v in wp['nodes'].items()}
    gedges = [(g.nodes[e['a']], g.nodes[e['b']]) for e in g.edges]
    # every hand made node lies on (or very near) a generated corridor
    for k, p in nodes.items():
        d = min(seg_dist(p, ga, gb) for ga, gb in gedges)
        assert d <= 0.30, 'node %s is %.2f m away from the generated graph' % (k, d)
    # and 12 of the 13 hand made corridors are found along their whole length. The 13th
    # (e_north_c) hugs the lower wall of a wide open area (clearance 0.43 m, a validator
    # WARN); the generator follows the middle of that area instead, which is correct.
    found = []
    for e in wp['edges']:
        a, b = nodes[e['from']], nodes[e['to']]
        n = int(math.hypot(b[0] - a[0], b[1] - a[1]) / 0.05) + 1
        pts = [(a[0] + (b[0] - a[0]) * i / n, a[1] + (b[1] - a[1]) * i / n) for i in range(n + 1)]
        ok = sum(1 for p in pts if min(seg_dist(p, ga, gb) for ga, gb in gedges) <= 0.20)
        if ok / len(pts) >= 0.95:
            found.append(e['id'])
    assert len(found) >= 12, found


if __name__ == '__main__':
    for name, fn in sorted(globals().items()):
        if name.startswith('test_') and callable(fn):
            fn()
            print('ok  ', name)
    print('all passed')
