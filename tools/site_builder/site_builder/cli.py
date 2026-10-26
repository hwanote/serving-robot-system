"""Command line for the site builder.   python -m site_builder.cli <command>

clearance : draw how much room there is around every free cell (A-1 check)
"""
import argparse
import os
import sys

import numpy as np
import yaml

from .clearance import clearance_map, rasterize_rects
from .extract import clearance_sampler, generate_graph, keep_big_components  # noqa: F401
from .furniture import load_entities, table_rects
from .mapio import MapData, save_png

# robot (scripts/validate_site_config.py defaults)
MIN_CORRIDOR_WIDTH_M = 0.80


def draw_clearance(mp, clr, need, scale, overlays=()):
    """RGB image: dark = blocked, grey = unknown, light red = free but too narrow,
    yellow = tight (need..need+0.10), green = comfortable."""
    img = np.zeros((mp.h, mp.w, 3), dtype=np.uint8)
    img[:] = (250, 190, 190)                         # free, too narrow
    img[clr >= need] = (250, 225, 110)               # tight
    img[clr >= need + 0.10] = (120, 200, 120)        # ok
    img[mp.unknown] = (170, 170, 170)
    img[mp.occupied] = (30, 30, 30)
    img = np.kron(img, np.ones((scale, scale, 1), dtype=np.uint8))
    def px(x, y):
        return int((x - mp.ox) / mp.res * scale), int((mp.h - (y - mp.oy) / mp.res) * scale)

    for wp, edge_color, node_color, half in overlays:
        pts = {k: (v['x'], v['y']) for k, v in wp['nodes'].items()}
        for e in wp.get('edges', []):
            (x0, y0), (x1, y1) = px(*pts[e['from']]), px(*pts[e['to']])
            for t in np.linspace(0, 1, max(abs(x1 - x0), abs(y1 - y0)) + 1):
                x, y = int(x0 + (x1 - x0) * t), int(y0 + (y1 - y0) * t)
                img[max(y - 1, 0):y + 2, max(x - 1, 0):x + 2] = edge_color
        for nx, ny in pts.values():
            x, y = px(nx, ny)
            img[max(y - half, 0):y + half + 1, max(x - half, 0):x + half + 1] = node_color
    return img


def cmd_clearance(a):
    mp = MapData(a.map)
    rects = []
    if a.entities:
        rects = table_rects(load_entities(a.entities))
    extra = rasterize_rects((mp.h, mp.w), mp, rects) if rects else np.zeros((mp.h, mp.w), bool)
    clr = clearance_map(mp.blocked | extra, mp.res, cap_m=1.0)
    need = a.min_width / 2.0
    wp = None
    if a.waypoints:
        with open(a.waypoints, 'r', encoding='utf-8') as f:
            wp = yaml.safe_load(f)
    free_cells = int(mp.free.sum())
    ok = int(((clr >= need) & ~extra & mp.free).sum())
    print('map %dx%d @ %.3f m, free %d cells, clearance >= %.2f m: %d cells (%.0f%% of free)'
          % (mp.w, mp.h, mp.res, free_cells, need, ok, 100.0 * ok / max(free_cells, 1)))
    os.makedirs(os.path.dirname(os.path.abspath(a.out)), exist_ok=True)
    overlays = [(wp, (30, 60, 220), (200, 30, 30), 4)] if wp else []
    save_png(a.out, draw_clearance(mp, np.where(extra, 0.0, clr), need, a.scale, overlays))
    print('wrote', a.out)
    return 0


def prepare(a):
    """map, obstacle mask incl. furniture, clearance, drivable mask"""
    mp = MapData(a.map)
    rects = table_rects(load_entities(a.entities)) if a.entities else []
    extra = rasterize_rects((mp.h, mp.w), mp, rects) if rects else np.zeros((mp.h, mp.w), bool)
    clr = np.where(extra, 0.0, clearance_map(mp.blocked | extra, mp.res, cap_m=1.0))
    return mp, extra, clr


def cmd_graph(a):
    mp, extra, clr = prepare(a)
    need = a.min_width / 2.0
    g, drivable, skel = generate_graph(mp, clr, need, a.spur, a.dp_tol)
    d = g.to_dict()
    print('drivable %d cells, centre line %d px -> %d nodes, %d edges'
          % (int(drivable.sum()), int(skel.sum()), len(d['nodes']), len(d['edges'])))
    for e in d['edges']:
        flag = ''
        if e['min_clearance_m'] is not None and e['min_clearance_m'] < need - 0.03:
            flag = '  <- below %.2f' % need
        print('  %-4s %-4s -> %-4s %5.2f m  %6.1f deg  clearance %s%s'
              % (e['id'], e['from'], e['to'], e['length_m'], e['angle_deg'],
                 e['min_clearance_m'], flag))
    os.makedirs(os.path.dirname(os.path.abspath(a.out)), exist_ok=True)
    overlays = [(d, (200, 30, 200), (255, 120, 0), 5)]
    if a.compare:
        with open(a.compare, 'r', encoding='utf-8') as f:
            overlays.insert(0, (yaml.safe_load(f), (30, 60, 220), (200, 30, 30), 3))
    img = draw_clearance(mp, clr, need, a.scale, overlays)
    save_png(a.out, img)
    print('wrote', a.out)
    if a.json:
        import json
        with open(a.json, 'w', encoding='utf-8') as f:
            json.dump(d, f, indent=2)
        print('wrote', a.json)
    return 0


def cmd_generate(a):
    from .params import load_params
    from .pipeline import generate, write_outputs
    par = load_params(min_corridor_m=a.min_width, spur_m=a.spur, dp_tol_m=a.dp_tol)
    res = generate(a.map, a.labels, par, adjustments=a.adjustments)
    write_outputs(res, a.out_dir)
    print('HardStop corner radius %.3f m' % par.corner_radius_m)
    for n, e in res.graph.nodes.items():
        chk = res.node_checks[n]
        print('  node %-4s (%6.2f, %6.2f)  room %.3f  margin %+.3f  %s'
              % (n, e[0], e[1], chk['clearance_m'], chk['margin_m'], chk['level']))
    for e in res.graph.edges:
        print('  edge %-4s %s -> %s  %.2f m  %.0f deg  clearance %.2f  %s'
              % (e['id'], e['a'], e['b'], e['length_m'], e['angle_deg'],
                 res.edge_checks[e['id']]['clearance_m'], res.edge_checks[e['id']]['level']))
    for n, p in res.poses.items():
        print('  pose %-11s node %-4s yaw %6.1f  %-13s HardStop margin %.3f  %s  %s'
              % (n, p['node'], p['yaw_deg'], p['kind'], p['hardstop_margin_m'], p['level'],
                 p['aliases']))
    for x in res.excluded:
        print('  EXCLUDED %s: %s' % (x['id'], x['reason']))
    for w in res.warnings:
        print('  WARN', w)
    print('wrote', a.out_dir)
    if a.png:
        mp, _, clr = prepare(argparse.Namespace(map=a.map, entities=None))
        from .pipeline import table_rect_list
        d = res.graph.to_dict()
        img = draw_clearance(mp, clr, a.min_width / 2.0, a.scale, [(d, (200, 30, 200), (255, 120, 0), 5)])
        save_png(a.png, img)
        print('wrote', a.png)
    return 0


def main(argv=None):
    p = argparse.ArgumentParser(prog='site_builder')
    sub = p.add_subparsers(dest='cmd', required=True)
    c = sub.add_parser('clearance', help='draw the room around every free cell')
    c.add_argument('--map', required=True)
    c.add_argument('--entities', help='entities.yaml: adds table tops and chairs as obstacles')
    c.add_argument('--waypoints', help='waypoints.yaml to draw on top (to compare)')
    c.add_argument('--min-width', type=float, default=MIN_CORRIDOR_WIDTH_M)
    c.add_argument('--scale', type=int, default=6)
    c.add_argument('--out', default='tools/site_builder/out/clearance.png')
    c.set_defaults(func=cmd_clearance)
    g = sub.add_parser('graph', help='extract straight corridors (nodes and edges)')
    g.add_argument('--map', required=True)
    g.add_argument('--entities')
    g.add_argument('--compare', help='waypoints.yaml to draw in blue for comparison')
    g.add_argument('--min-width', type=float, default=MIN_CORRIDOR_WIDTH_M)
    g.add_argument('--spur', type=float, default=0.40, help='drop dead ends shorter than this (m)')
    g.add_argument('--dp-tol', type=float, default=0.10, help='straightness tolerance (m)')
    g.add_argument('--scale', type=int, default=6)
    g.add_argument('--out', default='tools/site_builder/out/graph.png')
    g.add_argument('--json')
    g.set_defaults(func=cmd_graph)
    q = sub.add_parser('generate', help='labels.json + map -> waypoints/entities/zones/preview')
    q.add_argument('--map', required=True)
    q.add_argument('--labels', required=True)
    q.add_argument('--out-dir', default='config/generated')
    q.add_argument('--min-width', type=float, default=MIN_CORRIDOR_WIDTH_M)
    q.add_argument('--spur', type=float, default=0.40)
    q.add_argument('--dp-tol', type=float, default=0.10)
    q.add_argument('--scale', type=int, default=6)
    q.add_argument('--png')
    q.add_argument('--adjustments', help='adjustments.json: only proposals with status approved are applied')
    q.set_defaults(func=cmd_generate)
    a = p.parse_args(argv)
    return a.func(a)


if __name__ == '__main__':
    sys.exit(main())
