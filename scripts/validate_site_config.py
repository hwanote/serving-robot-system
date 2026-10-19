#!/usr/bin/env python3
"""Check config/common/{entities,waypoints,zones}.yaml against the map and the world.

Run it whenever the map or one of the three files changes (or after the route
generator wrote new values). No ROS needed:

    python3 scripts/validate_site_config.py
    python3 scripts/validate_site_config.py --map maps/stores/<new_store>/map.yaml

What it checks (FAIL = must fix, WARN = look at it):
  * references: nodes, edges, poses, zones, aliases are consistent
  * world: table/fixture models exist in the Gazebo world at the same position
    (entries without world_model, e.g. from the site generator, are listed as WARN and skipped)
  * nodes: on free map cells, room to turn in place. The HardStop rectangle (robot body +
    hard_stop_distance_m on every side, config/common/runtime_limits.yaml) sweeps a circle
    of radius hypot(front, half width) when the robot turns: FAIL below radius + 0.02 m,
    WARN below radius + 0.10 m
  * edges: straight corridors; clearance from the centre line to the nearest
    obstacle (map walls + table tops + chairs) is at least half the minimum
    corridor width (0.8 m -> 0.40 m)
  * poses: the HardStop rectangle fits on the pose, table/kitchen is straight ahead
  * zones: every node belongs to a zone
  * graph: connected, every pose reachable, and which single blocked corridor
    would cut a pose off (no detour possible)

Exit code = number of FAILs.
"""
import argparse
import math
import os
import re
import sys

import numpy as np
import yaml

REPO = os.path.abspath(os.path.join(os.path.dirname(__file__), '..'))
sys.path.insert(0, os.path.join(REPO, 'ros_ws', 'src', 'serving_robot_navigation'))
from serving_robot_navigation.site_config import SiteConfigError, load_site  # noqa: E402

FREE_PIXEL_MIN = 250   # 254 = free; 205 = unknown and 0 = occupied are blocked


class Report:
    def __init__(self):
        self.rows = []

    def add(self, level, where, msg):
        self.rows.append((level, where, msg))

    def ok(self, where, msg):
        self.add('OK', where, msg)

    def warn(self, where, msg):
        self.add('WARN', where, msg)

    def fail(self, where, msg):
        self.add('FAIL', where, msg)

    def count(self, level):
        return sum(1 for r in self.rows if r[0] == level)


# --------------------------------------------------------------------------- map
def read_pgm(path):
    with open(path, 'rb') as f:
        data = f.read()
    tokens, i = [], 0
    while len(tokens) < 4:
        while data[i:i + 1].isspace():
            i += 1
        if data[i:i + 1] == b'#':
            while data[i:i + 1] not in (b'\n', b''):
                i += 1
            continue
        j = i
        while not data[j:j + 1].isspace():
            j += 1
        tokens.append(data[i:j])
        i = j
    magic, w, h, maxv = tokens[0], int(tokens[1]), int(tokens[2]), int(tokens[3])
    if magic != b'P5' or maxv > 255:
        raise ValueError('only 8-bit binary PGM (P5) is supported: %s' % path)
    i += 1
    return np.frombuffer(data[i:i + w * h], dtype=np.uint8).reshape(h, w)


class MapInfo:
    def __init__(self, yaml_path):
        with open(yaml_path, 'r', encoding='utf-8') as f:
            m = yaml.safe_load(f)
        self.res = float(m['resolution'])
        self.ox, self.oy = float(m['origin'][0]), float(m['origin'][1])
        img = read_pgm(os.path.join(os.path.dirname(yaml_path), m['image']))
        if int(m.get('negate', 0)):
            img = 255 - img
        self.img = img                      # row 0 = top (max y)
        self.h, self.w = img.shape
        self.free = img >= FREE_PIXEL_MIN
        rows, cols = np.nonzero(~self.free)
        cx = self.ox + (cols + 0.5) * self.res
        cy = self.oy + (self.h - rows - 0.5) * self.res
        half = self.res / 2.0
        self.blocked = np.stack([cx - half, cx + half, cy - half, cy + half], 1)
        self.revision = None

    def cell_free(self, x, y):
        col = int(math.floor((x - self.ox) / self.res))
        row = self.h - 1 - int(math.floor((y - self.oy) / self.res))
        if not (0 <= col < self.w and 0 <= row < self.h):
            return False
        return bool(self.free[row, col])


def clearance(points, rects):
    """Distance from each point (n,2) to the nearest rectangle (m,4: x0,x1,y0,y1)."""
    if len(rects) == 0:
        return np.full(len(points), np.inf)
    px = points[:, 0:1]
    py = points[:, 1:2]
    dx = np.maximum(np.maximum(rects[:, 0] - px, px - rects[:, 1]), 0.0)
    dy = np.maximum(np.maximum(rects[:, 2] - py, py - rects[:, 3]), 0.0)
    return np.sqrt(dx * dx + dy * dy).min(axis=1)


def furniture_rects(site):
    rects = []
    for t in (site.entities.get('tables') or {}).values():
        cx, cy = t['center']['x'], t['center']['y']
        sx, sy = t['top_size_m']['x'] / 2, t['top_size_m']['y'] / 2
        rects.append([cx - sx, cx + sx, cy - sy, cy + sy])
        for c in t.get('chairs') or []:
            h = c['size_m'] / 2
            rects.append([cx + c['dx'] - h, cx + c['dx'] + h,
                          cy + c['dy'] - h, cy + c['dy'] + h])
    return np.array(rects, dtype=float).reshape(-1, 4)


def table_rect(site, name):
    t = site.entities['tables'][name]
    cx, cy = t['center']['x'], t['center']['y']
    sx, sy = t['top_size_m']['x'] / 2, t['top_size_m']['y'] / 2
    return [cx - sx, cx + sx, cy - sy, cy + sy]


def fixture_rect(site, name):
    f = site.entities['fixtures'][name]
    cx, cy = f['center']['x'], f['center']['y']
    if f['kind'] == 'box':
        sx, sy = f['size_m']['x'] / 2, f['size_m']['y'] / 2
    else:
        sx = sy = f['radius_m']
    return [cx - sx, cx + sx, cy - sy, cy + sy]


# --------------------------------------------------------------------------- world
def sdf_model_poses(path):
    """{model name: (x, y)} of the top-level models in an SDF world (best effort)."""
    text = open(path, 'r', encoding='utf-8').read()
    out = {}
    for m in re.finditer(r'<model name="([^"]+)">', text):
        tail = text[m.end():]
        stop = tail.find('<link')
        head = tail[:stop if stop > 0 else 400]
        p = re.search(r'<pose>([^<]+)</pose>', head)
        vals = [float(v) for v in p.group(1).split()] if p else [0.0] * 6
        out[m.group(1)] = (vals[0], vals[1])
    return out


# --------------------------------------------------------------------------- checks
def check_world(site, world_path, rep):
    if not world_path or not os.path.isfile(world_path):
        rep.warn('world', 'world file not found, skipped: %s' % world_path)
        return
    models = sdf_model_poses(world_path)
    # entities written by the site generator have no world_model (no Gazebo world yet):
    # they cannot be compared with the world, say so once instead of stopping
    every = [(n, t) for n, t in (site.entities.get('tables') or {}).items()]
    every += [(n, f) for n, f in (site.entities.get('fixtures') or {}).items()]
    unlinked = [n for n, e in every if not e.get('world_model')]
    if unlinked:
        rep.warn('world', 'not compared with the world (no world_model): %s' % ', '.join(unlinked))
    items = [(n, e['world_model'], e['center']) for n, e in every if e.get('world_model')]
    for name, model, c in items:
        if model not in models:
            rep.fail('world', "%s: model '%s' is not in the world" % (name, model))
            continue
        mx, my = models[model]
        d = math.hypot(mx - c['x'], my - c['y'])
        if d > 0.02:
            rep.fail('world', '%s: config (%.2f, %.2f) differs from world (%.2f, %.2f)'
                     % (name, c['x'], c['y'], mx, my))
        else:
            rep.ok('world', '%s = %s at (%.2f, %.2f)' % (name, model, mx, my))
    known = {t.get('world_model') for t in (site.entities.get('tables') or {}).values()}
    for model in models:
        if unlinked:
            break    # tables without a model: every world table would be reported as missing
        if model.startswith('table_set') and model not in known:
            rep.warn('world', "world has '%s' but entities.yaml has no table for it" % model)


def robot_footprint_points(x, y, yaw, length, width, step=0.05):
    hl, hw = length / 2, width / 2
    pts = []
    for u in np.arange(-hl, hl + 1e-9, step):
        pts += [(u, -hw), (u, hw)]
    for v in np.arange(-hw, hw + 1e-9, step):
        pts += [(-hl, v), (hl, v)]
    c, s = math.cos(yaw), math.sin(yaw)
    return np.array([(x + u * c - v * s, y + u * s + v * c) for u, v in pts])


def hard_stop_distance(args):
    """HardStop margin: --hard-stop, else runtime_limits.yaml (single source), else 0.10."""
    if args.hard_stop is not None:
        return args.hard_stop
    try:
        with open(os.path.join(args.config_dir, 'runtime_limits.yaml'), encoding='utf-8') as f:
            return float(yaml.safe_load(f)['obstacle_reaction']['hard_stop_distance_m'])
    except (OSError, KeyError, TypeError, ValueError):
        pass
    try:
        with open(os.path.join(REPO, 'config', 'common', 'runtime_limits.yaml'),
                  encoding='utf-8') as f:
            return float(yaml.safe_load(f)['obstacle_reaction']['hard_stop_distance_m'])
    except (OSError, KeyError, TypeError, ValueError):
        return 0.10


def run(args):
    rep = Report()

    try:
        site = load_site(args.config_dir)
    except (SiteConfigError, KeyError, OSError) as e:
        rep.fail('load', 'cannot load the configuration: %r' % e)
        return rep, None
    rep.ok('load', 'config revision %s (%s)' % (site.revision, site.config_dir))
    for f, d in (('entities', site.entities), ('waypoints', site.waypoints)):
        if 'status' not in d:
            rep.warn(f, "no 'status' field (provisional | reviewed)")
    rep.ok('refs', '%d nodes, %d edges, %d poses, %d zones'
           % (len(site.nodes), len(site.edges), len(site.poses), len(site.zones)))

    mp = MapInfo(args.map)
    rects = np.vstack([mp.blocked, furniture_rects(site)]) if not args.no_furniture \
        else mp.blocked
    rep.ok('map', '%s  %dx%d px @ %.3f m' % (os.path.basename(os.path.dirname(args.map)),
                                              mp.w, mp.h, mp.res))

    check_world(site, args.world, rep)

    half_len = args.robot_length / 2         # body front edge (front-gap rule, unchanged)
    hs = hard_stop_distance(args)
    hs_len = args.robot_length + 2 * hs      # HardStop rectangle around the body
    hs_wid = args.robot_width + 2 * hs
    circ = math.hypot(hs_len, hs_wid) / 2    # radius its corners sweep when turning in place
    turn_fail = circ + args.turn_fail_margin
    turn_warn = circ + args.turn_warn_margin
    need = args.min_corridor_width / 2
    rep.ok('rules', 'HardStop %.2f x %.2f m (body + %.2f m); turn radius %.3f m '
           '(FAIL < %.3f, WARN < %.3f); corridor clearance >= %.2f m'
           % (hs_len, hs_wid, hs, circ, turn_fail, turn_warn, need))

    # nodes: free cell + room to turn
    for nid, (x, y) in site.nodes.items():
        if not mp.cell_free(x, y):
            rep.fail('node ' + nid, '(%.2f, %.2f) is not on a free map cell' % (x, y))
            continue
        c = float(clearance(np.array([[x, y]]), rects)[0])
        if c < turn_fail:
            rep.fail('node ' + nid, 'no room to turn: nearest obstacle %.3f m < %.3f m '
                     '(HardStop corner radius %.3f m)' % (c, turn_fail, circ))
        elif c < turn_warn:
            rep.warn('node ' + nid, 'tight turn: nearest obstacle %.3f m < %.3f m '
                     '(HardStop corner radius %.3f m)' % (c, turn_warn, circ))
        else:
            rep.ok('node ' + nid, 'free, nearest obstacle %.2f m' % c)
        if site.zone_at(x, y) is None:
            rep.warn('node ' + nid, 'not inside any zone')

    # edges: sampled clearance along the straight segment
    for e in site.edges.values():
        (ax, ay), (bx, by) = site.nodes[e.a], site.nodes[e.b]
        n = max(2, int(e.length_m / 0.05) + 1)
        t = np.linspace(0.0, 1.0, n)
        pts = np.stack([ax + (bx - ax) * t, ay + (by - ay) * t], 1)
        free_ok = all(mp.cell_free(px, py) for px, py in pts)
        cl = clearance(pts, rects)
        cmin = float(cl.min())
        where = 'edge ' + e.id
        ang = math.degrees(math.atan2(by - ay, bx - ax))
        if e.length_m < 0.1:
            rep.fail(where, 'segment is shorter than 0.1 m')
        elif not free_ok or cmin <= 0.0:
            rep.fail(where, 'passes through an obstacle or unknown area')
        elif cmin < need:
            rep.fail(where, 'too narrow: nearest obstacle %.2f m < %.2f m (width ~%.2f m)'
                     % (cmin, need, 2 * cmin))
        elif cmin < need + 0.10:
            rep.warn(where, 'tight: nearest obstacle %.2f m (%s, %.1f m, %.0f deg)'
                     % (cmin, e.zone, e.length_m, ang))
        else:
            rep.ok(where, 'nearest obstacle %.2f m (%s, %.1f m, %.0f deg)'
                   % (cmin, e.zone, e.length_m, ang))

    # poses
    for p in site.poses.values():
        where = 'pose ' + p.name
        fp = robot_footprint_points(p.x, p.y, p.yaw_rad, hs_len, hs_wid)
        if float(clearance(fp, rects).min()) <= 0.02 or not all(
                mp.cell_free(px, py) for px, py in fp):
            rep.fail(where, 'the HardStop rectangle (body + %.2f m) touches an obstacle '
                     'on this pose' % hs)
            continue
        raw = site.waypoints['poses'][p.name]
        target = raw.get('faces') or {}
        trect = None
        if 'table' in target:
            if target['table'] not in (site.entities.get('tables') or {}):
                rep.fail(where, "faces unknown table '%s'" % target['table'])
                continue
            trect = table_rect(site, target['table'])
        elif 'fixture' in target:
            if target['fixture'] not in (site.entities.get('fixtures') or {}):
                rep.fail(where, "faces unknown fixture '%s'" % target['fixture'])
                continue
            trect = fixture_rect(site, target['fixture'])
        if trect is None:
            rep.ok(where, '%s at node %s, zone %s' % (p.kind, p.node, site.zone_at(p.x, p.y)))
            continue
        cx, cy = (trect[0] + trect[1]) / 2, (trect[2] + trect[3]) / 2
        bearing = math.degrees(math.atan2(cy - p.y, cx - p.x) - p.yaw_rad)
        bearing = (bearing + 180.0) % 360.0 - 180.0
        front = np.array([[p.x + half_len * math.cos(p.yaw_rad),
                           p.y + half_len * math.sin(p.yaw_rad)]])
        gap = float(clearance(front, np.array([trect]))[0])
        if abs(bearing) > 25.0:
            rep.fail(where, 'the target is not straight ahead (%.0f deg off)' % bearing)
        elif gap < 0.15 or gap > 0.90:
            rep.warn(where, 'gap robot front -> target %.2f m (expected 0.15..0.90)' % gap)
        else:
            rep.ok(where, '%s, target %.0f deg off-axis, front gap %.2f m'
                   % (p.kind, bearing, gap))

    # zones
    ids = [z.id for z in site.zones]
    if len(ids) != len(set(ids)):
        rep.fail('zones', 'duplicate zone ids')
    for z in site.zones:
        if not (z.x_min < z.x_max and z.y_min < z.y_max):
            rep.fail('zone ' + z.id, 'empty rectangle')

    # graph
    start = site.poses['standby_01'].node if 'standby_01' in site.poses else None
    if start is None:
        rep.fail('graph', "pose 'standby_01' is missing")
        return rep, site
    for nid in site.nodes:
        if site.shortest_path(start, nid) is None:
            rep.fail('graph', "node '%s' cannot be reached from standby_01" % nid)
    rep.ok('graph', 'every node is reachable from standby_01')

    for p in site.poses.values():
        res = site.shortest_path(start, p.node)
        if res is None:
            continue
        for eid in res[1]:
            if site.shortest_path(start, p.node, blocked_edges=[eid]) is None:
                rep.warn('graph', "blocking '%s' cuts off %s (no detour from standby)"
                         % (eid, p.name))
    return rep, site


def print_routes(site, speed=0.25, omega=0.80):
    if site is None or 'kitchen' not in site.poses:
        return
    print('\nroute lengths on the graph (straight driving at %.2f m/s, 90 deg turn %.1f s):'
          % (speed, (math.pi / 2) / omega))
    k = site.poses['kitchen']
    for name in sorted(n for n in site.poses if n.startswith('table_')):
        p = site.poses[name]
        res = site.shortest_path(k.node, p.node)
        if res:
            nodes, edges, length = res
            turns = max(0, len(nodes) - 2) + 2
            t = length / speed + turns * (math.pi / 2) / omega
            print('  kitchen -> %-9s %5.1f m  %-24s ~%3.0f s'
                  % (name, length, ' > '.join(nodes), t))


def main():
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawTextHelpFormatter)
    p.add_argument('--config-dir', default=os.path.join(REPO, 'config', 'common'))
    p.add_argument('--map', default=os.path.join(REPO, 'maps', 'stores', 'demo_store', 'map.yaml'))
    p.add_argument('--world', default=os.path.join(
        REPO, 'ros_ws', 'src', 'serving_robot_gazebo', 'worlds', 'cafe.world.sdf'))
    p.add_argument('--robot-length', type=float, default=0.52)  # base.xacro base_length
    p.add_argument('--robot-width', type=float, default=0.40)   # base.xacro base_width
    p.add_argument('--min-corridor-width', type=float, default=0.80)
    p.add_argument('--hard-stop', type=float, default=None,
                   help='HardStop margin in m (default: runtime_limits.yaml)')
    p.add_argument('--turn-fail-margin', type=float, default=0.02,
                   help='FAIL when clearance < HardStop corner radius + this')
    p.add_argument('--turn-warn-margin', type=float, default=0.10,
                   help='WARN when clearance < HardStop corner radius + this')
    p.add_argument('--no-furniture', action='store_true',
                   help='ignore table tops and chairs (map walls only)')
    p.add_argument('-q', '--quiet', action='store_true', help='print only WARN and FAIL')
    args = p.parse_args()

    rep, site = run(args)
    for level, where, msg in rep.rows:
        if args.quiet and level == 'OK':
            continue
        print('[%-4s] %-16s %s' % (level, where, msg))
    print_routes(site)
    print('\nresult: OK %d / WARN %d / FAIL %d'
          % (rep.count('OK'), rep.count('WARN'), rep.count('FAIL')))
    return rep.count('FAIL')


if __name__ == '__main__':
    sys.exit(main())
