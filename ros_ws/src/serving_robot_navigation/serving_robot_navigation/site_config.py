"""Site configuration of the serving robot: named destinations, route graph, zones.

Reads the shared files  config/common/{entities,waypoints,zones}.yaml  and
offers the lookups the rest of the robot software needs:

    site = load_site()
    pose = site.resolve("table_03")        # name or alias -> map pose
    site.zone_at(x, y)                     # -> "aisle_middle" (for alarm texts)
    site.shortest_path("W_S", "T13")       # straight-segment route on the graph
    site.route_legs("T13", pose)           # stop-and-turn legs along the route

No ROS dependency, so the validator and unit tests run anywhere.

Where the files are read from (first match wins):
    1. the `config_dir` argument
    2. environment variable SERVING_ROBOT_CONFIG_DIR
    3. the installed package share  (serving_robot_navigation/config/common)
    4. the repository's config/common (running from the source tree)
"""
import hashlib
import heapq
import math
import os
from dataclasses import dataclass

import yaml

CONFIG_ENV = 'SERVING_ROBOT_CONFIG_DIR'
FILES = ('entities.yaml', 'waypoints.yaml', 'zones.yaml')


class SiteConfigError(ValueError):
    """The site configuration is missing or inconsistent."""


@dataclass(frozen=True)
class Pose:
    name: str
    x: float
    y: float
    yaw_rad: float
    node: str
    kind: str

    @property
    def yaw_deg(self):
        return math.degrees(self.yaw_rad)


@dataclass(frozen=True)
class Edge:
    id: str
    a: str
    b: str
    zone: str
    length_m: float

    def other(self, node):
        return self.b if node == self.a else self.a


@dataclass(frozen=True)
class Leg:
    """One straight drive ending with a stop and an in-place turn."""
    node: str
    x: float
    y: float
    yaw_rad: float   # heading on arrival (towards the next leg, or the destination heading)
    final: bool
    # Where the leg comes from and which corridors it covers (a leg through
    # nodes where the route goes straight on covers several edges).
    from_node: str = ''
    path_nodes: tuple = ()   # from_node ... node, in driving order
    edges: tuple = ()        # edge ids between consecutive path_nodes

    @property
    def yaw_deg(self):
        return math.degrees(self.yaw_rad)


@dataclass(frozen=True)
class Zone:
    id: str
    x_min: float
    x_max: float
    y_min: float
    y_max: float

    def contains(self, x, y):
        return self.x_min <= x <= self.x_max and self.y_min <= y <= self.y_max


def default_config_dir():
    env = os.environ.get(CONFIG_ENV)
    if env:
        return env
    try:
        from ament_index_python.packages import get_package_share_directory
        share = os.path.join(
            get_package_share_directory('serving_robot_navigation'),
            'config', 'common')
        if os.path.isdir(share):
            return share
    except Exception:  # noqa: BLE001 - ROS not installed / package not built
        pass
    here = os.path.dirname(os.path.abspath(__file__))
    repo = os.path.abspath(os.path.join(here, '..', '..', '..', '..'))
    return os.path.join(repo, 'config', 'common')


def _read_yaml(path):
    if not os.path.isfile(path):
        raise SiteConfigError('missing config file: %s' % path)
    with open(path, 'r', encoding='utf-8') as f:
        data = yaml.safe_load(f)
    if not isinstance(data, dict):
        raise SiteConfigError('empty or invalid config file: %s' % path)
    return data


def _revision(config_dir):
    """Short hash of the three files (line endings ignored): the "config
    revision" to log next to the map revision."""
    h = hashlib.sha256()
    for name in FILES:
        with open(os.path.join(config_dir, name), 'rb') as f:
            h.update(f.read().replace(b'\r\n', b'\n'))
    return h.hexdigest()[:12]


def config_revision(config_dir=None):
    """The same short hash as Site.revision, without loading the site (the backend
    compares it with the revision a robot reports in its mission_status)."""
    return _revision(config_dir or default_config_dir())


def map_image_fingerprint(map_yaml):
    """Short hash of a map.yaml and the image it names (line endings of the yaml ignored): which
    map picture a robot loaded. The robot reports it in mission_status.map_image and the backend
    compares it with maps/stores/<store>/ (the Nav2 map_server reads that file only at launch, so
    a robot that started before the map changed shows a different value). '' when unreadable."""
    try:
        with open(map_yaml, 'rb') as f:
            raw = f.read().replace(b'\r\n', b'\n')
        meta = yaml.safe_load(raw) or {}
        image = meta.get('image', 'map.pgm')
        if not os.path.isabs(image):
            image = os.path.join(os.path.dirname(os.path.abspath(map_yaml)), image)
        with open(image, 'rb') as f:
            pixels = f.read()
    except (OSError, ValueError, AttributeError, yaml.YAMLError):
        return ''
    h = hashlib.sha256()
    h.update(raw)
    h.update(pixels)
    return h.hexdigest()[:12]


class Site:
    def __init__(self, entities, waypoints, zones, config_dir, revision):
        self.entities = entities
        self.waypoints = waypoints
        self.config_dir = config_dir
        self.revision = revision
        self.frame_id = waypoints.get('frame_id', 'map')

        self.nodes = {}
        for nid, n in (waypoints.get('nodes') or {}).items():
            self.nodes[str(nid)] = (float(n['x']), float(n['y']))

        self.zones = []
        for z in zones.get('zones') or []:
            r = z['rect']
            self.zones.append(Zone(
                str(z['id']), float(r['x_min']), float(r['x_max']),
                float(r['y_min']), float(r['y_max'])))
        zone_ids = {z.id for z in self.zones}

        self.edges = {}
        for e in waypoints.get('edges') or []:
            eid, a, b = str(e['id']), str(e['from']), str(e['to'])
            for n in (a, b):
                if n not in self.nodes:
                    raise SiteConfigError(
                        "edge '%s' refers to unknown node '%s'" % (eid, n))
            if eid in self.edges:
                raise SiteConfigError("duplicate edge id '%s'" % eid)
            zone = str(e.get('zone', ''))
            if zone and zone not in zone_ids:
                raise SiteConfigError(
                    "edge '%s' refers to unknown zone '%s'" % (eid, zone))
            ax, ay = self.nodes[a]
            bx, by = self.nodes[b]
            self.edges[eid] = Edge(eid, a, b, zone, math.hypot(bx - ax, by - ay))

        self.poses = {}
        self._alias = {}
        for pid, p in (waypoints.get('poses') or {}).items():
            pid = str(pid)
            node = str(p['node'])
            if node not in self.nodes:
                raise SiteConfigError(
                    "pose '%s' refers to unknown node '%s'" % (pid, node))
            x, y = self.nodes[node]
            self.poses[pid] = Pose(
                pid, x, y, math.radians(float(p.get('yaw_deg', 0.0))),
                node, str(p.get('kind', '')))
            for key in [pid] + [str(a) for a in p.get('aliases') or []]:
                k = key.lower()
                if k in self._alias and self._alias[k] != pid:
                    raise SiteConfigError(
                        "name '%s' is used by '%s' and '%s'"
                        % (key, self._alias[k], pid))
                self._alias[k] = pid

        self._adj = {n: [] for n in self.nodes}
        for e in self.edges.values():
            self._adj[e.a].append(e)
            self._adj[e.b].append(e)

    # ----- destinations ----------------------------------------------------
    def names(self):
        """All names (ids and aliases) that resolve() accepts."""
        return sorted(self._alias)

    def resolve(self, name):
        pid = self._alias.get(str(name).strip().lower())
        if pid is None:
            raise SiteConfigError(
                "unknown destination '%s' (known: %s)"
                % (name, ', '.join(sorted(self.poses))))
        return self.poses[pid]

    # ----- zones ------------------------------------------------------------
    def zone_at(self, x, y):
        """First zone containing the point (file order = priority), or None."""
        for z in self.zones:
            if z.contains(x, y):
                return z.id
        return None

    # ----- route graph --------------------------------------------------------
    def shortest_path(self, start_node, goal_node, blocked_edges=()):
        """Dijkstra over straight corridor segments.

        Returns (node_ids, edge_ids, length_m) or None when unreachable.
        `blocked_edges`: edge ids that must not be used (blocked corridors).
        """
        for n in (start_node, goal_node):
            if n not in self.nodes:
                raise SiteConfigError("unknown node '%s'" % n)
        blocked = set(blocked_edges)
        dist = {start_node: 0.0}
        prev = {}
        heap = [(0.0, start_node)]
        while heap:
            d, u = heapq.heappop(heap)
            if u == goal_node:
                break
            if d > dist.get(u, math.inf):
                continue
            for e in self._adj[u]:
                if e.id in blocked:
                    continue
                v = e.other(u)
                nd = d + e.length_m
                if nd < dist.get(v, math.inf):
                    dist[v] = nd
                    prev[v] = (u, e.id)
                    heapq.heappush(heap, (nd, v))
        if goal_node not in dist:
            return None
        nodes, edges = [goal_node], []
        while nodes[-1] != start_node:
            u, eid = prev[nodes[-1]]
            nodes.append(u)
            edges.append(eid)
        nodes.reverse()
        edges.reverse()
        return nodes, edges, dist[goal_node]

    def nearest_node(self, x, y):
        """(node_id, distance_m) of the graph node closest to the point."""
        best = min(self.nodes, key=lambda n: math.hypot(
            self.nodes[n][0] - x, self.nodes[n][1] - y))
        bx, by = self.nodes[best]
        return best, math.hypot(bx - x, by - y)

    def route_legs(self, start_node, dest, blocked_edges=(),
                   straight_tol_deg=5.0):
        """Legs to drive from `start_node` to the destination pose `dest`.

        Every leg is a straight drive that ends at a node with a stop and an
        in-place turn. A node where the route continues in a straight line is
        not a leg of its own (no needless stop). The last leg ends with the
        destination heading. Returns a list of Leg, or None when unreachable.
        """
        found = self.shortest_path(start_node, dest.node, blocked_edges)
        if found is None:
            return None
        nodes, edge_ids = found[0], found[1]
        if len(nodes) == 1:
            return [Leg(dest.node, dest.x, dest.y, dest.yaw_rad, True,
                        dest.node, (dest.node,), ())]
        legs = []
        tol = math.radians(straight_tol_deg)
        seg = 0                  # index in `nodes` where the current leg starts
        for i in range(1, len(nodes)):
            x, y = self.nodes[nodes[i]]
            if i == len(nodes) - 1:
                legs.append(Leg(nodes[i], x, y, dest.yaw_rad, True,
                                nodes[seg], tuple(nodes[seg:i + 1]),
                                tuple(edge_ids[seg:i])))
                break
            px, py = self.nodes[nodes[i - 1]]
            nx, ny = self.nodes[nodes[i + 1]]
            arrive = math.atan2(y - py, x - px)
            leave = math.atan2(ny - y, nx - x)
            turn = abs(math.atan2(math.sin(leave - arrive),
                                  math.cos(leave - arrive)))
            if turn < tol:
                continue
            legs.append(Leg(nodes[i], x, y, leave, False,
                            nodes[seg], tuple(nodes[seg:i + 1]),
                            tuple(edge_ids[seg:i])))
            seg = i
        return legs


def load_site(config_dir=None):
    config_dir = config_dir or default_config_dir()
    return Site(
        _read_yaml(os.path.join(config_dir, 'entities.yaml')),
        _read_yaml(os.path.join(config_dir, 'waypoints.yaml')),
        _read_yaml(os.path.join(config_dir, 'zones.yaml')),
        config_dir,
        _revision(config_dir),
    )
