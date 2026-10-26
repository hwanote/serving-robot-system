"""Read-only site data: map image, semantic map and the route graph (config/common)."""
import heapq
import io
import math
import os

import yaml
from PIL import Image


def semantics_from_config(wp, ent, store_id, revision):
    """Semantic Map the editor shows for a waypoints.yaml + entities.yaml pair (no user zones:
    they are not in those files)."""
    nodes, poses = wp['nodes'], wp['poses']
    tables = (ent.get('tables') or {})
    counter = ((ent.get('fixtures') or {}).get('counter') or {}).get('center')
    entities, waypoints = [], []
    for pid, p in poses.items():
        node = nodes[p['node']]
        wpose = {'x': float(node['x']), 'y': float(node['y']),
                 'yaw': math.radians(float(p.get('yaw_deg', 0.0)))}
        kind = p.get('kind')
        num = pid.rsplit('_', 1)[-1]
        if kind == 'table_service':
            etype, label = 'table', '테이블 %d' % int(num)
            c = (tables.get(pid) or {}).get('center')
            epose = {'x': float(c['x']), 'y': float(c['y']), 'yaw': 0.0} if c else wpose
        elif kind == 'pickup':
            etype, label = 'kitchen', '주방(데스크)'
            epose = ({'x': float(counter['x']), 'y': float(counter['y']), 'yaw': 0.0}
                     if counter else wpose)
        elif kind == 'standby':
            etype, label, epose = 'robot_wait', '대기 위치 %d' % int(num), wpose
        else:
            continue
        item = {'id': pid, 'type': etype, 'label': label, 'pose': epose}
        if etype == 'table':
            # real values from entities.yaml (never a made-up default)
            top = (tables.get(pid) or {}).get('top_size_m')
            if top:
                item['top_size_m'] = {'x': float(top['x']), 'y': float(top['y'])}
            chairs = (tables.get(pid) or {}).get('chairs')
            if chairs:
                item['chairs'] = [{'dx': float(c['dx']), 'dy': float(c['dy']),
                                  'size_m': float(c['size_m'])} for c in chairs]
        entities.append(item)
        waypoints.append({'id': 'wp_' + pid, 'entity_id': pid, 'pose': wpose})
    return {'store_id': store_id, 'map_revision': revision,
            'entities': entities, 'waypoints': waypoints, 'zones': []}


class Site:
    def __init__(self, repo, store_id):
        self.store_id, self.repo = store_id, repo
        self._load_config()
        self.map_yaml_path = os.path.join(repo, 'maps', 'stores', store_id, 'map.yaml')
        self.map_yaml, self.width, self.height, self.png = self._read_map()
        self.revision = self._meta_revision() or 1
        self._rebuild()

    def _read_map(self):
        """map.yaml + image of the store -> (yaml dict, width, height, png bytes)."""
        with open(self.map_yaml_path, encoding='utf-8') as f:
            meta = yaml.safe_load(f)
        image = os.path.join(os.path.dirname(self.map_yaml_path), meta.get('image', 'map.pgm'))
        img = Image.open(image).convert('L')
        buf = io.BytesIO()
        img.save(buf, format='PNG')
        return meta, img.size[0], img.size[1], buf.getvalue()

    def _meta_revision(self):
        """map_revision of maps/stores/<store>/map_meta.yaml (None when missing or unreadable).
        The adoption of a new map writes it, so the number survives a restart of the backend."""
        try:
            with open(os.path.join(os.path.dirname(self.map_yaml_path), 'map_meta.yaml'),
                      encoding='utf-8') as f:
                v = (yaml.safe_load(f) or {}).get('map_revision')
            return int(v) if int(v) >= 1 else None
        except (OSError, ValueError, TypeError, AttributeError, yaml.YAMLError):
            return None

    def reload_map_image(self):
        """The store's map image was replaced (adoption of a new mapping): read map.yaml and the
        image again and raise the revision, so a Semantic Map drawn on the old map is refused.
        Config/common (nodes, tables) is not touched. A broken file leaves the old map in place."""
        self.map_yaml, self.width, self.height, self.png = self._read_map()
        meta = self._meta_revision()
        self.revision = meta if meta and meta > self.revision else self.revision + 1
        self.semantics = self._build_semantics()

    def _load_config(self):
        cfg = os.path.join(self.repo, 'config', 'common')
        with open(os.path.join(cfg, 'waypoints.yaml'), encoding='utf-8') as f:
            self.wp = yaml.safe_load(f)
        with open(os.path.join(cfg, 'entities.yaml'), encoding='utf-8') as f:
            self.ent = yaml.safe_load(f)

    def _rebuild(self):
        self.semantics = self._build_semantics()
        self.tables = [e['id'] for e in self.semantics['entities'] if e['type'] == 'table']
        self._build_graph()

    def reload(self):
        """Read config/common again (after the map was replaced). The map image stays.
        Everything is built on the side first, so a broken file leaves the old data in place."""
        old = (self.wp, self.ent, self.semantics, self.tables, self.nodes, self.edges, self._adj)
        try:
            self._load_config()
            self._rebuild()
        except Exception:
            (self.wp, self.ent, self.semantics, self.tables,
             self.nodes, self.edges, self._adj) = old
            raise

    # ---- route graph
    def _build_graph(self):
        self.nodes = {k: (float(v['x']), float(v['y'])) for k, v in self.wp['nodes'].items()}
        self.edges = {}
        self._adj = {k: [] for k in self.nodes}
        for e in self.wp.get('edges') or []:
            eid, a, b = str(e['id']), str(e['from']), str(e['to'])
            ax, ay = self.nodes[a]
            bx, by = self.nodes[b]
            self.edges[eid] = (a, b, math.hypot(bx - ax, by - ay))
            self._adj[a].append((eid, b))
            self._adj[b].append((eid, a))

    def node_of_pose(self, pose_name):
        return str(self.wp['poses'][pose_name]['node'])

    def node_xy(self, pose_name):
        n = self.wp['nodes'][self.wp['poses'][pose_name]['node']]
        return float(n['x']), float(n['y'])

    def nearest_node(self, x, y):
        return min(self.nodes, key=lambda n: math.hypot(self.nodes[n][0] - x,
                                                        self.nodes[n][1] - y))

    def path(self, start, goal):
        """Shortest path over the corridors -> (nodes, edge ids) or None. Same Dijkstra as
        the robot's Site.shortest_path, so the routes agree."""
        dist, prev, heap = {start: 0.0}, {}, [(0.0, start)]
        while heap:
            d, u = heapq.heappop(heap)
            if u == goal:
                break
            if d > dist.get(u, math.inf):
                continue
            for eid, v in self._adj[u]:
                nd = d + self.edges[eid][2]
                if nd < dist.get(v, math.inf):
                    dist[v] = nd
                    prev[v] = (u, eid)
                    heapq.heappush(heap, (nd, v))
        if goal not in dist:
            return None
        nodes, eids = [goal], []
        while nodes[-1] != start:
            u, eid = prev[nodes[-1]]
            nodes.append(u)
            eids.append(eid)
        return nodes[::-1], eids[::-1]

    def edge_between(self, a, b):
        for eid, v in self._adj.get(a, []):
            if v == b:
                return eid
        return None

    def directed(self, nodes):
        """[(edge id, from node, to node)] for consecutive nodes of a route."""
        out = []
        for a, b in zip(nodes, nodes[1:]):
            eid = self.edge_between(a, b)
            if eid and a != b:
                out.append((eid, a, b))
        return out

    def home_of(self, robot):
        """robot_02 -> standby_02 (the standby pose with the robot's number)."""
        guess = 'standby_%s' % robot.rsplit('_', 1)[-1]
        if guess in self.wp['poses']:
            return guess
        for pid, p in self.wp['poses'].items():
            if p.get('kind') == 'standby':
                return pid
        raise ValueError('no standby pose in waypoints.yaml')

    # ---- map / semantics for the UI
    def raw_map(self):
        ox, oy, oyaw = (list(self.map_yaml['origin']) + [0.0])[:3]
        return {
            'store_id': self.store_id, 'map_revision': self.revision,
            'image_url': '/api/stores/%s/map.png' % self.store_id,
            'resolution': float(self.map_yaml['resolution']),
            'width': self.width, 'height': self.height,
            'origin': {'x': float(ox), 'y': float(oy), 'yaw': float(oyaw)},
            'frame_id': 'map',
        }

    def _build_semantics(self):
        return semantics_from_config(self.wp, self.ent, self.store_id, self.revision)
