"""No ROS. Mimics route_executor well enough to try the backend and the frontend:
kitchen -> wait for load -> table -> wait for handover (-> home), along the route graph."""
import math
import threading
import time


class FakeAdapter:
    def __init__(self, robot, site, speed_mps=1.0, wait_s=2.0, fail_table=''):
        self.robot, self.site = robot, site
        self.speed, self.wait_s, self.fail = speed_mps, wait_s, fail_table
        self.home = site.home_of(robot)
        self.cmds, self.lock = [], threading.Lock()
        self.pos = list(site.node_xy(self.home)) + [0.0]
        self.mission_id = ''
        self.state, self.result, self.code = 'IDLE', '', ''
        self.stages, self.stage, self.table = [], 0, ''
        self.wait_until = 0.0
        self.wps = []                 # nodes still to pass on the current drive
        self.last_node = self.site.nearest_node(*self.pos[:2])
        self.sink = None
        self.running = False
        self.muted = False      # test hook: stop reporting (robot goes silent)
        self.deaf = False       # test hook: ignore commands (robot never acknowledges)
        self.received = []      # commands that reached the robot
        self.localization = 'OK'     # test hook: 'LOST' etc.
        self.config_revision = self._live_revision()   # the map it was started with
        self.map_image = self._live_map_image()        # the map picture it was started with

    def _live_revision(self):
        from ..promote import live_revision
        return live_revision(self.site.repo) or ''

    def _live_map_image(self):
        from ..promote import live_map_image
        return live_map_image(self.site.repo, self.site.store_id) or ''

    def start(self, sink):
        self.sink, self.running = sink, True
        threading.Thread(target=self._run, daemon=True).start()

    def stop(self):
        self.running = False

    def send_cmd(self, cmd):
        with self.lock:
            if self.deaf:
                return
            self.received.append(cmd)
            self.cmds.append(cmd)

    def _enter_drive(self):
        """Along the corridors to the stage's node (the fake robot follows the graph)."""
        self.last_node = self.site.nearest_node(*self.pos[:2])
        p = self.site.path(self.last_node, self.site.node_of_pose(self.stages[self.stage][1]))
        self.wps = p[0][1:] if p else []

    def _route(self):
        if self.state != 'DRIVING' or not self.stages:
            return '', []
        last = self.wps[-1] if self.wps else self.last_node
        route = list(self.wps)
        for kind, name in self.stages[self.stage + 1:]:
            if kind == 'DRIVING':
                goal = self.site.node_of_pose(name)
                p = self.site.path(last, goal)
                route += p[0][1:] if p else []
                last = goal
        nxt = self.site.edge_between(self.last_node, self.wps[0]) if self.wps else ''
        return nxt or '', route

    def _status(self):
        dest = ''
        if self.stages and self.stage < len(self.stages):
            dest = self.stages[self.stage][1]
        nxt, route = self._route()
        return {'state': self.state, 'mission_id': self.mission_id, 'destination': dest,
                'serve': bool(self.table), 'table': self.table, 'result': self.result,
                'code': self.code, 'leg': 1, 'legs': 1, 'localization': self.localization, 'loc_hold': '',
                'config_revision': self.config_revision, 'map_image': self.map_image,
                'returning': False, 'node': self.last_node, 'next_edge': nxt, 'route': route}

    def _handle(self, cmd):
        kind = cmd.get('cmd')
        if kind == 'serve':
            if self.state != 'IDLE':
                self.sink.on_alarm(self.robot, {'code': 'BUSY', 'level': 'warn',
                                                'text': 'busy', 'rejected_id': cmd.get('id')})
                return
            self.mission_id, self.table = cmd.get('id', ''), cmd['table']
            self.result = self.code = ''
            if self.table == self.fail:
                self.state, self.result, self.code = 'IDLE', 'FAILED', 'NO_ROUTE'
                return
            self.stages = [('DRIVING', 'kitchen'), ('WAIT_LOAD', 'kitchen'),
                           ('DRIVING', self.table), ('WAIT_HANDOVER', self.table)]
            if cmd.get('home', True) is not False:
                self.stages.append(('DRIVING', self.home))
            self.stage, self.state = 0, 'DRIVING'
            self._enter_drive()
        elif kind == 'go' and self.state == 'IDLE':
            self.mission_id, self.table = cmd.get('id', ''), ''
            self.result = self.code = ''
            self.stages = [('DRIVING', n) for n in cmd.get('names', [])]
            self.stage, self.state = 0, 'DRIVING'
            self._enter_drive()
        elif kind == 'reset':
            node = cmd.get('node', '')
            try:
                x, y = self.site.node_xy(node)
            except Exception:
                self.sink.on_alarm(self.robot, {'code': 'BAD_COMMAND', 'level': 'warn',
                                                'text': 'reset refused: %s' % node})
                return
            had = self.state != 'IDLE'
            self.pos[0], self.pos[1] = x, y
            self.last_node = self.site.nearest_node(x, y)
            self.state, self.stages, self.wps = 'IDLE', [], []
            self.localization = 'OK'
            if had:
                self.result, self.code = 'CANCELED', 'RESET'
        elif kind == 'reload_map':
            if self.state != 'IDLE':
                self.sink.on_alarm(self.robot, {'code': 'MAP_RELOAD_REFUSED', 'level': 'warn',
                                                'text': 'map not reloaded: the robot is not idle'})
                return
            self.config_revision = self._live_revision()
            self.sink.on_alarm(self.robot, {'code': 'MAP_RELOADED', 'level': 'info',
                                            'text': 'map reloaded',
                                            'config_revision': self.config_revision})
        elif kind == 'cancel' and self.state != 'IDLE':
            self.state, self.result, self.stages, self.wps = 'IDLE', 'CANCELED', [], []

    def _run(self):
        last_status = last_pose = 0.0
        t_prev = time.monotonic()
        while self.running:
            time.sleep(0.05)
            t = time.monotonic()
            dt, t_prev = t - t_prev, t
            with self.lock:
                cmds, self.cmds = self.cmds, []
            for c in cmds:
                self._handle(c)
            if self.state in ('DRIVING', 'WAIT_LOAD', 'WAIT_HANDOVER') \
                    and self.stage < len(self.stages):
                kind, _name = self.stages[self.stage]
                if kind == 'DRIVING' and not self.wps:
                    self._next_stage(t)
                elif kind == 'DRIVING':
                    tx, ty = self.site.nodes[self.wps[0]]
                    dx, dy = tx - self.pos[0], ty - self.pos[1]
                    dist = math.hypot(dx, dy)
                    step = self.speed * dt
                    if dist <= step:
                        self.pos[0], self.pos[1] = tx, ty
                        self.last_node = self.wps.pop(0)
                    else:
                        self.pos[0] += dx / dist * step
                        self.pos[1] += dy / dist * step
                        self.pos[2] = math.atan2(dy, dx)
                elif t >= self.wait_until:
                    self._next_stage(t)
            if self.muted:
                continue
            if t - last_pose >= 0.2:
                last_pose = t
                self.sink.on_pose(self.robot, *self.pos)
            if t - last_status >= 0.5:
                last_status = t
                self.sink.on_health(self.robot, {
                    'robot_state': 'IDLE' if self.state == 'IDLE' else 'NAVIGATING'})
                self.sink.on_status(self.robot, self._status())

    def _next_stage(self, t):
        self.stage += 1
        if self.stage >= len(self.stages):
            self.state, self.result, self.stages = 'IDLE', 'SUCCEEDED', []
            return
        kind = self.stages[self.stage][0]
        self.state = kind
        if kind == 'DRIVING':
            self._enter_drive()
        else:
            self.wait_until = t + self.wait_s
