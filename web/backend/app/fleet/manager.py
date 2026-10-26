"""FleetManager: order queue, dispatcher and robot bookkeeping.

The adapters call on_status / on_health / on_pose / on_alarm from their own threads;
everything is guarded by one lock. The dispatcher gives the oldest queued task to a free
robot when
  - a robot is IDLE (the order's zone owner first, see `zones`),
  - at least `stagger_s` seconds passed since the previous order was sent,
  - no other robot is on its way to the kitchen or waiting there (one kitchen counter), and
  - the order's route touches no graph node another robot holds (traffic.find_clash).
A robot ends an order at the table (serve "home": false) and stays there; it goes home
after `idle_home_s` seconds without work and with an empty queue.
"""
import math
import threading
import time

from ..util import now_iso
from .traffic import find_clash, plan_order

TERMINAL = ('SUCCEEDED', 'FAILED', 'CANCELED')
RESULT_TO_STATUS = {'SUCCEEDED': 'SUCCEEDED', 'FAILED': 'FAILED', 'CANCELED': 'CANCELED',
                    'RETURNED': 'FAILED'}


def _secs(a, b):
    from datetime import datetime
    f = lambda x: datetime.strptime(x, '%Y-%m-%dT%H:%M:%S.%fZ')
    return (f(b) - f(a)).total_seconds()


def _stats(v):
    if not v:
        return {'count': 0, 'avg': None, 'max': None}
    return {'count': len(v), 'avg': round(sum(v) / len(v), 1), 'max': round(max(v), 1)}


class FleetManager:
    def __init__(self, robots, site, repo, hub, settings):
        self.robots = {r.name: r for r in robots}
        self.site, self.repo, self.hub, self.cfg = site, repo, hub, settings
        self.kitchen = settings.kitchen
        self.kitchen_node = site.node_of_pose(self.kitchen)
        self.kitchen_xy = site.node_xy(self.kitchen)
        self.lock = threading.RLock()
        self.queue = []                 # task ids waiting for a robot, oldest first
        self.queued_at = {}
        self.last_dispatch = -1e9
        self.home_count = 0
        self.live_revision = lambda: None     # set by the app: revision of config/common
        self.live_map_image = lambda: None    # set by the app: fingerprint of the store's map picture
        self.wait_reason = ''
        self.log = []
        self.dispatch_paused = False
        self.mapping_active = lambda: False   # set by the app: True while the robot is mapping

    def is_busy(self):
        """True while an order is waiting or a robot is driving one (or going home)."""
        with self.lock:
            return bool(self.queue) or any(r.active or r.homing for r in self.robots.values())

    def refresh_site(self):
        """The map was replaced: take the kitchen and the robots' home again."""
        with self.lock:
            self.kitchen_node = self.site.node_of_pose(self.kitchen)
            self.kitchen_xy = self.site.node_xy(self.kitchen)
            for r in self.robots.values():
                r.home = self.site.home_of(r.name)

    # ---- helpers
    def note(self, text):
        line = '%s %s' % (now_iso(), text)
        self.log = (self.log + [line])[-200:]
        print(line, flush=True)

    @staticmethod
    def task_event(t):
        return {'type': 'task_state', 'task_id': t['task_id'], 'status': t['status'],
                'robot_id': t['robot_id'], 'destination_id': t['destination_id'],
                'updated_at': t['updated_at']}

    def _set_status(self, t, status):
        if t['status'] == status:
            return
        t['status'], t['updated_at'] = status, now_iso()
        if status == 'ASSIGNED':
            t['assigned_at'] = t['updated_at']
        elif status == 'RUNNING':
            t.setdefault('started_at', t['updated_at'])
        elif status in TERMINAL:
            t['finished_at'] = t['updated_at']
        self.repo.save_task(t)
        self.repo.add_event('task_status', {'task_id': t['task_id'], 'status': status,
                                            'robot_id': t['robot_id'], 'at': t['updated_at']})
        self.note('task %s -> %s (%s)' % (t['task_id'], status, t['robot_id'] or '-'))
        self.hub.emit(self.task_event(t))

    def robot_state_name(self, r):
        """PROVISIONAL mapping robot mission state -> the frontend's robot state."""
        ms, h = r.ms, r.health
        if r.offline:
            return 'ERROR'                # the frontend has no OFFLINE state; see `online`
        if h and h.get('robot_state') == 'ERROR':
            return 'ERROR'
        if not ms:
            return 'IDLE'
        st = ms.get('state')
        if st == 'PAUSED' or ms.get('loc_hold'):
            return 'PAUSED'
        if st == 'WAIT_HANDOVER':
            return 'SERVING'
        if st == 'WAIT_LOAD':
            return 'NAVIGATING'
        if st == 'DRIVING':
            dest = ms.get('destination')
            if ms.get('returning') or dest == r.home:
                return 'RETURNING'
            return 'NAVIGATING' if dest == self.kitchen else 'SERVING'
        return 'IDLE'

    def robot_event(self, r):
        if r.pose is None:
            return None
        t = self.repo.get_task(r.active) if r.active else None
        return {'type': 'robot_state', 'robot_id': r.name,
                'state': self.robot_state_name(r),
                'task_id': t['task_id'] if t else None,
                'destination_id': t['destination_id'] if t else None,
                'battery_percent': 100,
                'online': not r.offline, 'accepting_orders': not r.paused,
                'localization': (r.ms or {}).get('localization') or None,
                'map_current': self._map_current(r), 'controls': self._controls(r),
                'map_image_current': self._map_image_current(r),
                'pose': {'x': round(r.pose[0], 3), 'y': round(r.pose[1], 3),
                         'yaw': round(r.pose[2], 4)}}

    # ---- what the robot card's buttons may do (the frontend only shows this)
    def _map_current(self, r):
        """True: the robot runs the map that is in config/common now. None: unknown
        (an older robot node does not report it)."""
        have, live = (r.ms or {}).get('config_revision'), self.live_revision()
        if not have or not live:
            return None
        return have == live

    def _map_image_current(self, r):
        """True: the robot loaded the map picture the store has now. False: the picture changed
        after the robot started (it must be restarted; "read the map again" cannot fix it).
        None: unknown (an older robot node does not report it)."""
        have, live = (r.ms or {}).get('map_image'), self.live_map_image()
        if not have or not live:
            return None
        return have == live

    def _idle(self, r):
        ms = r.ms or {}
        return (ms.get('state') == 'IDLE' and not ms.get('loc_hold')
                and r.active is None and not r.homing)

    def _controls(self, r):
        ms, online = r.ms or {}, not r.offline
        home_ok = ms.get('loc_hold') == 'RECOVERED' or (
            self._idle(r) and ms.get('localization') == 'OK' and not self._at_home(r))
        return {'reset': online, 'reset_nodes': [r.home, self.kitchen],
                'home': online and home_ok,
                'reload_map': online and self._idle(r) and self._map_current(r) is not True}

    def snapshot(self):
        with self.lock:
            events = [self.task_event(t) for t in self.repo.recent_tasks(20)]
            robots = [e for e in (self.robot_event(r) for r in self.robots.values()) if e]
            return robots + events

    def tick(self):
        """Every 0.5 s: run the dispatcher, return the robots' current state events."""
        with self.lock:
            self._watch()
            self._dispatch()
            return [e for e in (self.robot_event(r) for r in self.robots.values()) if e]

    # ---- robot -> backend (called from the adapters' threads)
    def on_status(self, name, ms):
        with self.lock:
            r = self.robots[name]
            self._seen(r)
            r.ms = ms
            self._track_idle(r, ms)
            t = self.repo.get_task(r.active) if r.active else None
            if t and ms.get('mission_id') == t['task_id']:
                st, res = ms.get('state'), ms.get('result', '')
                if st in ('DRIVING', 'WAIT_LOAD', 'WAIT_HANDOVER'):
                    self._set_status(t, 'RUNNING')
                elif st == 'IDLE' and res:
                    self._finish(r, t, RESULT_TO_STATUS.get(res, 'FAILED'), ms)
                elif st == 'PAUSED':
                    self._finish(r, t, 'FAILED', ms)
            self._dispatch()

    def on_health(self, name, h):
        with self.lock:
            self._seen(self.robots[name])
            self.robots[name].health = h

    def on_pose(self, name, x, y, yaw):
        with self.lock:
            self._seen(self.robots[name])
            self.robots[name].pose = (x, y, yaw)

    def on_alarm(self, name, a):
        with self.lock:
            r = self.robots[name]
            self._seen(r)
            self.note('alarm %s %s %s %s' % (name, a.get('level', ''), a.get('code', ''),
                                              a.get('text', '')))
            self.repo.add_event('alarm', {'robot_id': name, 'alarm': a, 'at': now_iso()})
            t = self.repo.get_task(r.active) if r.active else None
            if t and a.get('rejected_id') == t['task_id']:
                self._finish(r, t, 'FAILED', a)
            self._dispatch()

    def _finish(self, r, t, status, info):
        t['detail'] = info.get('code') or info.get('text') or ''
        self._set_status(t, status)
        self.repo.save_task(t)
        if r.active == t['task_id']:
            r.active = None

    # ---- going home when there is nothing to do
    def _at_home(self, r):
        return (r.pose is not None
                and self.site.nearest_node(*r.pose[:2]) == self.site.node_of_pose(r.home))

    def _track_idle(self, r, ms):
        now = time.monotonic()
        if r.homing:
            if ms.get('mission_id') == r.homing and ms.get('state') == 'IDLE':
                r.homing = None                       # arrived (or ended)
            elif ms.get('mission_id') != r.homing and now - r.homing_at > 5.0:
                r.homing = None                       # the robot never took it
        idle = (ms.get('state') == 'IDLE' and r.active is None and not r.homing
                and not self._at_home(r))
        if not idle:
            r.idle_since = None
        elif r.idle_since is None:
            r.idle_since = now

    def _idle_home(self):
        if self.queue or self.cfg.idle_home_s < 0:
            return
        now = time.monotonic()
        for r in self.robots.values():
            if r.idle_since is not None and now - r.idle_since >= self.cfg.idle_home_s \
                    and r.active is None and not r.homing and not r.offline \
                    and (r.ms or {}).get('localization') == 'OK':
                # (a robot that does not know where it is refuses 'go' anyway: mapping stack,
                #  or right after a new map; sending it only made the fleet look busy)
                self.home_count += 1
                r.homing, r.homing_at, r.idle_since = 'home-%04d' % self.home_count, now, None
                self.note('%s idle for %.0f s: going home (%s)'
                          % (r.name, self.cfg.idle_home_s, r.home))
                r.adapter.send_cmd({'cmd': 'go', 'id': r.homing, 'names': [r.home]})

    # ---- dispatcher
    @staticmethod
    def _ready(r):
        """The robot's own summary says it can drive: not still booting, not in ERROR."""
        h = r.health or {}
        return h.get('robot_state') not in ('BOOTING', 'ERROR')

    def _head(self):
        """The queued order to serve next: highest table priority, then the oldest."""
        prio = self.cfg.priorities
        best = min(enumerate(self.queue), key=lambda p: (
            -prio.get(self.repo.get_task(p[1])['destination_id'], 0), p[0]))
        return best[1]

    def _kitchen_busy(self, r):
        """True while robot r is on its way to the kitchen or waiting there."""
        if not r.active:
            return False
        ms = r.ms or {}
        if ms.get('mission_id') != r.active:
            return True                  # just assigned, the robot has not reported yet
        st = ms.get('state')
        if st == 'WAIT_LOAD':
            return True
        return (st == 'DRIVING' and ms.get('destination') == self.kitchen
                and not ms.get('returning'))

    def _dispatch(self):
        if self.mapping_active():
            return                     # the robot is mapping: no idle-home, no dispatch
        self._idle_home()
        if not self.queue:
            return
        cfg = self.cfg
        if time.monotonic() - self.last_dispatch < cfg.stagger_s:
            return
        if cfg.kitchen_lock and any(self._kitchen_busy(r) for r in self.robots.values()):
            return
        if self.dispatch_paused:
            return
        free = [r for r in self.robots.values()
                if r.active is None and not r.homing and r.pose is not None and r.ms
                and r.ms.get('state') == 'IDLE' and not r.offline and not r.paused
                and self._ready(r) and self._map_image_current(r) is not False]
        if not free:
            stale = [r.name for r in self.robots.values() if self._map_image_current(r) is False]
            if stale and self.queue:
                reason = 'orders wait: %s must be restarted (the map picture changed)' % ', '.join(stale)
                if reason != self.wait_reason:
                    self.wait_reason = reason
                    self.note(reason)
            return
        free.sort(key=lambda x: math.hypot(x.pose[0] - self.kitchen_xy[0],
                                           x.pose[1] - self.kitchen_xy[1]))
        head = self._head()
        t = self.repo.get_task(head)
        owner = cfg.zones.get(t['destination_id'])
        waited = time.monotonic() - self.queued_at.get(t['task_id'], time.monotonic())
        cands = [c for c in free if c.name == owner]
        if not owner or waited >= cfg.help_after_s:
            cands += [c for c in free if c.name != owner]
        r, plan, why = None, None, []
        for cand in cands:
            cplan = plan_order(self.site, self.kitchen, cand, t['destination_id'])
            clash = (find_clash(self.site, self.kitchen_node, cand, cplan,
                                list(self.robots.values())) if cfg.route_check else None)
            if clash is None:
                r, plan = cand, cplan
                break
            why.append('%s: %s' % (cand.name, clash))
        if r is None:
            if not cands:
                why.append('%s is busy, help after %.0f s' % (owner, cfg.help_after_s))
            reason = '%s waits: %s' % (t['task_id'], '; '.join(why))
            if reason != self.wait_reason:
                self.wait_reason = reason
                self.note(reason)
            return
        self.wait_reason = ''
        if owner and r.name != owner:
            self.note('%s: %s helps (owner %s, waited %.0f s)'
                      % (t['task_id'], r.name, owner, waited))
        self.queue.remove(head)
        r.plan = plan
        r.active = t['task_id']
        r.assigned_at = time.monotonic()
        t['robot_id'] = r.name
        self.last_dispatch = time.monotonic()
        self._set_status(t, 'ASSIGNED')
        cmd = {'cmd': 'serve', 'table': t['destination_id'], 'id': t['task_id']}
        if not cfg.serve_home:
            cmd['home'] = False
        r.adapter.send_cmd(cmd)

    # ---- frontend -> backend
    def create_task(self, destination_id, client_request_id=None):
        """-> (task, created). The same client_request_id returns the first task again."""
        with self.lock:
            if client_request_id:
                old = self.repo.find_by_request_id(client_request_id)
                if old:
                    return old, False
            tid = self.repo.next_task_id()
            ts = now_iso()
            t = {'task_id': tid, 'type': 'SERVE', 'destination_id': destination_id,
                 'status': 'QUEUED', 'robot_id': None, 'created_at': ts, 'updated_at': ts}
            if client_request_id:
                t['client_request_id'] = client_request_id
            self.repo.save_task(t)
            self.queue.append(tid)
            self.queued_at[tid] = time.monotonic()
            self.note('task %s created: serve %s' % (tid, destination_id))
            self.repo.add_event('task_status', {'task_id': tid, 'status': 'QUEUED',
                                                'robot_id': None, 'at': ts})
            self.hub.emit(self.task_event(t))
            self._dispatch()
            return dict(self.repo.get_task(tid)), True

    def cancel_task(self, task_id):
        """-> (task|None, problem|None); problem is 'not found' or 'finished'."""
        with self.lock:
            t = self.repo.get_task(task_id)
            if not t:
                return None, 'not found'
            if t['status'] in TERMINAL:
                return dict(t), 'finished'
            if task_id in self.queue:
                self.queue.remove(task_id)
            else:
                for r in self.robots.values():
                    if r.active == task_id:
                        r.adapter.send_cmd({'cmd': 'cancel', 'id': task_id})
                        r.active = None
            self._set_status(t, 'CANCELED')
            return dict(t), None

    # ---- operations
    def _seen(self, r):
        r.last_seen = time.monotonic()
        if r.offline:
            r.offline, r.offline_since = False, None
            self.note('%s is back online' % r.name)
            self.repo.add_event('robot_online', {'robot_id': r.name, 'at': now_iso()})

    def _watch(self):
        """Heartbeat and acknowledgement checks (every tick)."""
        now = time.monotonic()
        for r in self.robots.values():
            if r.last_seen is None:
                continue
            if not r.offline and now - r.last_seen > self.cfg.offline_after_s:
                r.offline, r.offline_since = True, now
                self.note('%s OFFLINE (no message for %.0f s)' % (r.name, now - r.last_seen))
                self.repo.add_event('robot_offline', {'robot_id': r.name, 'at': now_iso()})
            self._expire_homing(r, now)
            t = self.repo.get_task(r.active) if r.active else None
            if not t:
                continue
            if r.offline and now - r.offline_since > self.cfg.offline_fail_s:
                self._fail_active(r, t, 'ROBOT_OFFLINE')
            elif (t['status'] == 'ASSIGNED' and not r.offline
                  and now - r.assigned_at > self.cfg.ack_timeout_s):
                self._fail_active(r, t, 'COMMAND_NOT_ACKED')

    def _expire_homing(self, r, now):
        """Let go of a 'going home' mark that can no longer be finished. The mark used to
        end only when a status message arrived, so a robot that never took the command (or
        went silent) kept the whole fleet 'busy' (map adoption refused) until a restart."""
        if not r.homing:
            return
        if r.offline:
            self.note('%s offline: forgetting the home trip %s' % (r.name, r.homing))
            r.homing = None
        elif (r.ms or {}).get('mission_id') != r.homing and now - (r.homing_at or now) > 5.0:
            r.homing = None                       # the robot never took it

    def _fail_active(self, r, t, code):
        self.note('%s: task %s failed (%s)' % (r.name, t['task_id'], code))
        r.adapter.send_cmd({'cmd': 'cancel', 'id': t['task_id']})     # best effort
        self._finish(r, t, 'FAILED', {'code': code})

    def set_dispatch_paused(self, value):
        with self.lock:
            self.dispatch_paused = bool(value)
            self.note('dispatch %s' % ('paused' if value else 'resumed'))

    def set_robot_paused(self, name, value):
        """-> robot summary or None. A paused robot finishes its order but gets no new one."""
        with self.lock:
            r = self.robots.get(name)
            if not r:
                return None
            r.paused = bool(value)
            self.note('%s %s' % (name, 'paused (no new orders)' if value else 'resumed'))
            return self.robot_summary(r)

    def robot_command(self, name, action):
        """Operator choice after a localization loss: 'resume' or 'home'. -> (summary, problem)"""
        with self.lock:
            r = self.robots.get(name)
            if not r:
                return None, 'not found'
            if r.offline:
                return self.robot_summary(r), 'offline'
            ms = r.ms or {}
            if ms.get('loc_hold') != 'RECOVERED':
                return self.robot_summary(r), 'not recovered'
            r.adapter.send_cmd({'cmd': action})
            self.note('%s: operator chose %s' % (name, action))
            return self.robot_summary(r), None

    def reset_robot(self, name, node=None):
        """Emergency reset: the robot was put on `node` by hand (default: its home).
        An order it was driving is canceled. -> (summary, problem)"""
        with self.lock:
            r = self.robots.get(name)
            if not r:
                return None, 'not found'
            if r.offline:
                return self.robot_summary(r), 'offline'
            node = node or r.home
            if node not in (self.site.wp.get('poses') or {}):
                return self.robot_summary(r), 'bad node'
            t = self.repo.get_task(r.active) if r.active else None
            r.adapter.send_cmd({'cmd': 'reset', 'node': node})
            r.homing, r.idle_since = None, None
            if t:
                self._finish(r, t, 'CANCELED', {'code': 'RESET'})
            self.note('%s: operator reset on %s' % (name, node))
            return self.robot_summary(r), None

    def go_home(self, name):
        """Home button. After a localization loss the robot's own "home" choice; when it is
        idle, a drive home, but only if it knows where it is (reset first when it does not).
        -> (summary, problem)"""
        with self.lock:
            r = self.robots.get(name)
            if not r:
                return None, 'not found'
            if r.offline:
                return self.robot_summary(r), 'offline'
            ms = r.ms or {}
            if ms.get('loc_hold') == 'RECOVERED':
                r.adapter.send_cmd({'cmd': 'home'})
                self.note('%s: operator chose home' % name)
                return self.robot_summary(r), None
            if ms.get('loc_hold'):
                return self.robot_summary(r), 'not localized'
            if not self._idle(r):
                return self.robot_summary(r), 'busy'
            if ms.get('localization') != 'OK':
                return self.robot_summary(r), 'not localized'
            if self._at_home(r):
                return self.robot_summary(r), 'at home'
            self.home_count += 1
            r.homing, r.homing_at, r.idle_since = 'home-%04d' % self.home_count, time.monotonic(), None
            self.note('%s: operator chose home (%s)' % (name, r.home))
            r.adapter.send_cmd({'cmd': 'go', 'id': r.homing, 'names': [r.home]})
            return self.robot_summary(r), None

    def reload_map(self, name):
        """Tell one robot to read config/common again. It does this only while idle.
        -> (summary, problem)"""
        with self.lock:
            r = self.robots.get(name)
            if not r:
                return None, 'not found'
            if r.offline:
                return self.robot_summary(r), 'offline'
            if not self._idle(r):
                return self.robot_summary(r), 'busy'
            r.adapter.send_cmd({'cmd': 'reload_map'})
            self.note('%s: reading the map again' % name)
            return self.robot_summary(r), None

    def reload_map_all(self):
        """-> {robot_id: 'sent' | 'offline' | 'busy' | 'current'} (a robot already on the
        newest map is skipped)."""
        out = {}
        with self.lock:
            names = list(self.robots)
        for name in names:
            with self.lock:
                cur = self._map_current(self.robots[name])
            if cur is True:
                out[name] = 'current'
                continue
            _, problem = self.reload_map(name)
            out[name] = problem or 'sent'
        return out

    def robot_summary(self, r):
        e = self.robot_event(r) or {'type': 'robot_state', 'robot_id': r.name,
                                    'state': self.robot_state_name(r)}
        e = dict(e, online=not r.offline, accepting_orders=not r.paused, home=r.home,
                 mission_state=(r.ms or {}).get('state'), loc_hold=(r.ms or {}).get('loc_hold'))
        return e

    def list_robots(self):
        with self.lock:
            return [self.robot_summary(r) for r in self.robots.values()]

    def metrics(self):
        """Operating figures computed from the task history (see /api/metrics)."""
        with self.lock:
            tasks = self.repo.list_tasks()
            by = {}
            for t in tasks:
                by[t['status']] = by.get(t['status'], 0) + 1
            waits, services, per_robot = [], [], {}
            first = last = None
            for t in tasks:
                if t.get('assigned_at'):
                    waits.append(_secs(t['created_at'], t['assigned_at']))
                if t['status'] == 'SUCCEEDED' and t.get('assigned_at') and t.get('finished_at'):
                    services.append(_secs(t['assigned_at'], t['finished_at']))
                    per_robot[t['robot_id']] = per_robot.get(t['robot_id'], 0) + 1
                    last = max(last or t['finished_at'], t['finished_at'])
                    first = min(first or t['created_at'], t['created_at'])
            span_h = _secs(first, last) / 3600.0 if first and last else 0.0
            done = len(services)
            return {
                'generated_at': now_iso(), 'mode': self.cfg.mode,
                'tasks': {'total': len(tasks), 'by_status': by, 'queued_now': len(self.queue)},
                'wait_s': _stats(waits), 'service_s': _stats(services),
                'throughput_per_hour': round(done / span_h, 1) if span_h > 0 else None,
                'completed_by_robot': per_robot,
                'robots': {'total': len(self.robots),
                           'offline': sum(1 for r in self.robots.values() if r.offline),
                           'paused': sum(1 for r in self.robots.values() if r.paused)}}

    def recover(self):
        """After a restart: queued orders go back to the queue; orders that had been handed
        to a robot are FAILED (SERVER_RESTART) because their outcome is unknown."""
        with self.lock:
            for t in self.repo.unfinished_tasks():
                if t['status'] == 'QUEUED':
                    self.queue.append(t['task_id'])
                    self.queued_at[t['task_id']] = time.monotonic()
                    self.note('recovered queued task %s' % t['task_id'])
                else:
                    t['detail'] = 'SERVER_RESTART'
                    self._set_status(t, 'FAILED')

    def state_snapshot(self, n=50):
        """Everything a client needs to resync after a WebSocket reconnect."""
        with self.lock:
            return {'generated_at': now_iso(),
                    'robots': [e for e in (self.robot_event(r) for r in self.robots.values())
                               if e],
                    'tasks': self.repo.list_tasks()[:n],
                    'queue': list(self.queue)}

    def list_tasks(self):
        with self.lock:
            return self.repo.list_tasks()

    def debug(self):
        with self.lock:
            return {'robots': {n: {'mission_status': r.ms, 'health': r.health,
                                   'pose': r.pose, 'active': r.active, 'home': r.home}
                               for n, r in self.robots.items()},
                    'queue': list(self.queue), 'waiting': self.wait_reason,
                    'dispatch_paused': self.dispatch_paused,
                    'log': self.log[-30:]}
