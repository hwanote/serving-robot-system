"""Mission logic of the route executor (no ROS dependency).

The route executor drives the robot along the route graph of the site, one
straight leg after another (see site_config.Site.route_legs), and reacts when
an obstacle keeps the robot standing:

    stopped for alarm_after_s  (15 s)  -> alarm BLOCKED, the robot keeps waiting
    stopped for detour_after_s (30 s)  -> look for another straight corridor
                                          and drive there (DETOUR_STARTED)
    no other corridor                  -> alarm NO_DETOUR, keep waiting,
                                          look again every replan_interval_s
    too many detours                   -> alarm DETOUR_LIMIT, keep waiting

A corridor found blocked is avoided for `blocked_memory_s`. A detour makes the
robot drive straight back to the node it came from (it never reverses: it
turns in place first) and take another route from there. `request_detour()`
does the same at once, without waiting for the timers.

Straight rail: a leg along the route graph is sent as a straight path (from
where the robot is to the leg's node), not as a goal for the Nav2 planner, so
the robot never bends around an obstacle: it stops in front of it (person STOP,
stop zone) and only the detour rule above takes another corridor. If the robot
is more than off_rail_m away from that line it stops (OFF_RAIL alarm), waits
rail_resume_s and drives straight on from where it is. Direct goals (far from
every node) still go to the Nav2 planner.

Not moving (stall): while driving, a robot that has not moved stall_move_m nor
turned stall_turn_rad for stall_window_s counts as stopped too, whatever the
reason (no stop signal from any layer). So BLOCKED / detour also come when
something holds the robot without telling.

The core only decides. The ROS node feeds it the time (simulation clock), the
robot pose, the "stopped for an obstacle" flag of the soft stop limiter
and the collision latch, and carries out the returned actions:

    Action(SEND, leg=..., gen=n, start=(x, y))
                                   drive the leg: a straight path from `start`
                                   (start None: a Nav2 goal); report its result with gen
    Action(ALARM, code, text)      publish an alarm
    Action(ARRIVED, text=dest)     one destination of the mission is reached
    Action(CANCEL_NAV)             cancel the running Nav2 goal
    Action(DONE, result, code)     the mission has ended

Time spent while a collision is latched does not count towards the timers.

Localization hold: the node feeds the localization state of /robot_01/health
(on_localization). When it is LOST while driving, the robot stops at once
(CANCEL_NAV, alarm LOCALIZATION_LOST) and the mission is put on hold: no timer
runs, new orders are refused. Once the state has been out of LOST (OK or
DEGRADED) for loc_stable_s the alarm LOCALIZATION_RECOVERED asks the operator
to choose (a stopped robot cannot make AMCL more certain, so waiting for OK
could last forever; driving on does improve it): resume() drives on to
the same destination, go_home() cancels the mission (the node then drives to
home_name as a new mission). The core never moves again by itself. While
waiting at the kitchen / the table the robot stands still: only the alarm is
raised there. LOST / UNINITIALIZED also refuse new orders when idle.
"""
import math
from dataclasses import dataclass, field

from serving_robot_navigation.site_config import Leg, SiteConfigError

SEND = 'send'
ALARM = 'alarm'
ARRIVED = 'arrived'
CANCEL_NAV = 'cancel_nav'
DONE = 'done'

IDLE = 'IDLE'
DRIVING = 'DRIVING'
WAIT_LOAD = 'WAIT_LOAD'          # at the kitchen, waiting for the load
WAIT_HANDOVER = 'WAIT_HANDOVER'  # at the table, waiting for the customer
PAUSED = 'PAUSED'                # back at the kitchen after a failed delivery


@dataclass
class MissionParams:
    alarm_after_s: float = 15.0
    detour_after_s: float = 30.0
    blocked_memory_s: float = 60.0
    max_detours: int = 3
    replan_interval_s: float = 5.0
    node_reach_m: float = 0.35      # "at the node": no drive back needed
    max_node_dist_m: float = 0.6    # farther from every node: direct goal
    nav_retries: int = 2            # failed goals (not caused by an obstacle)
    # serving sequence
    kitchen_name: str = 'kitchen'
    home_name: str = 'standby_01'
    handover_timeout_s: float = 60.0  # then HANDOVER_TIMEOUT, keep waiting
    wait_repeat_s: float = 30.0       # repeat a waiting alarm this often
    auto_confirm_s: float = 0.0       # >0: confirm load/handover by itself (tests)
    delivery_limit_s: float = 180.0   # kitchen departure -> table arrival
    # not moving = stopped (0 = off; the ROS node sets them from runtime_limits)
    stall_window_s: float = 0.0
    stall_move_m: float = 0.05
    stall_turn_rad: float = 0.15
    # straight rail check (0 = off; the ROS node sets them from runtime_limits)
    off_rail_m: float = 0.0
    rail_resume_s: float = 3.0
    # localization OK for this long before the operator is asked to choose
    loc_stable_s: float = 2.0


@dataclass
class Action:
    kind: str
    leg: Leg = None
    gen: int = 0
    code: str = ''
    text: str = ''
    level: str = 'info'
    result: str = ''
    data: dict = field(default_factory=dict)
    start: tuple = None      # SEND: straight path from here to the leg node


def _seg_dist(px, py, ax, ay, bx, by):
    """Distance from a point to the segment a-b."""
    dx, dy = bx - ax, by - ay
    n = dx * dx + dy * dy
    t = 0.0 if n == 0.0 else max(0.0, min(1.0, ((px - ax) * dx + (py - ay) * dy) / n))
    return math.hypot(px - (ax + t * dx), py - (ay + t * dy))


class MissionCore:
    def __init__(self, site, params=None):
        self.site = site
        self.p = params or MissionParams()
        self.blocked = {}            # edge id -> time until it is avoided
        self.map_image = ''          # fingerprint of the map picture Nav2 loaded ('' = unknown)
        self.loc_state = None        # last localization state (None: unknown)
        self.loc_back_since = None   # since when it has been out of LOST (OK / DEGRADED)
        self._idle()

    # ----- state ------------------------------------------------------------
    def _idle(self):
        self.state = IDLE
        self.mission_id = ''
        self.queue = []
        self.dest = None
        self.legs = []
        self.leg_i = 0
        self.detours = 0
        self.stuck_s = 0.0
        self.alarmed = False
        self.no_detour_alarmed = False
        self.limit_alarmed = False
        self.detour_requested = False
        self.last_step = None
        self.last_replan = None
        self.retries_left = self.p.nav_retries
        self.gen = 0
        self._hold = False
        self._latched = False
        self.serve = False
        self.table = ''
        self.wait_since = 0.0
        self.wait_alarm_t = 0.0
        self.timeout_alarmed = False
        self.timing = False
        self.delivery_s = 0.0
        self.returning = False
        self.stuck_alarmed = False
        self.rail = None             # ((x0, y0), (x1, y1)) of the running leg
        self.rail_pause_until = None
        self.off_rail_count = 0
        self._stalled = False
        self.anchor = None           # (x, y, yaw, time) of the last movement
        self.loc_hold = ''           # '' | 'LOST' | 'RECOVERED' (waiting for the choice)
        self.loc_alarm_t = 0.0

    def status(self):
        return {
            'state': self.state,
            'mission_id': self.mission_id,
            'destination': self.dest.name if self.dest else '',
            'leg': self.leg_i + 1 if self.legs else 0,
            'legs': len(self.legs),
            'stopped_s': round(self.stuck_s, 1),
            'detours': self.detours,
            'avoiding': sorted(self.blocked),
            'serve': self.serve,
            'stalled': self._stalled,
            'localization': self.loc_state or '',
            'loc_hold': self.loc_hold,
            'config_revision': getattr(self.site, 'revision', ''),
            'map_image': self.map_image,
            'delivery_s': round(self.delivery_s, 1),
            'returning': self.returning,
            'table': self.table,
            'wait_s': round(self.last_step - self.wait_since, 1)
            if self.state in (WAIT_LOAD, WAIT_HANDOVER) and self.last_step is not None else 0.0,
            **self._route_info(),
        }

    def _route_info(self):
        """Where the robot is on the route graph (additive status fields).

        node: graph node the robot is at / last left ('' = unknown or off the
              graph). next_edge: corridor being driven ('' = none).
        route: nodes still ahead (next node first) up to the end of the whole
              order, queued destinations included. Empty when idle, waiting
              or driving a direct goal (no rails).
        """
        xy = getattr(self, '_last_xy', None)
        node, next_edge, route = '', '', []
        if self.state == DRIVING and self.legs:
            leg = self.legs[self.leg_i]
            if leg.edges and xy is not None:
                i = self._edge_index(leg, xy)
                node, next_edge = leg.path_nodes[i], leg.edges[i]
                route = list(leg.path_nodes[i + 1:])
                for later in self.legs[self.leg_i + 1:]:
                    route += list(later.path_nodes[1:]) if later.path_nodes \
                        else [later.node]
                last = route[-1] if route else leg.node
                for pose in self.queue:
                    sp = self.site.shortest_path(last, pose.node, self.blocked)
                    if sp is None:
                        break
                    route += sp[0][1:]
                    last = pose.node
        elif self.state in (WAIT_LOAD, WAIT_HANDOVER) and self.dest:
            node = self.dest.node
        elif xy is not None:
            near, dist = self.site.nearest_node(*xy)
            if dist <= self.p.max_node_dist_m:
                node = near
        return {'node': node, 'next_edge': next_edge, 'route': route}

    def _avoided(self, now):
        for e in [e for e, until in self.blocked.items() if until <= now]:
            del self.blocked[e]
        return set(self.blocked)

    # ----- commands -----------------------------------------------------------
    def start(self, now, mission_id, names, xy):
        """Begin a mission over the destination names, in order."""
        if self.state != IDLE:
            return self._busy(mission_id)
        if self.loc_refuses_orders():
            return self._not_localized(mission_id)
        try:
            poses = [self.site.resolve(n) for n in names]
        except SiteConfigError as e:
            return [Action(DONE, result='FAILED', code='BAD_DESTINATION',
                           text=str(e))]
        if not poses:
            return [Action(DONE, result='FAILED', code='BAD_DESTINATION',
                           text='no destination given')]
        self._idle()
        self.state = DRIVING
        self.mission_id = mission_id
        self.queue = poses
        self.last_step = now
        return self._begin_destination(now, xy, None)

    def _busy(self, rejected_id=''):
        """Refuse a new order (or a cancel). `rejected_id`: the order that was
        refused, so its sender can tell the refusal is meant for it."""
        data = {'rejected_id': rejected_id} if rejected_id else {}
        if self.loc_hold:
            return [Action(ALARM, code='LOCALIZATION_HOLD', level='warn',
                           text='new orders are refused: mission %s is on hold '
                                'after a localization loss (%s)'
                                % (self.mission_id, 'waiting for the position'
                                   if self.loc_hold == 'LOST'
                                   else 'operator must choose resume or home'),
                           data=data)]
        if self.state == PAUSED:
            return [Action(ALARM, code='HANDOVER_REQUIRED', level='warn',
                           text='new orders are held: staff must confirm the '
                                'handover of the failed delivery to %s first'
                           % self.table, data=data)]
        return [Action(ALARM, code='BUSY', level='warn',
                       text='a mission is already running (%s)' % self.mission_id,
                       data=data)]

    def loc_refuses_orders(self):
        """The robot does not know where it is: it takes no new order."""
        return self.loc_state in ('LOST', 'UNINITIALIZED')

    def _not_localized(self, rejected_id=''):
        data = {'rejected_id': rejected_id} if rejected_id else {}
        why = ('lost its position' if self.loc_state == 'LOST'
               else 'has no position yet')
        return [Action(ALARM, code='LOCALIZATION_%s' % self.loc_state,
                       level='warn',
                       text='new order refused: the robot %s (localization %s)'
                            % (why, self.loc_state), data=data)]

    def start_serve(self, now, mission_id, table, xy, home=True):
        """Serve one table: kitchen (load) -> table (handover) -> home.
        `home=False`: the mission ends at the table once the order is taken
        (the robot stays there, IDLE, for the next order)."""
        if self.state != IDLE:
            return self._busy(mission_id)
        if self.loc_refuses_orders():
            return self._not_localized(mission_id)
        try:
            if table in (self.p.kitchen_name, self.p.home_name):
                raise SiteConfigError('%r is not a table' % table)
            poses = [self.site.resolve(n) for n in
                     (self.p.kitchen_name, table)
                     + ((self.p.home_name,) if home else ())]
        except SiteConfigError as e:
            return [Action(DONE, result='FAILED', code='BAD_DESTINATION',
                           text=str(e))]
        self._idle()
        self.state = DRIVING
        self.mission_id = mission_id
        self.serve = True
        self.table = table
        self.queue = poses
        self.last_step = now
        return self._begin_destination(now, xy, None)

    def confirm_load(self, now, xy):
        """The load is on the robot: drive to the table. -> (ok, message, actions)"""
        if self.state != WAIT_LOAD:
            return False, 'the robot is not waiting for a load', []
        self.state = DRIVING
        self.last_step = now
        return True, 'load confirmed', self._begin_destination(
            now, xy, self.dest.node)

    def confirm_handover(self, now, xy):
        """The customer (or staff) took the load. -> (ok, message, actions)"""
        if self.state == WAIT_HANDOVER:
            if not self.queue:               # served without a way home
                return True, 'handover confirmed', self._finish('SUCCEEDED')
            self.state = DRIVING
            self.last_step = now
            return True, 'handover confirmed', self._begin_destination(
                now, xy, self.dest.node)
        if self.state == PAUSED:
            self._idle()
            return True, 'handover confirmed: new orders are accepted', [
                Action(ALARM, code='HANDOVER_CONFIRMED', level='info',
                       text='staff took over the failed delivery: ready for '
                            'new orders')]
        return False, 'the robot is not waiting for a handover', []

    def cancel(self, mission_id=None):
        """Cancel the running mission. With `mission_id` only that mission is
        canceled: a sender never cancels somebody else's mission by mistake."""
        if self.state == IDLE:
            return []
        if mission_id and mission_id != self.mission_id:
            return [Action(ALARM, code='BAD_COMMAND', level='warn',
                           text='cancel ignored: %s is not the running mission '
                                '(%s)' % (mission_id, self.mission_id))]
        if self.state == PAUSED:
            return self._busy()
        mission_id = self.mission_id
        self._idle()
        return [Action(CANCEL_NAV),
                Action(DONE, result='CANCELED', text=mission_id)]

    def request_detour(self):
        """Take another corridor now. Returns (ok, message)."""
        if self.state != DRIVING:
            return False, 'no mission is running'
        if self.loc_hold:
            return False, 'the mission is on hold (localization)'
        self.detour_requested = True
        return True, 'detour requested'

    # ----- localization hold -------------------------------------------------------
    def on_localization(self, now, state):
        """Feed the localization state ('OK' 'DEGRADED' 'LOST' 'UNINITIALIZED',
        None = unknown: no fresh health message). Call as often as you like.
        Out of LOST (OK or DEGRADED) for loc_stable_s ends the wait for the
        position; the operator then chooses."""
        prev, self.loc_state = self.loc_state, state
        if state in ('OK', 'DEGRADED'):      # a position, even a rough one
            if self.loc_back_since is None:
                self.loc_back_since = now
        else:                                # LOST, UNINITIALIZED, unknown
            self.loc_back_since = None
        if state != 'LOST':
            return []
        if self.state == DRIVING and self.loc_hold != 'LOST':
            return self._enter_loc_hold(now)
        if prev != 'LOST' and self.state in (WAIT_LOAD, WAIT_HANDOVER):
            # standing still already: nothing to stop, tell the operator
            return [Action(ALARM, code='LOCALIZATION_LOST', level='warn',
                           text='localization lost while waiting at %s: the '
                                'robot stays; check it before confirming'
                                % self.dest.name,
                           data={'destination': self.dest.name})]
        return []

    def _enter_loc_hold(self, now):
        again = self.loc_hold == 'RECOVERED'
        self.loc_hold = 'LOST'
        self.loc_alarm_t = now
        self.gen += 1                    # the canceled goal's result is ignored
        self.rail = None
        self.rail_pause_until = None
        self.anchor = None
        self._hold = self._stalled = False
        return [Action(CANCEL_NAV),
                Action(ALARM, code='LOCALIZATION_LOST', level='warn',
                       text='localization %s: stopped, mission %s to %s is on '
                            'hold. When the position is back you choose: '
                            'resume or home'
                            % ('lost again' if again else 'lost',
                               self.mission_id, self.dest.name),
                       data={'destination': self.dest.name})]

    def _recovered_alarm(self):
        rough = self.loc_state == 'DEGRADED'
        return Action(
            ALARM, code='LOCALIZATION_RECOVERED', level='warn',
            text='the robot knows its position again%s: choose "resume" (drive '
                 'on to %s) or "home" (cancel mission %s and drive to %s)'
                 % (' (still rough: DEGRADED, it gets better once the robot '
                    'drives)' if rough else '',
                    self.dest.name, self.mission_id, self.p.home_name),
            data={'destination': self.dest.name, 'home': self.p.home_name,
                  'choices': ['resume', 'home'],
                  'localization': self.loc_state or ''})

    def _loc_step(self, now):
        """Mission on hold: wait for the position, then for the operator."""
        self.last_step = now
        self.anchor = None
        if self.loc_hold == 'LOST':
            if (self.loc_back_since is not None
                    and now - self.loc_back_since >= self.p.loc_stable_s):
                self.loc_hold = 'RECOVERED'
                self.loc_alarm_t = now
                return [self._recovered_alarm()]
            if now - self.loc_alarm_t >= self.p.wait_repeat_s:
                self.loc_alarm_t = now
                return [Action(ALARM, code='LOCALIZATION_LOST', level='warn',
                               text='localization still lost: mission %s on hold'
                                    % self.mission_id,
                               data={'destination': self.dest.name})]
            return []
        if now - self.loc_alarm_t >= self.p.wait_repeat_s:
            self.loc_alarm_t = now
            return [self._recovered_alarm()]       # reminder
        return []

    def _loc_choice_error(self):
        if self.loc_hold == 'LOST':
            return 'localization is not OK yet: wait for LOCALIZATION_RECOVERED'
        return 'no mission is waiting for a localization decision'

    def resume(self, now, xy):
        """Operator choice 1: drive on to the same destination.
        -> (ok, message, actions)"""
        if self.loc_hold != 'RECOVERED':
            return False, self._loc_choice_error(), []
        self.loc_hold = ''
        self.last_step = now
        name = self.dest.name
        if self._near_current_leg(xy):
            # still on its corridor: the same leg again, straight from here
            self.stuck_s = 0.0
            self.alarmed = self.no_detour_alarmed = self.limit_alarmed = False
            self.detour_requested = False
            self.last_replan = None
            self.retries_left = self.p.nav_retries
            acts = [self._send(xy)]
        else:
            # the position jumped: plan again from where the robot is now (the
            # delivery time already used stays)
            used = self.delivery_s
            self.queue.insert(0, self.dest)
            acts = self._begin_destination(now, xy, None)
            self.delivery_s = used
        return True, 'resumed', [
            Action(ALARM, code='MISSION_RESUMED', level='info',
                   text='operator chose resume: driving on to %s' % name)] + acts

    def _near_current_leg(self, xy):
        """The robot is within max_node_dist_m of the corridor of its leg."""
        leg = self.legs[self.leg_i] if self.legs else None
        if leg is None or not leg.edges or xy is None:
            return False
        i = self._edge_index(leg, xy)
        ax, ay = self.site.nodes[leg.path_nodes[i]]
        bx, by = self.site.nodes[leg.path_nodes[i + 1]]
        return _seg_dist(xy[0], xy[1], ax, ay, bx, by) <= self.p.max_node_dist_m

    def go_home(self, now):
        """Operator choice 2: cancel the mission. The caller then starts the
        drive home (start(..., [home_name])). -> (ok, mission_id, actions)"""
        if self.loc_hold != 'RECOVERED':
            return False, self._loc_choice_error(), []
        mission_id = self.mission_id
        self._idle()
        return True, mission_id, [
            Action(CANCEL_NAV),
            Action(ALARM, code='GO_HOME', level='info',
                   text='operator chose home: mission %s canceled, driving to %s'
                        % (mission_id, self.p.home_name)),
            Action(DONE, result='CANCELED', code='LOCALIZATION_LOST',
                   text=mission_id)]

    def reset(self, now, node_name):
        """Operator emergency reset: the robot was put by hand on `node_name`.
        Whatever the state (driving, stuck, waiting, held), the mission and the Nav2
        goal are dropped and every timer / hold / flag is cleared: back to IDLE.
        The caller gives the node's pose to AMCL. The localization state is NOT
        cleared here: new orders stay refused until the health message says the
        position is not LOST again. -> (mission_id, actions)"""
        mission_id = self.mission_id
        had_mission = self.state != IDLE
        was = self.state
        self._idle()
        out = [Action(CANCEL_NAV),
               Action(ALARM, code='ROBOT_RESET', level='info',
                      text='operator reset: robot placed on %s%s'
                           % (node_name, ' (mission %s canceled, was %s)'
                              % (mission_id, was) if had_mission else ''))]
        if had_mission:
            out.append(Action(DONE, result='CANCELED', code='RESET', text=mission_id))
        return mission_id, out

    def replace_site(self, site):
        """Operator asked the robot to read config/common again. Only while IDLE and
        not waiting for a localization decision (a map change under a running
        mission would change its legs). The caller has already loaded and checked
        `site`. Edges the old map had are forgotten (ids may mean something else).
        -> (ok, message)"""
        if self.state != IDLE or self.loc_hold:
            return False, 'map not reloaded: the robot is not idle (%s)' % (
                self.loc_hold or self.state)
        self.site = site
        self.blocked = {}
        return True, 'map reloaded'

    # ----- planning -------------------------------------------------------------
    def _begin_destination(self, now, xy, start_node):
        self.dest = self.queue.pop(0)
        self.detours = 0
        self.stuck_s = 0.0
        self.alarmed = self.no_detour_alarmed = self.limit_alarmed = False
        self.detour_requested = False
        self.last_replan = None
        self.retries_left = self.p.nav_retries
        self.timing = (self.serve and not self.returning
                       and self.dest.name == self.table)
        self.delivery_s = 0.0
        self.stuck_alarmed = False
        self.anchor = None               # stall timing starts afresh
        if start_node is None:
            start_node, dist = self.site.nearest_node(*xy)
            if dist > self.p.max_node_dist_m:
                d = self.dest
                self.legs = [Leg(d.node, d.x, d.y, d.yaw_rad, True)]
                self.leg_i = 0
                return [Action(ALARM, code='DIRECT_GOAL', level='info',
                               text='robot is %.2f m from the nearest route '
                                    'node %s: direct goal to %s (no detours)'
                                    % (dist, start_node, d.name)),
                        self._send(xy)]
        legs = self.site.route_legs(start_node, self.dest, self._avoided(now))
        if legs is None:
            return self._finish('FAILED', 'NO_ROUTE',
                                'no route from node %s to %s'
                                % (start_node, self.dest.name))
        self.legs = legs
        self.leg_i = 0
        return [self._send(xy)]

    def _send(self, xy):
        """Drive the current leg. Along the route graph: a straight rail from
        the robot position `xy`; a direct goal (no from_node): a Nav2 goal."""
        self.gen += 1
        leg = self.legs[self.leg_i]
        start = None
        if leg.from_node and xy is not None:
            start = (float(xy[0]), float(xy[1]))
            self.rail = (start, (leg.x, leg.y))
        else:
            self.rail = None
        self.rail_pause_until = None
        return Action(SEND, leg=leg, gen=self.gen, start=start)

    def _finish(self, result, code='', text=''):
        self._idle()
        return [Action(DONE, result=result, code=code, text=text)]

    # ----- Nav2 results -----------------------------------------------------------
    def on_nav_result(self, now, gen, ok, xy):
        """A Nav2 goal ended. Goals that were replaced (old `gen`) are ignored."""
        if self.state != DRIVING or gen != self.gen or self.loc_hold:
            return []
        if not ok:
            if self._hold or self._stalled or self._latched:
                return [self._send(xy)]       # waiting for an obstacle: try again
            if self.retries_left > 0:
                self.retries_left -= 1
                return [self._send(xy)]
            return self._finish('FAILED', 'NAV_FAILED',
                                'Nav2 could not reach node %s'
                                % self.legs[self.leg_i].node)
        self.retries_left = self.p.nav_retries
        self.leg_i += 1
        if self.leg_i < len(self.legs):
            return [self._send(xy)]
        reached = self.dest
        out = [Action(ARRIVED, text=reached.name)]
        if self.serve:
            return out + self._serve_arrived(now, reached)
        if self.queue:
            return out + self._begin_destination(now, xy, reached.node)
        return out + self._finish('SUCCEEDED')

    def _serve_arrived(self, now, reached):
        self.timing = False
        if self.returning:
            return self._pause(now)
        if reached.name == self.p.kitchen_name:
            self.state, kind, text = WAIT_LOAD, 'WAIT_LOAD', (
                'at the kitchen: waiting for the load for %s' % self.table)
        elif reached.name == self.table:
            self.state, kind, text = WAIT_HANDOVER, 'WAIT_HANDOVER', (
                'at %s: waiting for the customer to take the order' % self.table)
        else:
            return self._finish('SUCCEEDED')
        self.wait_since = self.wait_alarm_t = now
        self.timeout_alarmed = False
        return [Action(ALARM, code=kind, level='info', text=text,
                       data={'destination': self.table})]

    def _pause(self, now):
        """Back at the kitchen after a failed delivery: hold new orders."""
        table, mid = self.table, self.mission_id
        self._idle()
        self.state, self.mission_id, self.table = PAUSED, mid, table
        self.serve = True
        return [Action(ALARM, code='RETURNED_TO_KITCHEN', level='warn',
                       text='delivery to %s failed: the robot is back at the '
                            'kitchen. New orders are held until staff confirm '
                            'the handover' % table,
                       data={'destination': table}),
                Action(DONE, result='RETURNED', code='DELIVERY_OVERTIME',
                       text='back at the kitchen, handover required')]

    def _overtime_return(self, now, xy):
        self.returning = True
        self.timing = False
        self.queue = [self.site.resolve(self.p.kitchen_name)]
        out = [Action(CANCEL_NAV),
               Action(ALARM, code='DELIVERY_OVERTIME', level='warn',
                      text='delivery to %s took more than %.0f s: returning to '
                           'the kitchen' % (self.table, self.p.delivery_limit_s),
                      data={'destination': self.table}),
               Action(ALARM, code='RETURN_STARTED', level='info',
                      text='driving back to the kitchen',
                      data={'destination': self.table})]
        return out + self._begin_destination(now, xy, None)

    def _wait_step(self, now, xy):
        """Waiting at the kitchen / the table for a confirmation."""
        self.last_step = now
        waited = now - self.wait_since
        loading = self.state == WAIT_LOAD
        if self.p.auto_confirm_s > 0 and waited >= self.p.auto_confirm_s:
            fn = self.confirm_load if loading else self.confirm_handover
            return fn(now, xy)[2]
        if loading:
            if now - self.wait_alarm_t >= self.p.wait_repeat_s:
                self.wait_alarm_t = now
                return [Action(ALARM, code='WAIT_LOAD', level='info',
                               text='still waiting for the load for %s (%.0f s)'
                                    % (self.table, waited),
                               data={'destination': self.table})]
            return []
        due = (not self.timeout_alarmed
               or now - self.wait_alarm_t >= self.p.wait_repeat_s)
        if waited >= self.p.handover_timeout_s and due:
            self.timeout_alarmed = True
            self.wait_alarm_t = now
            return [Action(ALARM, code='HANDOVER_TIMEOUT', level='warn',
                           text='nobody took the order at %s for %.0f s: '
                                'still waiting' % (self.table, waited),
                           data={'destination': self.table})]
        return []

    # ----- periodic -------------------------------------------------------------------
    def _edge_index(self, leg, xy):
        """Index of the corridor of `leg` the robot is on (nearest; ahead on ties)."""
        best, best_d = 0, math.inf
        for i, _eid in enumerate(leg.edges):
            ax, ay = self.site.nodes[leg.path_nodes[i]]
            bx, by = self.site.nodes[leg.path_nodes[i + 1]]
            d = _seg_dist(xy[0], xy[1], ax, ay, bx, by)
            if d <= best_d + 1e-9:
                best, best_d = i, d
        return best

    def _where(self, xy):
        leg = self.legs[self.leg_i] if self.legs else None
        if leg is not None and leg.edges:
            return self.site.edges[leg.edges[self._edge_index(leg, xy)]].zone
        return self.site.zone_at(*xy) or ''

    def _track_motion(self, now, xy, yaw):
        """True when the robot has not moved stall_move_m nor turned
        stall_turn_rad for stall_window_s (off when stall_window_s is 0)."""
        if self.p.stall_window_s <= 0.0:
            return False
        a = self.anchor
        if a is None:
            self.anchor = (xy[0], xy[1], yaw, now)
            return False
        moved = math.hypot(xy[0] - a[0], xy[1] - a[1]) >= self.p.stall_move_m
        turned = (yaw is not None and a[2] is not None and abs(math.atan2(
            math.sin(yaw - a[2]), math.cos(yaw - a[2]))) >= self.p.stall_turn_rad)
        if moved or turned:
            self.anchor = (xy[0], xy[1], yaw, now)
            return False
        return now - a[3] >= self.p.stall_window_s

    def _check_rail(self, now, xy):
        """Stop when the robot left the straight line of its leg; drive on
        straight from where it is after rail_resume_s."""
        if self.rail_pause_until is not None:
            if now < self.rail_pause_until:
                return []
            self.rail_pause_until = None
            self.anchor = None
            return [self._send(xy)]
        if self.p.off_rail_m <= 0.0 or self.rail is None:
            return []
        (ax, ay), (bx, by) = self.rail
        off = _seg_dist(xy[0], xy[1], ax, ay, bx, by)
        if off <= self.p.off_rail_m:
            return []
        self.off_rail_count += 1
        self.rail = None
        self.gen += 1                    # the canceled goal's result is ignored
        self.rail_pause_until = now + self.p.rail_resume_s
        return [Action(CANCEL_NAV),
                Action(ALARM, code='OFF_RAIL', level='warn',
                       text='%.2f m off the straight line to node %s in %s: '
                            'stopped, driving on straight from here in %.0f s'
                            % (off, self.legs[self.leg_i].node, self._where(xy),
                               self.p.rail_resume_s),
                       data={'off_m': round(off, 2), 'x': round(xy[0], 2),
                             'y': round(xy[1], 2)})]

    def step(self, now, xy, hold, latched, yaw=None):
        """Call regularly. `hold`: a stop signal is on (obstacle / person).
        `yaw` (rad, optional): turning in place counts as moving."""
        self._last_xy = xy
        if self.state in (WAIT_LOAD, WAIT_HANDOVER):
            self.anchor = None
            return self._wait_step(now, xy)
        if self.state != DRIVING:
            self.last_step = now
            self.anchor = None
            return []
        if self.loc_hold:                   # on hold: no timer runs
            return self._loc_step(now)
        dt = 0.0 if self.last_step is None else min(max(now - self.last_step, 0.0), 1.0)
        self.last_step = now
        stalled = self._track_motion(now, xy, yaw) and not latched
        self._hold, self._stalled, self._latched = hold, stalled, latched
        blocked = hold or stalled           # stopped, whatever the reason
        self._avoided(now)
        out = []
        if not latched:
            out += self._check_rail(now, xy)
        if self.timing:
            if not latched:                 # a collision stop does not count
                self.delivery_s += dt
            if self.delivery_s >= self.p.delivery_limit_s:
                if not blocked and not latched:
                    return out + self._overtime_return(now, xy)
                if not self.stuck_alarmed:
                    self.stuck_alarmed = True
                    out.append(Action(
                        ALARM, code='DELIVERY_OVERTIME_STUCK', level='warn',
                        text='delivery to %s took more than %.0f s and the '
                             'robot cannot move (%s) at %s'
                             % (self.table, self.p.delivery_limit_s,
                                'collision' if latched else
                                'obstacle' if hold else 'not moving',
                                self._where(xy)),
                        data={'destination': self.table,
                              'x': round(xy[0], 2), 'y': round(xy[1], 2)}))
        if latched:
            return out                      # a collision stop is not an obstacle wait
        if blocked:
            if stalled and not hold and self.stuck_s == 0.0:
                # count from the moment it stopped moving, not from the detection
                self.stuck_s = now - self.anchor[3]
            else:
                self.stuck_s += dt
        else:
            self.stuck_s = 0.0
            self.alarmed = self.no_detour_alarmed = self.limit_alarmed = False
        if blocked and self.stuck_s >= self.p.alarm_after_s and not self.alarmed:
            self.alarmed = True
            why = ('by an obstacle' if hold else
                   'without moving (no stop signal)')
            out.append(Action(
                ALARM, code='BLOCKED', level='warn',
                text='stopped for %.0f s %s in %s on the way to %s'
                     % (self.stuck_s, why, self._where(xy), self.dest.name),
                data={'cause': 'obstacle' if hold else 'not_moving'}))
        wants = self.detour_requested or (blocked and self.stuck_s >= self.p.detour_after_s)
        due = (self.detour_requested or self.last_replan is None
               or now - self.last_replan >= self.p.replan_interval_s)
        if wants and due:
            out += self._detour(now, xy)
        return out

    def _detour(self, now, xy):
        self.last_replan = now
        requested, self.detour_requested = self.detour_requested, False
        leg = self.legs[self.leg_i]
        if not leg.edges:
            if not self.no_detour_alarmed:
                self.no_detour_alarmed = True
                return [Action(ALARM, code='NO_DETOUR', level='warn',
                               text='no detour possible: the robot is not '
                                    'following the route graph')]
            return []
        if self.detours >= self.p.max_detours:
            if not self.limit_alarmed:
                self.limit_alarmed = True
                return [Action(ALARM, code='DETOUR_LIMIT', level='warn',
                               text='%d detours already used for %s: waiting'
                                    % (self.detours, self.dest.name))]
            return []
        idx = self._edge_index(leg, xy)
        edge = leg.edges[idx]
        behind = leg.path_nodes[idx]         # node the robot passed last
        plan = self.site.route_legs(behind, self.dest,
                                    self._avoided(now) | {edge})
        if plan is None:
            if not self.no_detour_alarmed:
                self.no_detour_alarmed = True
                return [Action(ALARM, code='NO_DETOUR', level='warn',
                               text='no other corridor to %s: waiting at %s'
                                    % (self.dest.name,
                                       self.site.edges[edge].zone))]
            return []

        self.blocked[edge] = now + self.p.blocked_memory_s
        self.detours += 1
        legs = plan
        bx, by = self.site.nodes[behind]
        if (behind != self.dest.node
                and math.hypot(xy[0] - bx, xy[1] - by) > self.p.node_reach_m):
            nx, ny = self.site.nodes[plan[0].path_nodes[1]]
            back = Leg(behind, bx, by, math.atan2(ny - by, nx - bx), False,
                       behind, (behind,), ())
            legs = [back] + plan
        self.legs = legs
        self.leg_i = 0
        self.stuck_s = 0.0
        self.anchor = None               # stall timing starts afresh
        self.alarmed = self.no_detour_alarmed = self.limit_alarmed = False
        self.retries_left = self.p.nav_retries
        return [Action(ALARM, code='DETOUR_STARTED', level='warn',
                       text='%s blocked (%s): detour %d/%d via %s'
                            % (self.site.edges[edge].zone,
                               'requested' if requested else 'stopped too long',
                               self.detours, self.p.max_detours,
                               ' -> '.join(l.node for l in legs))),
                self._send(xy)]
