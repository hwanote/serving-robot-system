"""Scenario tests for the mission core (route executor logic, no ROS needed).

    python3 tests/robot/test_mission.py
"""
import math
import os
import sys
import tempfile

import yaml

sys.path.insert(0, os.path.join(
    os.path.dirname(__file__), '..', '..', 'ros_ws', 'src',
    'serving_robot_navigation'))
from serving_robot_navigation.command.mission import (  # noqa: E402
    ALARM, ARRIVED, CANCEL_NAV, DONE, DRIVING, IDLE, PAUSED, SEND, WAIT_HANDOVER,
    WAIT_LOAD,
    MissionCore, MissionParams)
from serving_robot_navigation.command.rail import rail_poses  # noqa: E402
from serving_robot_navigation.site_config import load_site  # noqa: E402

REPO_CONFIG = os.path.join(os.path.dirname(__file__), '..', '..', 'config', 'common')
W_S = (-3.20, -2.40)
MID_WEST_S = (-3.20, -1.40)      # half way on corridor e_west_s (W_S -> W_M)


def make_site(nodes, edges, poses):
    d = tempfile.mkdtemp()
    for name, data in (
            ('entities.yaml', {'schema_version': 1}),
            ('zones.yaml', {'schema_version': 1, 'zones': [{'id': 'z', 'rect': {
                'x_min': -100, 'x_max': 100, 'y_min': -100, 'y_max': 100}}]}),
            ('waypoints.yaml', {
                'schema_version': 1, 'frame_id': 'map',
                'nodes': {k: {'x': v[0], 'y': v[1]} for k, v in nodes.items()},
                'edges': [{'id': '%s_%s' % (a, b), 'from': a, 'to': b, 'zone': 'z'}
                          for a, b in edges],
                'poses': poses})):
        with open(os.path.join(d, name), 'w') as f:
            yaml.safe_dump(data, f)
    return load_site(d)


class Run:
    """Drives a MissionCore like the ROS node does (time, position, flags)."""

    def __init__(self, site, xy, params=None):
        self.core = MissionCore(site, params)
        self.t = 0.0
        self.xy = xy
        self.hold = False
        self.latched = False
        self.yaw = None
        self.log = []
        self.gen = 0

    def _rec(self, acts):
        for a in acts:
            if a.kind == SEND:
                self.gen = a.gen
        self.log += acts
        return acts

    def start(self, names, mission_id='m1'):
        return self._rec(self.core.start(self.t, mission_id, names, self.xy))

    def advance(self, seconds, dt=0.1):
        out = []
        for _ in range(int(round(seconds / dt))):
            self.t += dt
            out += self._rec(self.core.step(self.t, self.xy, self.hold, self.latched,
                                            yaw=self.yaw))
        return out

    def nav(self, ok=True, gen=None):
        return self._rec(self.core.on_nav_result(
            self.t, self.gen if gen is None else gen, ok, self.xy))

    def sends(self, acts=None):
        return [a for a in (self.log if acts is None else acts) if a.kind == SEND]

    @staticmethod
    def codes(acts):
        return [a.code for a in acts if a.kind == ALARM]


def real():
    return load_site(REPO_CONFIG)


def mid_run(names=('table_01',)):
    """Mission started at W_S; the robot is now half way on corridor e_west_s."""
    r = Run(real(), W_S)
    r.start(list(names))
    r.xy = MID_WEST_S
    return r


def test_normal_trip():
    r = Run(real(), W_S)
    acts = r.start(['table_01'])
    assert [a.leg.node for a in acts if a.kind == SEND] == ['W_M']
    acts = r.nav()
    assert [a.leg.node for a in acts if a.kind == SEND] == ['T13']
    acts = r.nav()
    assert [a.kind for a in acts] == [ARRIVED, DONE]
    assert acts[-1].result == 'SUCCEEDED' and r.core.state == IDLE


def test_alarm_at_15s_once_and_no_detour_before_30s():
    r = mid_run()
    r.hold = True
    assert r.codes(r.advance(14.8)) == []
    assert r.codes(r.advance(0.6)) == ['BLOCKED']
    acts = r.advance(14.0)                     # up to ~29.4 s
    assert acts == [], 'no second alarm, no detour before 30 s'


def test_detour_at_30s_drives_back_and_avoids_corridor():
    r = mid_run()
    r.hold = True
    r.advance(29.8)
    acts = r.advance(0.6)
    assert r.codes(acts) == ['DETOUR_STARTED']
    send = r.sends(acts)[0]
    assert send.leg.node == 'W_S', 'first drive back to the node it came from'
    assert abs(send.leg.yaw_rad - 0.0) < 1e-6, 'and face the new route (east)'
    assert 'e_west_s' in r.core.blocked
    used = [e for leg in r.core.legs for e in leg.edges]
    assert 'e_west_s' not in used and 'e_south' in used
    # the robot is driving again: finish the trip
    r.hold = False
    r.xy = W_S
    while r.core.state == DRIVING:
        r.nav()
    assert r.log[-1].kind == DONE and r.log[-1].result == 'SUCCEEDED'


def test_obstacle_leaves_before_30s_no_detour_and_new_episode_restarts():
    r = mid_run()
    r.hold = True
    r.advance(20.0)
    r.hold = False
    r.advance(10.0)
    assert 'DETOUR_STARTED' not in r.codes(r.log)
    r.hold = True
    assert r.codes(r.advance(14.0)) == []
    assert r.codes(r.advance(1.5)) == ['BLOCKED'], 'new episode counts from 0'


def _blocked_by(r, obstacle, person):
    """The route executor passes `obstacle_hold or person STOP` as one signal."""
    r.hold = bool(obstacle or person)


def test_person_to_obstacle_handover_keeps_the_timer():
    r = mid_run()
    _blocked_by(r, False, True)                 # a person blocks the way
    assert r.codes(r.advance(10.0)) == []
    _blocked_by(r, True, False)                 # the person leaves, a box still blocks
    assert r.codes(r.advance(4.8)) == []
    assert r.codes(r.advance(0.6)) == ['BLOCKED'], 'counted from the first stop'
    assert r.codes(r.advance(14.0)) == []
    assert r.codes(r.advance(1.0)) == ['DETOUR_STARTED'], '30 s in total'


def test_obstacle_to_person_handover_keeps_the_timer():
    r = mid_run()
    _blocked_by(r, True, False)                 # a box blocks the way
    assert r.codes(r.advance(10.0)) == []
    _blocked_by(r, False, True)                 # the box is gone, a person stands there
    assert r.codes(r.advance(4.8)) == []
    assert r.codes(r.advance(0.6)) == ['BLOCKED']
    assert r.codes(r.advance(15.0)) == ['DETOUR_STARTED']


def test_a_gap_between_the_two_signals_starts_a_new_wait():
    # Not smoothed on purpose: a short gap (both signals off) restarts the wait,
    # which is the safe side (the robot waits longer before it detours).
    r = mid_run()
    _blocked_by(r, False, True)
    r.advance(25.0)
    _blocked_by(r, False, False)
    r.advance(0.2)
    _blocked_by(r, True, False)
    assert 'DETOUR_STARTED' not in r.codes(r.advance(10.0))
    assert r.core.stuck_s < 11.0


def test_instant_detour_request():
    r = mid_run()
    r.advance(1.0)                             # driving, not stopped
    ok, _ = r.core.request_detour()
    assert ok
    acts = r.advance(0.2)
    assert r.codes(acts) == ['DETOUR_STARTED']
    assert 'e_west_s' in r.core.blocked


def test_detour_request_without_mission_is_refused():
    r = Run(real(), W_S)
    ok, msg = r.core.request_detour()
    assert not ok and 'no mission' in msg


def test_blocked_right_at_the_start_node_needs_no_drive_back():
    r = Run(real(), W_S)
    r.start(['table_01'])
    r.hold = True
    acts = r.advance(30.5)
    assert 'DETOUR_STARTED' in r.codes(acts)
    first = r.sends(acts)[0]
    assert first.leg.node == 'C_S', 'straight on to the next node of the new route'


def test_no_other_corridor_waits_with_one_alarm():
    site = make_site(
        {'A': (0, 0), 'B': (2, 0), 'C': (4, 0)}, [('A', 'B'), ('B', 'C')],
        {'dest': {'node': 'C', 'yaw_deg': 0.0, 'kind': 'standby'}})
    r = Run(site, (3.0, 0.0))
    r.start(['dest'])
    r.hold = True
    acts = r.advance(60.0)
    codes = r.codes(acts)
    assert codes.count('BLOCKED') == 1
    assert codes.count('NO_DETOUR') == 1
    assert 'DETOUR_STARTED' not in codes
    assert r.core.blocked == {}, 'nothing to avoid when there is no way around'


def test_detour_limit():
    nodes = {'A': (0, 0), 'B': (10, 0)}
    edges, poses = [], {'dest': {'node': 'B', 'yaw_deg': 0.0, 'kind': 'standby'}}
    for i, y in enumerate((2, 4, 6, 8), 1):
        nodes['M%d' % i] = (5, y)
        edges += [('A', 'M%d' % i), ('M%d' % i, 'B')]
    r = Run(make_site(nodes, edges, poses), (0.0, 0.0))
    r.start(['dest'])
    r.hold = True
    acts = r.advance(31.0 * 4 + 10.0)
    codes = r.codes(acts)
    assert codes.count('DETOUR_STARTED') == 3
    assert codes.count('DETOUR_LIMIT') == 1
    assert r.core.detours == 3


def test_collision_latch_time_does_not_count():
    r = mid_run()
    r.hold = True
    r.latched = True
    assert r.advance(60.0) == []
    r.latched = False
    assert r.codes(r.advance(14.0)) == []
    assert r.codes(r.advance(1.5)) == ['BLOCKED']


def test_nav_failure_handling():
    r = Run(real(), W_S)
    r.start(['table_01'])
    r.hold = True
    r.advance(1.0)
    for _ in range(5):                         # waiting for an obstacle: just retry
        acts = r.nav(ok=False)
        assert [a.kind for a in acts] == [SEND]
    r.hold = False
    r.advance(0.2)
    assert [a.kind for a in r.nav(ok=False)] == [SEND]       # retry 1
    assert [a.kind for a in r.nav(ok=False)] == [SEND]       # retry 2
    acts = r.nav(ok=False)                                   # out of retries
    assert acts[-1].kind == DONE and acts[-1].code == 'NAV_FAILED'
    assert r.core.state == IDLE


def test_result_of_a_replaced_goal_is_ignored():
    r = mid_run()
    old = r.gen
    r.hold = True
    r.advance(31.0)
    assert r.gen != old
    assert r.nav(ok=False, gen=old) == []
    assert r.nav(ok=True, gen=old) == []


def test_direct_goal_far_from_nodes_has_no_detour():
    r = Run(real(), (0.8, -1.4))
    acts = r.start(['table_01'])
    assert r.codes(acts) == ['DIRECT_GOAL']
    assert r.sends(acts)[0].leg.node == 'T13'
    r.hold = True
    codes = r.codes(r.advance(31.0))
    assert 'BLOCKED' in codes and 'NO_DETOUR' in codes


def test_several_destinations_in_a_row():
    r = Run(real(), W_S)
    r.start(['kitchen', 'table_01'])
    nodes = []
    while r.core.state == DRIVING:
        nodes += [a.leg.node for a in r.sends(r.nav())]
        r.xy = W_S
    kinds = [a.kind for a in r.log]
    assert kinds.count(ARRIVED) == 2 and r.log[-1].result == 'SUCCEEDED'
    assert [a.text for a in r.log if a.kind == ARRIVED] == ['kitchen', 'table_01']


def test_cancel_and_bad_destination_and_busy():
    r = Run(real(), W_S)
    acts = r.start(['no_such_table'])
    assert acts[-1].kind == DONE and acts[-1].code == 'BAD_DESTINATION'
    r.start(['table_01'])
    assert r.codes(r.start(['table_02'])) == ['BUSY']
    acts = r.core.cancel()
    assert [a.kind for a in acts] == ['cancel_nav', DONE]
    assert acts[-1].result == 'CANCELED' and r.core.state == IDLE


def test_avoided_corridor_is_forgotten_after_60s():
    r = mid_run()
    r.core.request_detour()
    r.advance(0.2)
    assert 'e_west_s' in r.core.blocked
    r.hold = False
    r.advance(61.0)
    assert r.core.blocked == {}


def drive(r, until=None):
    """Finish Nav2 goals until the core stops driving (or reaches `until`)."""
    for _ in range(40):
        if r.core.state != DRIVING:
            return
        r.nav()
    raise AssertionError('still driving')


def serving(table='table_01', params=None):
    r = Run(real(), W_S, params)
    r.log += r.core.start_serve(r.t, 's1', table, r.xy)
    for a in r.log:
        if a.kind == SEND:
            r.gen = a.gen
    return r


def test_serve_waits_for_load_then_table_then_home():
    r = serving()
    drive(r)
    assert r.core.state == WAIT_LOAD and r.core.dest.name == 'kitchen'
    assert 'WAIT_LOAD' in r.codes(r.log)
    ok, _, acts = r.core.confirm_load(r.t, r.xy)
    assert ok
    r._rec(acts)
    assert r.core.state == DRIVING and r.core.dest.name == 'table_01'
    drive(r)
    assert r.core.state == WAIT_HANDOVER and 'WAIT_HANDOVER' in r.codes(r.log)
    ok, _, acts = r.core.confirm_handover(r.t, r.xy)
    assert ok
    r._rec(acts)
    assert r.core.dest.name == 'standby_01'
    drive(r)
    assert r.core.state == IDLE and r.log[-1].result == 'SUCCEEDED'


def test_confirmations_in_the_wrong_state_are_refused():
    r = serving()
    assert r.core.confirm_load(r.t, r.xy)[0] is False      # still driving
    assert r.core.confirm_handover(r.t, r.xy)[0] is False
    drive(r)
    assert r.core.confirm_handover(r.t, r.xy)[0] is False  # waiting for load


def test_load_wait_repeats_alarm_every_30s_and_never_gives_up():
    r = serving()
    drive(r)
    first = len(r.codes(r.log))
    r.advance(95.0)
    assert r.codes(r.log)[first:] == ['WAIT_LOAD'] * 3
    assert r.core.state == WAIT_LOAD


def test_handover_timeout_after_60s_then_repeats():
    r = serving()
    drive(r)
    r._rec(r.core.confirm_load(r.t, r.xy)[2])
    drive(r)
    mark = len(r.log)
    r.advance(59.0)
    assert 'HANDOVER_TIMEOUT' not in r.codes(r.log[mark:])
    r.advance(2.0)
    assert r.codes(r.log[mark:]) == ['HANDOVER_TIMEOUT']
    r.advance(31.0)
    assert r.codes(r.log[mark:]) == ['HANDOVER_TIMEOUT'] * 2
    assert r.core.state == WAIT_HANDOVER


def test_auto_confirm_for_tests():
    r = serving(params=MissionParams(auto_confirm_s=3.0))
    for _ in range(40):
        if r.core.state == IDLE:
            break
        if r.core.state == DRIVING:
            r.nav()
        else:
            r.advance(3.2)
    assert r.core.state == IDLE and r.log[-1].result == 'SUCCEEDED'


def test_refusals_name_the_refused_order():
    """A refused order carries its own id, so its sender can stop waiting."""
    r = Run(real(), W_S)
    r.start(['table_01'], mission_id='m1')
    busy = r.start(['table_02'], mission_id='m2')
    assert r.codes(busy) == ['BUSY'] and busy[0].data['rejected_id'] == 'm2'
    busy = r.core.start_serve(r.t, 's3', 'table_02', r.xy)
    assert busy[0].code == 'BUSY' and busy[0].data['rejected_id'] == 's3'
    assert r.core.mission_id == 'm1' and r.core.state == DRIVING


def test_cancel_with_an_id_cancels_only_that_mission():
    r = Run(real(), W_S)
    r.start(['table_01'], mission_id='m1')
    acts = r.core.cancel('m2')                 # another client's Ctrl-C
    assert r.codes(acts) == ['BAD_COMMAND'] and 'm2' in acts[0].text
    assert r.core.state == DRIVING and r.core.mission_id == 'm1'
    acts = r.core.cancel('m1')                 # the owner
    assert acts[-1].result == 'CANCELED' and r.core.state == IDLE
    assert r.core.cancel('m1') == []           # nothing left to cancel
    r.start(['table_01'], mission_id='m3')
    assert r.core.cancel()[-1].result == 'CANCELED'   # no id: the running one


def test_serve_bad_table_and_busy():
    r = Run(real(), W_S)
    acts = r.core.start_serve(0.0, 'x', 'kitchen', W_S)
    assert acts[0].kind == DONE and acts[0].code == 'BAD_DESTINATION'
    acts = r.core.start_serve(0.0, 'x', 'table_99', W_S)
    assert acts[0].code == 'BAD_DESTINATION'
    r = serving()
    assert r.codes(r.core.start_serve(r.t, 'y', 'table_02', r.xy)) == ['BUSY']


def test_cancel_during_wait():
    r = serving()
    drive(r)
    acts = r.core.cancel()
    assert acts[-1].result == 'CANCELED' and r.core.state == IDLE


def on_way_to_table(limit=180.0):
    """Serving mission, load confirmed at the kitchen, driving to the table."""
    r = serving(params=MissionParams(delivery_limit_s=limit))
    drive(r)
    r._rec(r.core.confirm_load(r.t, r.xy)[2])
    r.xy = (-2.0, 1.55)
    return r


def test_overtime_returns_to_kitchen_then_holds_new_orders():
    r = on_way_to_table()
    assert r.codes(r.advance(179.0)) == []
    acts = r.advance(2.0)
    assert r.codes(acts) == ['DELIVERY_OVERTIME', 'RETURN_STARTED']
    assert r.core.returning and r.sends(acts)[0].leg.node == 'D'
    drive(r)
    assert r.core.state == PAUSED
    last = [a for a in r.log if a.kind == ALARM][-1]
    assert last.code == 'RETURNED_TO_KITCHEN' and last.data['destination'] == 'table_01'
    assert r.log[-1].kind == DONE and r.log[-1].code == 'DELIVERY_OVERTIME'
    # new orders are held
    assert r.codes(r.core.start(r.t, 'n', ['table_02'], r.xy)) == ['HANDOVER_REQUIRED']
    assert r.codes(r.core.start_serve(r.t, 'n', 'table_02', r.xy)) == ['HANDOVER_REQUIRED']
    assert r.core.cancel()[0].code == 'HANDOVER_REQUIRED' and r.core.state == PAUSED
    held = r.core.start_serve(r.t, 'n2', 'table_02', r.xy)
    assert held[0].data['rejected_id'] == 'n2'
    assert 'rejected_id' not in r.core.cancel()[0].data      # not an order
    other = r.core.cancel('someone_else')
    assert other[0].code == 'BAD_COMMAND' and r.core.state == PAUSED
    ok, _, acts = r.core.confirm_handover(r.t, r.xy)
    assert ok and r.core.state == IDLE and r.codes(acts) == ['HANDOVER_CONFIRMED']
    assert r.core.start_serve(r.t, 'n', 'table_02', r.xy)[0].kind == SEND


def test_overtime_stuck_alarm_once_then_returns_when_free():
    r = on_way_to_table(limit=60.0)
    r.hold = True
    acts = r.advance(70.0)
    stuck = [a for a in acts if a.code == 'DELIVERY_OVERTIME_STUCK']
    assert len(stuck) == 1 and stuck[0].data['destination'] == 'table_01'
    assert 'x' in stuck[0].data and not r.core.returning
    r.hold = False
    assert 'DELIVERY_OVERTIME' in r.codes(r.advance(0.5))
    assert r.core.returning


def test_collision_time_does_not_count_for_the_180s():
    r = on_way_to_table(limit=60.0)
    r.advance(40.0)
    r.latched = True
    r.advance(100.0)
    assert r.core.delivery_s < 45.0 and not r.core.returning
    r.latched = False
    r.advance(10.0)
    assert not r.core.returning
    r.advance(15.0)
    assert r.core.returning


def test_timer_only_runs_on_the_way_to_the_table():
    r = serving(params=MissionParams(delivery_limit_s=20.0))
    drive(r)                          # waiting for load: no timer
    r.advance(100.0)
    assert r.core.delivery_s == 0.0 and not r.core.returning
    r._rec(r.core.confirm_load(r.t, r.xy)[2])
    drive(r)                          # at the table, waiting for handover
    r.advance(100.0)
    assert not r.core.returning and r.core.state == WAIT_HANDOVER


def test_plain_go_missions_have_no_delivery_timer():
    r = mid_run()
    r.advance(400.0)
    assert 'DELIVERY_OVERTIME' not in r.codes(r.log)



# ----- straight rail (R1) and "not moving" (R2) ------------------------------------
def stall_params(**kw):
    return MissionParams(stall_window_s=3.0, stall_move_m=0.05,
                         stall_turn_rad=0.15, **kw)


def test_legs_on_the_graph_are_rails_from_the_robot_position():
    r = Run(real(), W_S)
    first = r.sends(r.start(['table_01']))[0]
    assert first.start == W_S and first.leg.from_node == 'W_S'
    r.xy = (-3.2, -0.42)                      # arrived near W_M
    nxt = r.sends(r.nav())[0]
    assert nxt.start == (-3.2, -0.42)          # straight from where it is


def test_direct_goal_is_not_a_rail():
    r = Run(real(), (-3.0, 0.6))              # far from every node
    acts = r.start(['table_01'])
    assert r.codes(acts) == ['DIRECT_GOAL'] and r.sends(acts)[0].start is None


def test_detour_drive_back_is_a_rail_from_where_it_stands():
    r = mid_run()
    r.hold = True
    acts = r.advance(30.1)
    assert 'DETOUR_STARTED' in r.codes(acts)
    back = r.sends(acts)[-1]
    assert back.leg.node == 'W_S' and back.start == MID_WEST_S


def test_rail_poses_are_dense_straight_and_end_with_the_heading():
    poses = rail_poses(-3.2, -2.4, -3.2, -0.4, math.radians(0.0), step_m=0.05)
    assert poses[0][:2] == (-3.2, -2.4) and poses[-1] == (-3.2, -0.4, 0.0)
    assert all(abs(x + 3.2) < 1e-9 for x, _y, _a in poses)          # on the line
    gaps = [math.hypot(b[0] - a[0], b[1] - a[1]) for a, b in zip(poses, poses[1:])]
    assert max(gaps) <= 0.05 + 1e-9 and len(poses) == 41
    assert all(abs(a - math.pi / 2) < 1e-9 for _x, _y, a in poses[:-1])  # north
    still = rail_poses(1.0, 1.0, 1.0, 1.0, 0.7)
    assert still[-1] == (1.0, 1.0, 0.7) and len(still) == 2


def test_not_moving_without_any_signal_gives_blocked_then_detour():
    """The 3-c case: no hold, no person STOP, but the robot stands still."""
    r = mid_run()
    r.core.p = stall_params()
    acts = r.advance(14.8)
    assert r.codes(acts) == [] and r.core._stalled
    acts = r.advance(0.4)
    assert r.codes(acts) == ['BLOCKED'], 'counted from when it stopped moving'
    blocked = [a for a in acts if a.code == 'BLOCKED'][0]
    assert blocked.data['cause'] == 'not_moving' and 'no stop signal' in blocked.text
    assert r.core.status()['stalled'] is True
    assert r.codes(r.advance(14.5)) == []
    assert r.codes(r.advance(0.6)) == ['DETOUR_STARTED'], '30 s in total'


def test_a_moving_robot_is_never_stalled():
    r = mid_run()
    r.core.p = stall_params()
    for _ in range(600):                       # 60 s at 0.05 m/s along the corridor
        r.xy = (r.xy[0], r.xy[1] + 0.005)
        assert r.codes(r.advance(0.1)) == []
    assert not r.core._stalled and r.core.stuck_s == 0.0


def test_turning_in_place_is_moving():
    r = mid_run()
    r.core.p = stall_params()
    r.yaw = 0.0
    for _ in range(100):                       # 10 s turning at 0.2 rad/s
        r.yaw += 0.02
        r.advance(0.1)
    assert not r.core._stalled and r.codes(r.log) == []


def test_after_a_detour_the_wait_starts_again():
    r = mid_run()
    r.core.p = stall_params()
    r.advance(30.2)
    assert r.codes(r.log).count('DETOUR_STARTED') == 1
    acts = r.advance(25.0)                     # still not moving
    assert 'DETOUR_STARTED' not in r.codes(acts), 'no detour every 5 s'
    assert r.codes(acts) == ['BLOCKED']


def test_nav_failure_while_not_moving_is_retried_not_failed():
    r = mid_run()
    r.core.p = stall_params(nav_retries=0)
    r.advance(4.0)
    for _ in range(5):                          # the controller gives up again and again
        acts = r.nav(ok=False)
        assert r.sends(acts) and r.core.state == DRIVING


def test_collision_latch_is_not_a_stall():
    r = mid_run()
    r.core.p = stall_params()
    r.latched = True
    assert r.codes(r.advance(60.0)) == [] and not r.core._stalled


def test_stop_signal_and_stall_share_one_timer():
    r = mid_run()
    r.core.p = stall_params()
    r.hold = True                               # a person stops the robot
    r.advance(10.0)
    r.hold = False                              # signal gone, robot still stuck
    assert r.codes(r.advance(5.2)) == ['BLOCKED']


def test_off_rail_stops_then_drives_on_straight_from_there():
    r = Run(real(), W_S, MissionParams(off_rail_m=0.15, rail_resume_s=3.0))
    r.start(['table_01'])
    old_gen = r.gen
    r.xy = (-3.07, -1.4)                        # 0.13 m off: still fine
    assert r.codes(r.advance(1.0)) == []
    r.xy = (-2.95, -1.4)                        # 0.25 m off the line x = -3.2
    acts = r.advance(0.1)
    assert [a.kind for a in acts][:2] == [CANCEL_NAV, ALARM]
    assert r.codes(acts) == ['OFF_RAIL'] and acts[1].data['off_m'] == 0.25
    assert r.nav(ok=False, gen=old_gen) == [], 'the canceled goal is ignored'
    assert r.sends(r.advance(2.8)) == [], 'waits rail_resume_s'
    acts = r.advance(0.3)
    assert r.sends(acts)[0].start == (-2.95, -1.4) and r.sends(acts)[0].leg.node == 'W_M'
    assert r.codes(r.advance(5.0)) == [], 'on the new line now'


# ----- localization hold ---------------------------------------------------------------
def loc(r, state):
    return r._rec(r.core.on_localization(r.t, state))


def lost_run(**kw):
    """Driving half way to table_01, localization just lost."""
    r = mid_run()
    loc(r, 'OK')
    r.advance(1.0)
    return r, loc(r, 'LOST')


def recovered_run():
    r, _ = lost_run()
    r.advance(1.0)
    loc(r, 'OK')
    acts = r.advance(2.2)
    assert r.codes(acts) == ['LOCALIZATION_RECOVERED'], r.codes(acts)
    return r


def test_lost_while_driving_stops_and_holds_the_mission():
    r, acts = lost_run()
    assert [a.kind for a in acts] == [CANCEL_NAV, ALARM]
    assert acts[1].code == 'LOCALIZATION_LOST' and acts[1].level == 'warn'
    assert acts[1].data['destination'] == 'table_01'
    s = r.core.status()
    assert s['state'] == DRIVING and s['loc_hold'] == 'LOST' and s['localization'] == 'LOST'
    assert r.core.mission_id == 'm1', 'the mission is kept'
    assert r.nav(ok=True) == [], 'a goal result during the hold is ignored'


def test_lost_is_reported_once_however_often_it_is_fed():
    r, _ = lost_run()
    out = []
    for _ in range(50):
        r.t += 0.1
        out += loc(r, 'LOST')
        out += r.advance(0.1)
    assert r.codes(out) == [], 'no repeat inside 30 s'
    assert r.sends(out) == []


def test_lost_alarm_is_repeated_every_30s_while_waiting():
    r, _ = lost_run()
    loc(r, 'LOST')
    assert r.codes(r.advance(29.0)) == []
    assert r.codes(r.advance(1.5)) == ['LOCALIZATION_LOST']


def test_degraded_localization_does_not_stop_the_robot():
    r = mid_run()
    assert loc(r, 'DEGRADED') == []
    assert r.advance(5.0) == []
    assert r.core.status()['loc_hold'] == ''


def test_unknown_localization_does_not_stop_the_robot():
    r = mid_run()
    assert loc(r, None) == []
    assert r.core.status()['loc_hold'] == ''
    assert r.core.start(r.t, 'x', ['kitchen'], r.xy)[0].code == 'BUSY'


def test_hold_stops_every_timer():
    r = on_way_to_table(limit=180.0)
    r.advance(50.0)
    used = r.core.delivery_s
    assert 49.0 < used < 51.0
    loc(r, 'LOST')
    r.hold = True                                # an obstacle in front, too
    out = r.advance(200.0)                       # way beyond 15 s / 30 s / 180 s
    assert set(r.codes(out)) == {'LOCALIZATION_LOST'}, r.codes(out)    # reminders only
    assert r.sends(out) == []
    assert r.core.delivery_s == used and r.core.stuck_s < 1.0
    assert r.core.state == DRIVING


def test_ok_must_be_stable_before_the_operator_is_asked():
    r, _ = lost_run()
    loc(r, 'OK')
    assert r.codes(r.advance(1.8)) == []
    loc(r, 'LOST')                               # flicker: starts again
    r.advance(0.5)
    loc(r, 'OK')
    assert r.codes(r.advance(1.8)) == []
    assert r.core.status()['loc_hold'] == 'LOST'
    acts = r.advance(0.4)
    assert r.codes(acts) == ['LOCALIZATION_RECOVERED']
    a = acts[0]
    assert a.level == 'warn' and a.data['choices'] == ['resume', 'home']
    assert a.data['localization'] == 'OK' and 'rough' not in a.text
    assert a.data['destination'] == 'table_01' and a.data['home'] == 'standby_01'
    assert r.core.status()['loc_hold'] == 'RECOVERED'


def test_a_rough_position_is_enough_to_ask_the_operator():
    """A stopped robot cannot make AMCL more sure: DEGRADED must not wait for OK."""
    r, _ = lost_run()
    loc(r, 'DEGRADED')
    assert r.codes(r.advance(1.8)) == []
    acts = r.advance(0.4)
    assert r.codes(acts) == ['LOCALIZATION_RECOVERED']
    assert acts[0].data['localization'] == 'DEGRADED' and 'rough' in acts[0].text
    ok, _, go = r.core.resume(r.t, r.xy)         # and the choice works as usual
    assert ok and r.sends(go)


def test_ok_and_degraded_count_together_but_lost_or_unknown_breaks_it():
    r, _ = lost_run()
    loc(r, 'DEGRADED')
    r.advance(1.5)
    loc(r, 'OK')                                 # same period: not a restart
    assert r.codes(r.advance(0.6)) == ['LOCALIZATION_RECOVERED']
    r, _ = lost_run()
    loc(r, 'DEGRADED')
    r.advance(1.5)
    loc(r, None)                                 # no health: unknown breaks it
    loc(r, 'DEGRADED')
    assert r.codes(r.advance(1.5)) == []
    assert r.codes(r.advance(0.8)) == ['LOCALIZATION_RECOVERED']


def test_uninitialized_does_not_end_the_wait():
    r, _ = lost_run()
    loc(r, 'UNINITIALIZED')
    assert r.codes(r.advance(10.0)) == []
    assert r.core.status()['loc_hold'] == 'LOST'


def test_reminder_names_the_current_state():
    r, _ = lost_run()
    loc(r, 'DEGRADED')
    r.advance(2.2)
    loc(r, 'OK')
    out = r.advance(30.5)
    a = [x for x in out if x.kind == ALARM][0]
    assert a.code == 'LOCALIZATION_RECOVERED' and a.data['localization'] == 'OK'


def test_degraded_while_waiting_for_the_choice_does_not_cancel_it():
    r = recovered_run()
    loc(r, 'DEGRADED')
    assert r.advance(5.0) == []
    assert r.core.status()['loc_hold'] == 'RECOVERED'


def test_the_core_never_moves_by_itself_after_recovery():
    r = recovered_run()
    out = r.advance(25.0)
    assert r.sends(out) == [] and r.core.status()['loc_hold'] == 'RECOVERED'
    # a late result of the goal that was running must not move it on either
    assert r.core.on_nav_result(r.t, r.core.gen, True, r.xy) == []
    assert r.core.on_nav_result(r.t, r.core.gen, False, r.xy) == []
    assert r.core.status()['loc_hold'] == 'RECOVERED' and r.core.leg_i == 0


def test_recovered_alarm_reminds_every_30s():
    r = recovered_run()
    assert r.codes(r.advance(29.0)) == []
    assert r.codes(r.advance(1.5)) == ['LOCALIZATION_RECOVERED']


def test_resume_drives_on_to_the_same_destination():
    r = recovered_run()
    ok, text, acts = r.core.resume(r.t, r.xy)
    assert ok and r.codes(acts)[0] == 'MISSION_RESUMED'
    r._rec(acts)
    send = r.sends(acts)[0]
    assert send.start == MID_WEST_S and send.leg.node == 'W_M'
    assert 'DIRECT_GOAL' not in r.codes(acts), 'still a straight rail, no planner'
    s = r.core.status()
    assert s['loc_hold'] == '' and s['destination'] == 'table_01' and s['state'] == DRIVING
    drive(r)
    assert r.log[-1].kind == DONE and r.log[-1].result == 'SUCCEEDED'


def test_resume_keeps_the_delivery_time_already_used():
    r = on_way_to_table()
    r.advance(50.0)
    loc(r, 'LOST')
    r.advance(1.0)
    loc(r, 'OK')
    r.advance(2.2)
    r._rec(r.core.resume(r.t, r.xy)[2])
    assert 49.0 < r.core.delivery_s < 51.0
    assert r.core.timing and r.core.serve


def test_resume_after_a_jump_keeps_the_delivery_time_too():
    r = on_way_to_table()
    r.advance(50.0)
    loc(r, 'LOST')
    r.advance(1.0)
    loc(r, 'OK')
    r.advance(2.2)
    r.xy = W_S                                  # relocalized far from the corridor
    r._rec(r.core.resume(r.t, (-0.5, -0.5))[2])
    assert 49.0 < r.core.delivery_s < 51.0 and r.core.timing


def test_resume_resets_the_wait_counters():
    r = recovered_run()
    r.core.stuck_s, r.core.alarmed = 12.0, True
    r._rec(r.core.resume(r.t, r.xy)[2])
    assert r.core.stuck_s == 0.0 and r.core.alarmed is False


def test_resume_plans_again_when_the_position_jumped_away():
    r = recovered_run()
    r.xy = (-2.0, 1.55)                         # far from the corridor it was on
    ok, _, acts = r.core.resume(r.t, r.xy)
    assert ok and r.sends(acts), 'a new plan from the new position'
    r._rec(acts)
    drive(r)
    assert r.log[-1].result == 'SUCCEEDED'


def test_resume_and_home_are_refused_in_the_wrong_phase():
    r = mid_run()
    for fn in (lambda: r.core.resume(r.t, r.xy), lambda: r.core.go_home(r.t)):
        ok, text, acts = fn()
        assert not ok and acts == [] and 'no mission' in text
    r, _ = lost_run()                           # position not back yet
    for fn in (lambda: r.core.resume(r.t, r.xy), lambda: r.core.go_home(r.t)):
        ok, text, acts = fn()
        assert not ok and acts == [] and 'RECOVERED' in text
    assert r.core.status()['loc_hold'] == 'LOST' and r.core.mission_id == 'm1'


def test_home_cancels_the_mission_and_the_caller_drives_home():
    r = recovered_run()
    ok, mid, acts = r.core.go_home(r.t)
    assert ok and mid == 'm1'
    assert [a.kind for a in acts] == [CANCEL_NAV, ALARM, DONE]
    assert acts[1].code == 'GO_HOME'
    assert acts[2].result == 'CANCELED' and acts[2].code == 'LOCALIZATION_LOST'
    assert r.core.state == IDLE and r.core.status()['loc_hold'] == ''
    r._rec(acts)
    acts = r.core.start(r.t, 'm1_home', ['standby_01'], r.xy)    # what the node does
    r._rec(acts)
    assert r.sends(acts), 'driving home'
    drive(r)
    assert r.log[-1].result == 'SUCCEEDED' and r.core.state == IDLE


def test_new_orders_are_refused_during_the_hold():
    r, _ = lost_run()
    for phase in ('LOST', 'RECOVERED'):
        if phase == 'RECOVERED':
            r = recovered_run()
        a = r.core.start(r.t, 'n1', ['kitchen'], r.xy)
        assert [x.code for x in a] == ['LOCALIZATION_HOLD']
        assert a[0].data['rejected_id'] == 'n1'
        a = r.core.start_serve(r.t, 'n2', 'table_02', r.xy)
        assert a[0].code == 'LOCALIZATION_HOLD' and a[0].data['rejected_id'] == 'n2'
        assert r.core.mission_id == 'm1'


def test_idle_robot_without_a_position_takes_no_order():
    r = Run(real(), W_S)
    for state, code in (('LOST', 'LOCALIZATION_LOST'),
                        ('UNINITIALIZED', 'LOCALIZATION_UNINITIALIZED')):
        loc(r, state)
        for acts in (r.core.start(r.t, 'o1', ['table_01'], r.xy),
                     r.core.start_serve(r.t, 'o2', 'table_01', r.xy)):
            assert [a.code for a in acts] == [code] and r.sends(acts) == []
            assert acts[0].data['rejected_id'] in ('o1', 'o2')
        assert r.core.state == IDLE
    loc(r, 'OK')
    assert r.sends(r.core.start(r.t, 'o3', ['table_01'], r.xy))


def test_lost_again_while_waiting_for_the_choice_holds_again():
    r = recovered_run()
    acts = loc(r, 'LOST')
    assert [a.kind for a in acts] == [CANCEL_NAV, ALARM]
    assert 'lost again' in acts[1].text and r.core.status()['loc_hold'] == 'LOST'
    assert r.core.resume(r.t, r.xy)[0] is False


def test_lost_while_waiting_at_the_kitchen_only_raises_the_alarm():
    r = serving()
    drive(r)
    assert r.core.state == WAIT_LOAD
    acts = loc(r, 'LOST')
    assert [a.kind for a in acts] == [ALARM] and acts[0].code == 'LOCALIZATION_LOST'
    assert r.core.state == WAIT_LOAD and r.core.status()['loc_hold'] == ''
    assert loc(r, 'LOST') == [], 'once'
    assert r.core.confirm_load(r.t, r.xy)[0], 'the confirmation still works'


def test_cancel_during_the_hold_ends_it():
    r = recovered_run()
    acts = r.core.cancel()
    assert [a.kind for a in acts] == [CANCEL_NAV, DONE] and acts[1].result == 'CANCELED'
    assert r.core.state == IDLE and r.core.status()['loc_hold'] == ''


def test_detour_request_is_refused_during_the_hold():
    r, _ = lost_run()
    ok, text = r.core.request_detour()
    assert not ok and 'hold' in text


def test_off_rail_restart_does_not_happen_during_the_hold():
    r = Run(real(), W_S, MissionParams(off_rail_m=0.15, rail_resume_s=3.0))
    r.start(['table_01'])
    r.xy = (-2.95, -1.4)
    assert r.codes(r.advance(0.1)) == ['OFF_RAIL']
    loc(r, 'LOST')
    out = r.advance(20.0)
    assert r.sends(out) == [], 'only the operator restarts the robot'


def test_status_tells_the_localization_state():
    r = mid_run()
    assert r.core.status()['localization'] == '' and r.core.status()['loc_hold'] == ''
    loc(r, 'DEGRADED')
    assert r.core.status()['localization'] == 'DEGRADED'


def test_serve_without_home_ends_at_the_table():
    r = Run(real(), W_S)
    r.log += r.core.start_serve(r.t, 's1', 'table_01', r.xy, home=False)
    for a in r.log:
        if a.kind == SEND:
            r.gen = a.gen
    drive(r)
    assert r.core.state == WAIT_LOAD
    ok, _, acts = r.core.confirm_load(r.t, r.xy)
    r._rec(acts)
    drive(r)
    assert r.core.state == WAIT_HANDOVER and r.core.dest.name == 'table_01'
    ok, _, acts = r.core.confirm_handover(r.t, r.xy)
    assert ok and [a.kind for a in acts] == [DONE]
    assert acts[0].result == 'SUCCEEDED' and r.core.state == IDLE
    assert r.core.status()['route'] == [] and r.core.status()['mission_id'] == ''


def test_serve_with_home_is_unchanged_by_the_option():
    r = serving()
    drive(r)
    r._rec(r.core.confirm_load(r.t, r.xy)[2])
    drive(r)
    acts = r.core.confirm_handover(r.t, r.xy)[2]
    assert any(a.kind == SEND for a in acts) and r.core.dest.name == 'standby_01'


# ----- route info in the status (node / next_edge / route) -------------------------
def test_status_route_info_while_driving_a_serve_order():
    r = serving()
    r.xy = MID_WEST_S
    r.advance(0.2)
    st = r.core.status()
    assert st['node'] == 'W_S' and st['next_edge'] == 'e_west_s'
    assert st['route'][0] == 'W_M' and 'D' in st['route']
    assert 'T13' in st['route'] and st['route'][-1] == 'W_S'
    r.xy = (-3.20, -0.80)                    # past W_S, on e_west_s still
    r.advance(0.2)
    assert r.core.status()['route'][0] == 'W_M'
    r.xy = (-3.20, 0.60)                     # now on e_west_n
    r.advance(0.2)
    st = r.core.status()
    assert st['node'] == 'W_M' and st['next_edge'] == 'e_west_n'
    assert st['route'][0] == 'W_N'


def test_status_route_info_is_empty_or_quiet_when_not_driving():
    r = Run(real(), W_S)
    r.advance(0.2)
    st = r.core.status()
    assert st['node'] == 'W_S' and st['next_edge'] == '' and st['route'] == []
    r = serving()
    drive(r)
    assert r.core.state == WAIT_LOAD
    st = r.core.status()
    assert st['node'] == 'D' and st['next_edge'] == '' and st['route'] == []


# ----- operator reset (robot put by hand on a node) --------------------------------
def test_reset_frees_a_stuck_mission_and_clears_every_timer():
    r = mid_run()
    r.hold = True
    r.advance(40.0)                                   # BLOCKED at 15 s, detour at 30 s
    assert r.core.status()['stopped_s'] > 0.0 and r.core.detours == 1
    mid, acts = r.core.reset(r.t, 'kitchen')
    assert mid == 'm1'
    assert [a.kind for a in acts] == [CANCEL_NAV, ALARM, DONE]
    assert acts[1].code == 'ROBOT_RESET' and 'kitchen' in acts[1].text
    assert acts[2].result == 'CANCELED' and acts[2].code == 'RESET'
    st = r.core.status()
    assert r.core.state == IDLE and st['loc_hold'] == '' and st['stopped_s'] == 0.0
    assert st['detours'] == 0 and not st['stalled']
    out = r.advance(60.0)
    assert out == [], 'nothing runs after the reset (no alarm, no goal)'


def test_reset_in_idle_only_cancels_and_tells():
    r = Run(real(), W_S)
    mid, acts = r.core.reset(r.t, 'standby_01')
    assert [a.kind for a in acts] == [CANCEL_NAV, ALARM] and acts[1].code == 'ROBOT_RESET'
    assert r.core.state == IDLE


def test_reset_drops_the_localization_hold_but_orders_wait_for_a_good_position():
    r, _ = lost_run()
    assert r.core.status()['loc_hold'] == 'LOST'
    r.core.reset(r.t, 'kitchen')
    assert r.core.status()['loc_hold'] == ''
    a = r.core.start(r.t, 'n1', ['kitchen'], r.xy)
    assert not r.sends(a), 'still LOST: refused until health says otherwise'
    loc(r, 'OK')
    a = r.core.start(r.t, 'n2', ['kitchen'], r.xy)
    assert r.sends(a), 'position is back: the order is accepted'


def test_reset_works_in_every_waiting_state_and_ignores_old_nav_results():
    r = Run(real(), W_S)
    r.start(['table_01'])
    gen = r.gen
    r.core.reset(r.t, 'kitchen')
    out = r.core.on_nav_result(r.t, gen, True, r.xy)
    assert out == [] and r.core.state == IDLE, 'result of the dropped goal is ignored'
    r2 = Run(real(), W_S)
    r2.core.start_serve(r2.t, 's1', 'table_01', r2.xy)
    for _ in range(3):
        r2.advance(100.0)
        if r2.core.state == WAIT_LOAD:
            break
        r2.nav()
    r2.core.reset(r2.t, 'kitchen')
    assert r2.core.state == IDLE and r2.advance(60.0) == []


def test_replace_site_only_while_idle_and_reports_its_revision():
    r = Run(real(), W_S)
    assert r.core.status()['config_revision'] == real().revision
    other = real()
    other.revision = 'abc123abc123'
    ok, _ = r.core.replace_site(other)
    assert ok and r.core.site is other
    assert r.core.status()['config_revision'] == 'abc123abc123'
    r.core.blocked = {'e1': 99.0}
    ok, _ = r.core.replace_site(real())
    assert ok and r.core.blocked == {}, 'old edge ids are forgotten'


def test_replace_site_refused_while_driving_or_on_localization_hold():
    r = Run(real(), W_S)
    r.core.start_serve(r.t, 's1', 'table_01', r.xy)
    old = r.core.site
    ok, text = r.core.replace_site(real())
    assert not ok and r.core.site is old and 'not idle' in text
    r2 = Run(real(), W_S)
    r2.core.loc_hold = 'LOST'
    ok, _ = r2.core.replace_site(real())
    assert not ok


if __name__ == '__main__':
    for name, fn in sorted(globals().items()):
        if name.startswith('test_') and callable(fn):
            fn()
            print('ok  ', name)
    print('all passed')
