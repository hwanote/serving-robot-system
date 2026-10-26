#!/usr/bin/env python3
"""C-1: tour plan, drive-test folder, verdicts, report, runner (no ROS: a fake driver).
Run: python3 tools/site_builder/tests/test_c1_tour.py"""
import hashlib
import json
import os
import sys
import tempfile

import yaml

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, '..'))
sys.path.insert(0, os.path.join(REPO := os.path.abspath(os.path.join(HERE, '..', '..', '..')),
                                'ros_ws', 'src', 'serving_robot_navigation'))
from site_builder import probe                                 # noqa: E402
from site_builder.params import load_params                    # noqa: E402
from site_builder.pipeline import generate, write_outputs      # noqa: E402
from site_builder.report import Report, ReportError            # noqa: E402
from site_builder.runner import Driver, TourRunner             # noqa: E402
from site_builder.tour import TourError, euler_tour, graph_of, plan   # noqa: E402
from site_builder.verdict import judge, level                  # noqa: E402

DEMO_MAP = os.path.join(REPO, 'maps', 'stores', 'demo_store', 'map.yaml')
DEMO_LABELS = os.path.join(HERE, '..', 'examples', 'labels_demo.json')
COMMON = os.path.join(REPO, 'config', 'common')
PAR = load_params()


def wp(nodes, edges, poses=None):
    return {'nodes': {k: {'x': v[0], 'y': v[1]} for k, v in nodes.items()},
            'edges': [{'id': e, 'from': a, 'to': b, 'zone': 'z'} for e, a, b in edges],
            'poses': poses or {'home': {'node': next(iter(nodes)), 'yaw_deg': 0, 'kind': 'standby'}}}


SHAPES = {
    'line': ({'a': (0, 0), 'b': (1, 0), 'c': (2, 0)}, [('e1', 'a', 'b'), ('e2', 'b', 'c')]),
    'L': ({'a': (0, 0), 'b': (2, 0), 'c': (2, 2)}, [('e1', 'a', 'b'), ('e2', 'b', 'c')]),
    'cross': ({'o': (0, 0), 'n': (0, 2), 's': (0, -2), 'e': (2, 0), 'w': (-2, 0)},
              [('e1', 'o', 'n'), ('e2', 'o', 's'), ('e3', 'o', 'e'), ('e4', 'o', 'w')]),
    'ring': ({'a': (0, 0), 'b': (2, 0), 'c': (2, 2), 'd': (0, 2)},
             [('e1', 'a', 'b'), ('e2', 'b', 'c'), ('e3', 'c', 'd'), ('e4', 'd', 'a')]),
    'ring+tail': ({'a': (0, 0), 'b': (2, 0), 'c': (2, 2), 'd': (0, 2), 't': (-2, 0)},
                  [('e1', 'a', 'b'), ('e2', 'b', 'c'), ('e3', 'c', 'd'), ('e4', 'd', 'a'), ('e5', 'a', 't')]),
}


def test_tour_covers_every_corridor_once_each_way_and_is_continuous():
    for name, (nodes, edges) in SHAPES.items():
        start = next(iter(nodes))
        t = euler_tour(nodes, edges, start)
        assert len(t) == 2 * len(edges), name
        assert len({(a, b, e) for e, a, b in t}) == len(t), name           # no arc twice
        assert {(a, b) for _, a, b in t} | {(b, a) for _, a, b in t} == \
            {(a, b) for _, a, b in t}, name
        assert t[0][1] == start and t[-1][2] == start, name                # closed at home
        for (e1, a1, b1), (e2, a2, b2) in zip(t, t[1:]):
            assert b1 == a2, (name, 'not continuous')
        for e, a, b in edges:
            assert (e, a, b) in t and (e, b, a) in t, (name, e)


def test_tour_avoids_turning_straight_back_when_it_can():
    nodes, edges = SHAPES['cross']
    t = euler_tour(nodes, edges, 'o')
    back = sum(1 for x, y in zip(t, t[1:]) if x[0] == y[0])
    assert back == len(edges), back      # in a star every arm needs one turn-back at its tip only
    nodes, edges = SHAPES['ring']
    t = euler_tour(nodes, edges, 'a')
    # a ring is one lap each way: switching direction needs exactly one turn-back, no more
    assert sum(1 for x, y in zip(t, t[1:]) if x[0] == y[0]) == 1, t


def test_disconnected_or_unknown_start_is_an_error():
    nodes = {'a': (0, 0), 'b': (1, 0), 'c': (5, 5), 'd': (6, 5)}
    try:
        euler_tour(nodes, [('e1', 'a', 'b'), ('e2', 'c', 'd')], 'a')
    except TourError as e:
        assert 'e2' in str(e)
    else:
        raise AssertionError('disconnected graph must be refused')
    try:
        euler_tour(nodes, [('e1', 'a', 'b')], 'zz')
    except TourError:
        pass
    else:
        raise AssertionError('unknown start must be refused')


def test_plan_has_lengths_turns_and_a_stable_hash():
    nodes, edges = SHAPES['L']
    p1 = plan(wp(nodes, edges), 'a')
    p2 = plan(wp(nodes, edges), 'a')
    assert p1['hash'] == p2['hash'] and len(p1['steps']) == 4
    s0 = p1['steps'][0]
    assert s0['length_m'] == 2.0 and s0['turn_deg'] == 0.0
    assert p1['steps'][1]['turn_deg'] == 90.0 and p1['steps'][1]['expected_s'] > 2.0 / 0.25
    nodes, edges = SHAPES['ring']
    assert plan(wp(nodes, edges), 'b')['hash'] != plan(wp(nodes, edges), 'a')['hash']


def tree_hash(d):
    h = hashlib.sha1()
    for n in sorted(os.listdir(d)):
        h.update(n.encode())
        h.update(open(os.path.join(d, n), 'rb').read())
    return h.hexdigest()


def test_probe_folder_loads_resolves_probe_poses_and_leaves_the_candidate_alone():
    from serving_robot_navigation.site_config import load_site
    res = generate(DEMO_MAP, DEMO_LABELS)
    with tempfile.TemporaryDirectory() as tmp:
        cand, out = os.path.join(tmp, 'cand'), os.path.join(tmp, 'drive')
        write_outputs(res, cand)
        before = tree_hash(cand)
        tour = probe.build(cand, COMMON, out)
        assert tree_hash(cand) == before
        site = load_site(out)
        assert os.path.isfile(os.path.join(out, 'interfaces.yaml'))       # copied from config/common
        assert len(tour['steps']) == 2 * len(res.graph.edges)
        for s in tour['steps']:
            p = site.resolve(s['probe'])
            assert p.node == s['to'] and abs(p.yaw_deg - s['heading_deg']) < 0.06, s
        for name in res.poses:                                           # the real poses are still there
            assert site.resolve(name).name == name
        # a leg from the step's start node to the probe pose is exactly that corridor
        for s in tour['steps']:
            legs = site.route_legs(s['from'], site.resolve(s['probe']))
            assert [e for l in legs for e in l.edges] == [s['edge']], s
        for bad in (cand, COMMON):
            try:
                probe.build(cand, COMMON, bad)
            except probe.ProbeError:
                pass
            else:
                raise AssertionError('must refuse to write over %s' % bad)


def obs(**kw):
    d = {'result': 'SUCCEEDED', 'alarms': [], 'duration_s': 8.0, 'expected_s': 8.0}
    d.update(kw)
    return d


def test_verdicts():
    assert judge(obs(), PAR) == ('PASS', [], [])
    assert judge(obs(latched=True, result='FAILED'), PAR)[0] == 'COLLISION'
    assert judge(obs(hardstop_s=2.0, result='CANCELED', alarms=['BLOCKED']), PAR)[0] == 'HARDSTOP'
    assert judge(obs(alarms=['BLOCKED'], result='CANCELED'), PAR)[0] == 'BLOCKED'
    assert judge(obs(alarms=['DETOUR_STARTED']), PAR)[0] == 'BLOCKED'
    assert judge(obs(alarms=['OFF_RAIL']), PAR)[0] == 'OFF_RAIL'
    assert judge(obs(result='FAILED', code='NAV_FAILED'), PAR)[0] == 'NAV_FAILED'
    assert judge(obs(result='FAILED', code='NO_ROUTE'), PAR)[0] == 'FAILED'
    assert judge(obs(result='', timed_out=True, timeout_s=34), PAR)[0] == 'TIMEOUT'
    assert judge(obs(result='CANCELED'), PAR)[0] == 'FAILED'
    # worst one wins
    assert judge(obs(latched=True, alarms=['BLOCKED', 'OFF_RAIL']), PAR)[0] == 'COLLISION'
    assert judge(obs(hardstop_s=1.5, latched=False, alarms=['BLOCKED']), PAR)[0] == 'HARDSTOP'
    assert judge(obs(alarms=['BLOCKED', 'OFF_RAIL']), PAR)[0] == 'BLOCKED'
    # a pass can still carry warnings
    v, f, r = judge(obs(duration_s=30.0), PAR)
    assert (v, f) == ('PASS', ['SLOW']) and r
    assert judge(obs(stopped_max_s=3.5), PAR)[1] == ['STALLED']
    v, f, r = judge(obs(hardstop_s=0.4), PAR)           # touched the zone but got through
    assert (v, f) == ('PASS', ['HARDSTOP_BRIEF']) and level(v, f) == 'WARN'
    assert judge(obs(hardstop_s=0.05), PAR)[1] == []    # too short to mean anything
    v, f, r = judge(obs(person_slow_s=3.0, person_stop_s=2.0), PAR)   # held by the person speed limit
    assert (v, f) == ('PASS', ['PERSON_LIMIT']) and level(v, f) == 'WARN' and 'person' in r[0]
    assert judge(obs(person_slow_s=0.4), PAR)[1] == []  # a blink is not worth a flag
    assert judge(obs(min_left_m=PAR.hs_half_m + 0.05), PAR)[1] == ['TIGHT']
    assert judge(obs(min_left_m=PAR.hs_half_m + 0.30, min_right_m=None), PAR)[1] == []
    assert level('PASS', []) == 'OK' and level('PASS', ['TIGHT']) == 'WARN' and level('BLOCKED', []) == 'FAIL'


class Fake(Driver):
    """script: {step index: observation overrides}; records what the runner asked for."""

    def __init__(self, script=None, recover_ok=True, start_ok=True):
        self.script, self.recover_ok, self.start_ok = script or {}, recover_ok, start_ok
        self.calls = []

    def start(self, node):
        self.calls.append(('start', node))
        return self.start_ok

    def recover(self, node, reason):
        self.calls.append(('recover', node))
        return self.recover_ok

    def run_step(self, step, timeout_s):
        self.calls.append(('step', step['i']))
        o = {'result': 'SUCCEEDED', 'alarms': [], 'duration_s': step['expected_s']}
        o.update(self.script.get(step['i'], {}))
        return o


def ring_tour():
    nodes, edges = SHAPES['ring+tail']
    return plan(wp(nodes, edges), 'a')


def runner(tmp, driver, tour=None, **kw):
    tour = tour or ring_tour()
    rep = Report(os.path.join(tmp, 'r.json'), tour)
    return TourRunner(tour, rep, driver, PAR, log=lambda *_: None, **kw), rep, tour


def test_runner_clean_tour():
    with tempfile.TemporaryDirectory() as tmp:
        d = Fake()
        r, rep, tour = runner(tmp, d)
        out = r.run()
        assert out == {'ran': 10, 'skipped': 0, 'aborted': None}, out
        assert d.calls[0] == ('start', 'a') and not any(c[0] == 'recover' for c in d.calls)
        assert all(x['verdict'] == 'PASS' for x in rep.data['steps'].values())
        assert 'edges: OK 5 / WARN 0 / FAIL 0' in rep.summary()


def test_a_failed_corridor_is_marked_and_the_tour_goes_on_after_recovery():
    with tempfile.TemporaryDirectory() as tmp:
        tour = ring_tour()
        bad = 3                                  # the third step is blocked
        s = tour['steps'][bad - 1]
        d = Fake({bad: {'result': 'CANCELED', 'alarms': ['BLOCKED']}})
        r, rep, _ = runner(tmp, d, tour)
        out = r.run()
        assert out['ran'] == 10 and out['aborted'] is None
        assert rep.get(bad)['verdict'] == 'BLOCKED'
        nxt = tour['steps'][bad]                 # the following step starts from its own from-node
        assert ('recover', nxt['from']) in d.calls
        assert d.calls.index(('recover', nxt['from'])) < d.calls.index(('step', nxt['i']))
        assert rep.get(nxt['i'])['recovered_before']
        assert sum(1 for c in d.calls if c[0] == 'recover') == 1
        # the other direction of that corridor was still tried and passed
        both = rep.edge_table()[s['edge']]
        assert both['verdict'] == 'BLOCKED' and len(both['dirs']) == 2


def test_runner_stops_after_too_many_failures_in_a_row():
    with tempfile.TemporaryDirectory() as tmp:
        d = Fake({i: {'result': 'FAILED', 'code': 'NAV_FAILED'} for i in range(1, 11)})
        r, rep, _ = runner(tmp, d, max_consecutive_failures=3)
        out = r.run()
        assert out['ran'] == 3 and 'in a row' in out['aborted'], out


def test_runner_stops_when_recovery_is_refused_or_start_is_not_ready():
    with tempfile.TemporaryDirectory() as tmp:
        d = Fake({2: {'hardstop_s': 3.0, 'result': 'FAILED'}}, recover_ok=False)
        r, rep, _ = runner(tmp, d)
        out = r.run()
        assert out['ran'] == 2 and 'putting the robot on node' in out['aborted'], out
        assert rep.get(2)['verdict'] == 'HARDSTOP' and not rep.done(3)
    with tempfile.TemporaryDirectory() as tmp:
        d = Fake(start_ok=False)
        r, rep, _ = runner(tmp, d)
        out = r.run()
        assert out['ran'] == 0 and 'not ready' in out['aborted'] and not rep.data['steps']


def test_resume_skips_finished_steps_and_does_not_repeat_the_start():
    with tempfile.TemporaryDirectory() as tmp:
        tour = ring_tour()
        d1 = Fake()
        r1, rep1, _ = runner(tmp, d1, tour, only={'1', '2', '3'})
        r1.run()
        assert [c for c in d1.calls if c[0] == 'step'] == [('step', 1), ('step', 2), ('step', 3)]
        # second session: same report file, new driver
        d2 = Fake()
        rep2 = Report(os.path.join(tmp, 'r.json'), tour)
        out = TourRunner(tour, rep2, d2, PAR, log=lambda *_: None).run()
        assert out['ran'] == 7 and out['skipped'] == 3, out
        assert ('start', 'a') not in d2.calls
        assert [c for c in d2.calls if c[0] == 'step'][0] == ('step', 4)


def test_only_filters_by_edge_and_recovers_before_the_first_wanted_step():
    with tempfile.TemporaryDirectory() as tmp:
        tour = ring_tour()
        d = Fake()
        r, rep, _ = runner(tmp, d, tour, only={'e3'})
        out = r.run()
        assert out['ran'] == 2 and out['skipped'] == 8, out
        assert d.calls[0][0] == 'recover' and not any(c[0] == 'start' for c in d.calls)


def test_report_refuses_another_tour_and_survives_a_crash():
    with tempfile.TemporaryDirectory() as tmp:
        tour = ring_tour()
        r, rep, _ = runner(tmp, Fake())
        r.run()
        nodes, edges = SHAPES['ring']
        other = plan(wp(nodes, edges), 'a')
        try:
            Report(os.path.join(tmp, 'r.json'), other)
        except ReportError as e:
            assert 'another tour' in str(e)
        else:
            raise AssertionError('a report of another tour must be refused')
        assert not os.path.exists(os.path.join(tmp, 'r.json.tmp'))
        assert json.load(open(os.path.join(tmp, 'r.json')))['tour_hash'] == tour['hash']

        class Boom(Fake):
            def run_step(self, step, timeout_s):
                if step['i'] == 4:
                    raise KeyboardInterrupt
                return super().run_step(step, timeout_s)
        with tempfile.TemporaryDirectory() as tmp2:
            r2, rep2, _ = runner(tmp2, Boom())
            out = r2.run()
            assert 'interrupted' in out['aborted'] and out['ran'] == 3
            assert len(json.load(open(os.path.join(tmp2, 'r.json')))['steps']) == 3


def test_time_scale_moves_the_time_limit_and_the_slow_check():
    """A slow machine (simulation at 1/3.5 speed) must not time out or be flagged SLOW; scale 1 is unchanged."""
    class Slow(Fake):
        def run_step(self, step, timeout_s):
            self.timeouts = getattr(self, 'timeouts', []) + [timeout_s]
            o = super().run_step(step, timeout_s)
            o['duration_s'] = step['expected_s'] * 3.4
            return o
    with tempfile.TemporaryDirectory() as tmp:
        d = Slow()
        r, rep, tour = runner(tmp, d)
        r.run()
        assert all('SLOW' in x['flags'] for x in rep.data['steps'].values())       # default: flagged
        assert d.timeouts[0] == tour['steps'][0]['expected_s'] * 3.0 + 10.0
    with tempfile.TemporaryDirectory() as tmp:
        d = Slow()
        r, rep, tour = runner(tmp, d, time_scale=3.5)
        r.run()
        assert all(x['flags'] == [] and x['verdict'] == 'PASS' for x in rep.data['steps'].values())
        assert abs(d.timeouts[0] - (tour['steps'][0]['expected_s'] * 3.5 * 3.0 + 10.0)) < 1e-9
        st = rep.get(1)['obs']
        assert st['time_scale'] == 3.5 and st['expected_s'] == round(tour['steps'][0]['expected_s'] * 3.5, 1)
    for bad in (0, -1):
        try:
            runner(tempfile.mkdtemp(), Fake(), time_scale=bad)
        except ValueError:
            continue
        raise AssertionError('time_scale %s accepted' % bad)


if __name__ == '__main__':
    n = 0
    for k, v in sorted(globals().items()):
        if k.startswith('test_') and callable(v):
            v()
            n += 1
            print('ok', k)
    print('all passed' if n else 'no tests')
