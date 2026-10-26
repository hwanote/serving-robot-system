#!/usr/bin/env python3
"""E: stops placed by hand ("stop" on a label) -> the corridor point nearest to it, same checks.
Run: python3 tools/site_builder/tests/test_e_pins.py"""
import copy
import json
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
sys.path.insert(0, os.path.join(HERE, '..'))
from site_builder.labels import LabelError, load_labels   # noqa: E402
from site_builder.pipeline import generate                # noqa: E402

REPO = os.path.abspath(os.path.join(HERE, '..', '..', '..'))
DEMO_MAP = os.path.join(REPO, 'maps', 'stores', 'demo_store', 'map.yaml')
DEMO = json.load(open(os.path.join(HERE, '..', 'examples', 'labels_demo.json')))


def pinned(**stops):
    d = copy.deepcopy(DEMO)
    for lab in d['labels']:
        if lab['id'] in stops:
            lab['stop'] = stops[lab['id']]
    return d


def stop(x, y, yaw=None):
    s = {'x_m': x, 'y_m': y}
    if yaw is not None:
        s['yaw_deg'] = yaw
    return s


def reason(res, lid):
    return next((x['reason'] for x in res.excluded if x['id'] == lid), None)


BASE = generate(DEMO_MAP, DEMO)


def same(q, b):
    return all(abs(q[k] - b[k]) < 1e-6 for k in ('x', 'y', 'yaw_deg'))


def test_without_stop_nothing_changes():
    res = generate(DEMO_MAP, pinned())
    assert {k: p['node'] for k, p in res.poses.items()} == {k: p['node'] for k, p in BASE.poses.items()}
    assert all(same(p, BASE.poses[k]) for k, p in res.poses.items())
    assert not any(p['pinned'] for p in res.poses.values())


def test_a_good_pin_moves_only_that_stop():
    res = generate(DEMO_MAP, pinned(table_01=stop(-1.8, -0.4, 90)))
    p = res.poses['table_01']
    assert p['pinned'] and abs(p['x'] + 1.8) < 1e-6 and abs(p['y'] + 0.375) < 1e-6, p
    assert p['yaw_deg'] == 90.0 and p['level'] in ('OK', 'WARN')
    assert p['node'] != res.poses['table_03']['node']          # no longer shares the node
    for k in ('table_02', 'table_03', 'table_04', 'kitchen', 'standby_01'):
        q, b = res.poses[k], BASE.poses[k]
        assert same(q, b), k
        assert not q['pinned']
    assert not res.excluded and res.count('FAIL') == 0


def test_the_pin_snaps_to_the_nearest_corridor_point():
    res = generate(DEMO_MAP, pinned(table_01=stop(-1.8, 0.1, 90)))      # 0.5 m off the corridor
    p = res.poses['table_01']
    assert abs(p['x'] + 1.8) < 1e-6 and abs(p['y'] + 0.375) < 1e-6, p


def test_a_pin_close_to_a_node_uses_the_node():
    res = generate(DEMO_MAP, pinned(table_01=stop(-1.55, -0.4, 90)))
    assert res.poses['table_01']['node'] == res.poses['table_03']['node']
    assert len(res.graph.nodes) == len(BASE.graph.nodes)


def test_direction_can_be_left_out_and_then_faces_the_target():
    res = generate(DEMO_MAP, pinned(table_01=stop(-1.62, -0.4)))     # nearly straight below the table
    assert res.poses['table_01']['yaw_deg'] == 90.0 and res.poses['table_01']['pinned']


def test_a_wrong_direction_is_refused_not_replaced():
    res = generate(DEMO_MAP, pinned(table_01=stop(-1.6, -0.375, 180)))
    assert 'table_01' not in res.poses and 'pinned' in reason(res, 'table_01'), res.excluded
    # nothing else moved and the failed pin left no extra node behind
    assert same(res.poses['table_03'], BASE.poses['table_03'])
    assert len(res.graph.nodes) == len(BASE.graph.nodes) and len(res.graph.edges) == len(BASE.graph.edges)


def test_a_slanted_direction_is_refused():
    res = generate(DEMO_MAP, pinned(table_01=stop(-1.6, -0.375, 45)))
    assert 'table_01' not in res.poses and 'not east' in reason(res, 'table_01')


def test_too_far_from_every_corridor_is_refused():
    res = generate(DEMO_MAP, pinned(table_01=stop(30, 30, 90)))
    assert 'table_01' not in res.poses and 'no corridor within' in reason(res, 'table_01')


def test_facing_away_from_the_target_is_refused():
    res = generate(DEMO_MAP, pinned(table_01=stop(-1.6, -0.375, -90)))   # towards table_03
    assert 'table_01' not in res.poses and 'pinned stop' in reason(res, 'table_01'), res.excluded


def test_a_pinned_standby_keeps_the_chosen_direction():
    res = generate(DEMO_MAP, pinned(standby_01=stop(-3.16, -1.0, 90)))
    p = res.poses['standby_01']
    assert p['pinned'] and abs(p['x'] + 3.158) < 0.01 and abs(p['y'] + 1.0) < 1e-6 and p['yaw_deg'] == 90.0, p


def test_the_pin_is_kept_in_the_labels_and_checked():
    lab = load_labels(pinned(table_01=stop(-1.8, -0.4, 90)))['labels']
    assert next(x for x in lab if x['id'] == 'table_01')['stop'] == {'x': -1.8, 'y': -0.4, 'yaw_deg': 90.0}
    assert 'stop' not in next(x for x in lab if x['id'] == 'table_02')
    for bad in ({'x_m': 1}, {'x_m': 'a', 'y_m': 1}, {'x_m': float('nan'), 'y_m': 1}, 5):
        try:
            load_labels(pinned(table_01=bad))
        except LabelError as e:
            assert 'stop' in str(e), e
        else:
            raise AssertionError('expected LabelError for %r' % (bad,))


if __name__ == '__main__':
    n = 0
    for k, v in sorted(globals().items()):
        if k.startswith('test_') and callable(v):
            v()
            n += 1
            print('ok', k)
    print('all passed' if n else 'no tests')
