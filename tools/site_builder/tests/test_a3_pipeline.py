#!/usr/bin/env python3
"""A-3: labels -> stop poses, HardStop pre-check, outputs.
Run: python3 tools/site_builder/tests/test_a3_pipeline.py"""
import json
import math
import os
import sys
import tempfile

import numpy as np
import yaml

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
sys.path.insert(0, os.path.join(HERE, '..'))
from test_a2_graph import build   # noqa: E402  (synthetic map writer)
from site_builder.labels import LabelError, load_labels   # noqa: E402
from site_builder.params import load_params   # noqa: E402
from site_builder.pipeline import PipelineError, generate, write_outputs   # noqa: E402

REPO = os.path.abspath(os.path.join(HERE, '..', '..', '..'))
DEMO_MAP = os.path.join(REPO, 'maps', 'stores', 'demo_store', 'map.yaml')
DEMO_LABELS = os.path.join(HERE, '..', 'examples', 'labels_demo.json')
sys.path.insert(0, os.path.join(REPO, 'ros_ws', 'src', 'serving_robot_navigation'))


def lab(i, kind, x, y, **kw):
    d = {'id': i, 'kind': kind, 'x_m': x, 'y_m': y}
    d.update(kw)
    return d


def doc(*labels, **kw):
    d = {'schema_version': 1, 'labels': list(labels)}
    d.update(kw)
    return d


def raises(fn, exc, text=''):
    try:
        fn()
    except exc as e:
        assert text in str(e), str(e)
        return
    raise AssertionError('expected %s' % exc.__name__)


def test_labels_are_checked():
    ok = lab('standby_01', 'standby', 0, 0)
    raises(lambda: load_labels(doc(lab('a', 'chair', 0, 0), ok)), LabelError, 'kind')
    raises(lambda: load_labels(doc(ok, ok)), LabelError, 'duplicate')
    raises(lambda: load_labels(doc(lab('T 1', 'table', 0, 0), ok)), LabelError, 'id')
    raises(lambda: load_labels(doc(lab('t', 'table', 0, 0))), LabelError, '대기(홈) 점')
    raises(lambda: load_labels(doc(lab('t', 'table', float('nan'), 0), ok)), LabelError, 'finite')
    raises(lambda: load_labels(doc(ok, map_revision=2), map_revision=1), LabelError, 'map_revision')
    raises(lambda: load_labels({'schema_version': 2, 'labels': []}), LabelError, 'schema')


def test_params_come_from_the_project_files():
    p = load_params()
    assert abs(p.corner_radius_m - 0.4686) < 1e-3, p.corner_radius_m
    assert p.hard_stop_m == 0.10 and abs(p.front_m - 0.26) < 1e-9


def test_demo_labels_reproduce_the_manual_destinations():
    res = generate(DEMO_MAP, DEMO_LABELS)
    man = yaml.safe_load(open(os.path.join(REPO, 'config/common/waypoints.yaml')))
    P = res.poses
    assert set(P) == {'standby_01', 'kitchen', 'table_01', 'table_02', 'table_03', 'table_04'}, set(P)
    assert not res.excluded, res.excluded
    # same shared nodes and facing as the hand-made grid
    assert P['table_01']['node'] == P['table_03']['node'] and P['table_02']['node'] == P['table_04']['node']
    assert (P['table_01']['yaw_deg'], P['table_03']['yaw_deg']) == (90.0, -90.0)
    assert (P['table_02']['yaw_deg'], P['table_04']['yaw_deg']) == (90.0, -90.0)
    for name, p in P.items():
        mn = man['nodes'][man['poses'][name]['node']]
        d = math.hypot(p['x'] - mn['x'], p['y'] - mn['y'])
        assert d <= 0.15, (name, d)
        assert abs(p['yaw_deg'] - man['poses'][name]['yaw_deg']) < 1e-6, name
    assert P['standby_01']['aliases'] == ['wait_01', 'home'] and P['kitchen']['aliases'] == ['desk']
    assert res.count('FAIL') == 0
    # the tight kitchen aisle is reported as the validator reports it (room 0.50 m)
    assert res.node_checks[P['kitchen']['node']]['level'] == 'WARN'


def test_outputs_load_with_the_real_site_loader():
    from serving_robot_navigation.site_config import load_site
    res = generate(DEMO_MAP, DEMO_LABELS)
    with tempfile.TemporaryDirectory() as d:
        write_outputs(res, d)
        site = load_site(d)
        assert len(site.nodes) == len(res.graph.nodes) and len(site.edges) == len(res.graph.edges)
        for e in site.edges.values():
            assert site.zone_at(*site.nodes[e.a]) is not None, e.id
        assert site.poses['standby_01'].kind == 'standby'
        pv = json.load(open(os.path.join(d, 'preview.json')))
        assert pv['summary']['FAIL'] == 0 and pv['map_revision'] == 1


def test_front_gap_limit_is_applied():
    res = generate(DEMO_MAP, DEMO_LABELS, load_params(gap_hi_m=0.30))
    gone = {x['id'] for x in res.excluded}
    assert 'table_02' in gone and 'table_02' not in res.poses, res.excluded
    assert 'table_02' in generate(DEMO_MAP, DEMO_LABELS).poses
    assert 'kitchen' in res.poses and 'standby_01' in res.poses


def validator_levels(map_yaml, cfg_dir):
    """Run the real scripts/validate_site_config.py; {'node n01': 'OK', ...} and the exit code."""
    import subprocess
    r = subprocess.run([sys.executable, os.path.join(REPO, 'scripts', 'validate_site_config.py'),
                        '--config-dir', cfg_dir, '--map', map_yaml, '--world', 'none'],
                       capture_output=True, text=True, env=dict(os.environ, PYTHONDONTWRITEBYTECODE='1'))
    out = {}
    for line in r.stdout.splitlines():
        if line.startswith('['):
            lvl = line[1:line.index(']')].strip()
            rest = line[line.index(']') + 1:].split()
            if len(rest) >= 2 and rest[0] in ('node', 'edge', 'pose'):
                out['%s %s' % (rest[0], rest[1])] = lvl
    return out, r.returncode, r.stdout


def check_against_validator(map_yaml, res):
    with tempfile.TemporaryDirectory() as d:
        write_outputs(res, d)
        lv, code, text = validator_levels(map_yaml, d)
    assert code == 0, text
    for nid, c in res.node_checks.items():
        assert lv['node ' + nid] == c['level'], (nid, lv['node ' + nid], c)
    for name in res.poses:
        assert lv['pose ' + name] in ('OK', 'WARN'), (name, text)
    return lv


def test_generated_nodes_get_the_same_verdict_from_the_real_validator():
    check_against_validator(DEMO_MAP, generate(DEMO_MAP, DEMO_LABELS))


def l_map(tmp):
    """L corridor 1.2 m wide, an unlabelled branch to the north, a far island."""
    h, w = 120, 160
    free = [(40, 64, 10, 110),        # horizontal leg
            (40, 110, 86, 110),       # vertical leg (down)
            (10, 40, 40, 64),         # dead-end branch, no label
            (95, 118, 10, 40)]        # island, far from the corridor
    return build(tmp, free_rects=free, h=h, w=w)


def test_other_shape_l_corridor():
    with tempfile.TemporaryDirectory() as tmp:
        mp = l_map(tmp)
        x_left, y_mid = mp.xy_of(52, 12)
        _, y_north = mp.xy_of(30, 70)         # inside the north wall of the horizontal leg
        x_east, y_low = mp.xy_of(90, 116)     # inside the east wall of the vertical leg
        labels = doc(lab('standby_01', 'standby', x_left, y_mid),
                     lab('kitchen', 'pickup', mp.xy_of(52, 70)[0], mp.xy_of(34, 70)[1]),
                     lab('table_01', 'table', x_east + 0.2, y_low),
                     lab('standby_02', 'standby', *mp.xy_of(105, 20)))
        res = generate(os.path.join(tmp, 'm.yaml'), labels)
        P = res.poses
        assert set(P) == {'standby_01', 'kitchen', 'table_01'}, set(P)
        assert [x['id'] for x in res.excluded] == ['standby_02'], res.excluded
        assert P['kitchen']['yaw_deg'] == 90.0 and P['table_01']['yaw_deg'] == 0.0
        assert P['standby_01']['yaw_deg'] == 0.0
        # the unlabelled dead-end branch to the north is gone
        top = mp.xy_of(45, 0)[1]
        assert all(y < top for _, y in res.graph.nodes.values()), res.graph.nodes
        assert res.count('FAIL') == 0, res.node_checks
        check_against_validator(os.path.join(tmp, 'm.yaml'), res)


def test_other_shape_cross_corridors():
    with tempfile.TemporaryDirectory() as tmp:
        mp = build(tmp, free_rects=[(50, 74, 10, 150), (10, 110, 68, 92)], h=120, w=160)
        labels = doc(lab('standby_01', 'standby', *mp.xy_of(62, 12)),
                     lab('table_01', 'table', *mp.xy_of(30, 59)),     # wall west of the vertical arm
                     lab('table_02', 'table', *mp.xy_of(90, 101)))    # wall east of the vertical arm
        res = generate(os.path.join(tmp, 'm.yaml'), labels)
        assert set(res.poses) == {'standby_01', 'table_01', 'table_02'}, res.excluded
        assert res.poses['table_01']['yaw_deg'] == 180.0 and res.poses['table_02']['yaw_deg'] == 0.0
        assert res.count('FAIL') == 0
        check_against_validator(os.path.join(tmp, 'm.yaml'), res)


def test_hardstop_turn_check_removes_a_too_tight_corridor():
    with tempfile.TemporaryDirectory() as tmp:
        mp = build(tmp, free_rects=[(40, 58, 10, 110)], h=100, w=130)   # 0.9 m wide
        labels = doc(lab('standby_01', 'standby', *mp.xy_of(49, 12)),
                     lab('kitchen', 'pickup', *mp.xy_of(49, 108)))
        raises(lambda: generate(os.path.join(tmp, 'm.yaml'), labels), PipelineError, 'standby')


if __name__ == '__main__':
    n = 0
    for k, v in sorted(globals().items()):
        if k.startswith('test_') and callable(v):
            v()
            n += 1
            print('ok', k)
    print('all passed' if n else 'no tests')
