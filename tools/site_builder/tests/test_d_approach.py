#!/usr/bin/env python3
"""D: approach branches (a straight spur from the corridor node towards a table).
The wide-hall map is built here for the tests only; it is not a store map.
Run: python3 tools/site_builder/tests/test_d_approach.py"""
import math
import os
import sys
import tempfile

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
sys.path.insert(0, os.path.join(HERE, '..'))
from test_a2_graph import build   # noqa: E402
from test_a3_pipeline import DEMO_LABELS, DEMO_MAP, check_against_validator, doc, lab   # noqa: E402
from site_builder.params import Params   # noqa: E402
from site_builder.pipeline import generate, to_preview   # noqa: E402


def on():
    par = Params()
    par.approach = True      # off by default (not yet tested on a robot)
    return par


def hall(tmp, table_row):
    """2.0 m wide hall (rows 40..80); the table sits `table_row` (the lower, the farther away)."""
    mp = build(tmp, free_rects=[(40, 80, 10, 150)], h=120, w=160)
    labels = doc(lab('standby_01', 'standby', *mp.xy_of(60, 14)),
                 lab('table_01', 'table', *mp.xy_of(table_row, 80)))
    return os.path.join(tmp, 'm.yaml'), labels


def test_far_table_gets_a_branch_to_the_wanted_gap():
    with tempfile.TemporaryDirectory() as tmp:
        m, labels = hall(tmp, 30)
        res = generate(m, labels, on())
        a = res.approach['table_01']
        assert a['branch_m'] >= 0.30, a
        # the dock is where the robot stops: wanted gap = max(30 cm, room to turn round)
        assert abs(a['gap_m'] - a['gap_goal_m']) < 0.03, a
        n = res.poses['table_01']['node']
        assert n == a['node'] and n != a['from_node']
        edges = [e for e in res.graph.edges if n in (e['a'], e['b'])]
        assert len(edges) == 1 and edges[0].get('branch'), edges
        assert res.poses['table_01']['yaw_deg'] == 90.0
        assert res.count('FAIL') == 0, res.node_checks
        check_against_validator(m, res)


def test_branch_is_shorter_than_min_so_none_is_built():
    with tempfile.TemporaryDirectory() as tmp:
        m, labels = hall(tmp, 38)
        res = generate(m, labels, on())
        a = res.approach['table_01']
        assert a['branch_m'] == 0.0 and a.get('reason'), a
        assert res.poses['table_01']['node'] == a['node']
        assert not any(e.get('branch') for e in res.graph.edges)


def test_default_is_off():
    with tempfile.TemporaryDirectory() as tmp:
        m, labels = hall(tmp, 30)
        res = generate(m, labels)
        assert res.approach == {} and not any(e.get('branch') for e in res.graph.edges)


def test_switch_off_gives_the_old_result():
    with tempfile.TemporaryDirectory() as tmp:
        m, labels = hall(tmp, 30)
        par = Params()
        par.approach = False
        res = generate(m, labels, par)
        assert res.approach == {}
        assert not any(e.get('branch') for e in res.graph.edges)


def test_serve_gap_setting_is_used():
    with tempfile.TemporaryDirectory() as tmp:
        m, labels = hall(tmp, 30)
        par = on()
        par.serve_gap_m = 0.40
        a = generate(m, labels, par).approach['table_01']
        assert abs(a["gap_m"] - 0.40) < 0.03, a


def test_demo_map_has_no_branch_and_notes_why():
    res = generate(DEMO_MAP, DEMO_LABELS, on())
    assert set(res.approach) == {'table_01', 'table_02', 'table_03', 'table_04'}
    assert all(v['branch_m'] == 0.0 and v['reason'] for v in res.approach.values()), res.approach
    assert not any(e.get('branch') for e in res.graph.edges)


def test_preview_carries_the_approach_notes():
    with tempfile.TemporaryDirectory() as tmp:
        m, labels = hall(tmp, 30)
        pv = to_preview(generate(m, labels, on()))
        assert pv['approach']['table_01']['branch_m'] >= 0.30


if __name__ == '__main__':
    n = 0
    for k, v in sorted(globals().items()):
        if k.startswith('test_') and callable(v):
            v()
            n += 1
            print('ok', k)
    print(n, 'passed')
