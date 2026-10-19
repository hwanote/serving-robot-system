"""C-3: proposals from measured results, and their deterministic application."""
import copy, json, os, sys, tempfile, unittest, filecmp
HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, '..'))
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from _fixtures import ensure_demo_fixtures   # noqa: E402
ensure_demo_fixtures()
import yaml
from site_builder import adjust
from site_builder.params import load_params
from site_builder.pipeline import generate, write_outputs, PipelineError

REPO = os.path.abspath(os.path.join(HERE, '..', '..', '..'))
MAP = os.path.join(REPO, 'maps', 'stores', 'demo_store', 'map.yaml')
LABELS = os.path.join(HERE, '..', 'examples', 'labels_demo.json')
CAND = os.path.join(HERE, '..', 'out', 'gen_demo')


def wp():
    return yaml.safe_load(open(os.path.join(CAND, 'waypoints.yaml')))


def rec(w, eid, fwd=True, verdict='PASS', flags=(), left=0.5, right=0.5, reasons=()):
    e = next(x for x in w['edges'] if x['id'] == eid)
    a, b = (e['from'], e['to']) if fwd else (e['to'], e['from'])
    return {'i': 0, 'edge': eid, 'from': a, 'to': b, 'verdict': verdict, 'flags': list(flags),
            'reasons': list(reasons), 'obs': {'min_left_m': left, 'min_right_m': right}}


def report(*recs):
    return {'tour_hash': 'x', 'steps': {str(i): r for i, r in enumerate(recs)}}


def base_report(w, **over):
    """every edge fine and centred, except what the test overrides: {eid: (fwd_rec, rev_rec)}"""
    recs = []
    for e in w['edges']:
        if e['id'] in over:
            recs += list(over[e['id']])
        else:
            recs += [rec(w, e['id'], True), rec(w, e['id'], False)]
    return report(*recs)


class T(unittest.TestCase):
    def test_centred_gives_nothing(self):
        w = wp()
        self.assertEqual(adjust.propose(w, base_report(w))['proposals'], [])

    def test_shift_sign_size_and_cap(self):
        w = wp()
        # e01 runs n01 (-3.158,-2.375) -> n02 (-3.158,-0.375): heading +90 deg, robot-left is -x.
        # room on the left 0.60, on the right 0.40 -> move 0.10 towards the left = x - 0.10
        both = lambda e, l, rr: (rec(w, e, True, left=l, right=rr), rec(w, e, False, left=rr, right=l))
        r = base_report(w, e01=both('e01', 0.60, 0.40), e03=both('e03', 0.60, 0.40))
        p = adjust.propose(w, r)['proposals']
        sh = [x for x in p if x['kind'] == 'shift_line']
        self.assertEqual(len(sh), 1)
        self.assertAlmostEqual(sh[0]['d'][0], -0.10, places=3); self.assertEqual(sh[0]['d'][1], 0.0)
        self.assertEqual(sh[0]['status'], 'proposed')
        r = base_report(w, e01=both('e01', 1.5, 0.4), e03=both('e03', 1.5, 0.4))
        sh = [x for x in adjust.propose(w, r)['proposals'] if x['kind'] == 'shift_line'][0]
        self.assertAlmostEqual(sh['d'][0], -0.15, places=3)
        r = base_report(w, e01=both('e01', 0.52, 0.48), e03=both('e03', 0.52, 0.48))
        self.assertEqual(adjust.propose(w, r)['proposals'], [])     # 2 cm: below the threshold
        # one tight section, the rest centred: the shift is a compromise, not the full 10 cm
        r = base_report(w, e01=both('e01', 0.60, 0.40))
        sh = [x for x in adjust.propose(w, r)['proposals'] if x['kind'] == 'shift_line'][0]
        self.assertAlmostEqual(sh['d'][0], -0.05, places=2)

    def test_failures(self):
        w = wp()
        bad = lambda f: rec(w, 'e08', f, verdict='BLOCKED', reasons=['alarm BLOCKED'])
        one = adjust.propose(w, base_report(w, e08=(bad(True), rec(w, 'e08', False))))['proposals']
        self.assertEqual([p['kind'] for p in one], ['review_edge'])
        both = adjust.propose(w, base_report(w, e08=(bad(True), bad(False))))['proposals']
        rm = [p for p in both if p['kind'] == 'remove_edge']
        self.assertEqual(len(rm), 1); self.assertEqual(rm[0]['edge'], 'e08')
        self.assertFalse([p for p in both if p['kind'] == 'shift_line' and 'e08' in p['edges']])
        # n06 only hangs on e08: removing it cuts off whatever is posed on n06 (if anything) - list must be a list
        self.assertIsInstance(rm[0]['loses'], list)

    def test_unclean_shift_is_only_a_review(self):
        w = wp()
        nodes = {k: (v['x'], v['y']) for k, v in w['nodes'].items()}
        ax, S, E = adjust.chain_of(nodes, w['edges'], 'e12')      # y = 2.025, ends at the slanted e14
        self.assertFalse(adjust.shift_is_clean(nodes, w['edges'], S, ax))
        both = lambda e, l, rr: (rec(w, e, True, left=l, right=rr), rec(w, e, False, left=rr, right=l))
        p = adjust.propose(w, base_report(w, e12=both('e12', 0.9, 0.4)))['proposals']
        self.assertFalse([x for x in p if x['kind'] == 'shift_line' and 'e12' in x['edges']])
        self.assertTrue([x for x in p if x['kind'] == 'review_edge'])

    def run_gen(self, adj=None, out=None):
        res = generate(MAP, LABELS, load_params(), adjustments=adj)
        if out:
            write_outputs(res, out)
        return res

    def test_base_hash_matches_written_waypoints(self):
        res = self.run_gen()
        self.assertEqual(adjust.hash_of_graph(res.graph), adjust.hash_waypoints(wp()))

    def test_apply_shift_exactly_and_deterministic(self):
        base = self.run_gen()
        w = wp()
        adj = {'base_hash': adjust.hash_waypoints(w), 'proposals': [
            {'id': 'adj01', 'kind': 'shift_line', 'status': 'approved', 'nodes': ['n01', 'n02', 'n03'],
             'edges': ['e01', 'e03'], 'd': [0.05, 0.0]}]}
        o1, o2 = tempfile.mkdtemp(), tempfile.mkdtemp()
        r1 = self.run_gen(adj, o1); self.run_gen(adj, o2)
        for f in ('waypoints.yaml', 'entities.yaml', 'zones.yaml', 'preview.json', 'adjustments_applied.json'):
            self.assertTrue(filecmp.cmp(os.path.join(o1, f), os.path.join(o2, f), shallow=False), f)
        self.assertEqual(r1.adjust_info['applied'], ['adj01'])
        a = {k: v for k, v in r1.graph.nodes.items()}
        b = base.graph.nodes
        moved = [k for k in a if k in b and abs(a[k][0] - b[k][0] - 0.05) < 1e-6 and abs(a[k][1] - b[k][1]) < 1e-6]
        self.assertEqual(len(moved), 3)
        for e in r1.graph.edges:       # still straight along an axis
            p, q = r1.graph.nodes[e['a']], r1.graph.nodes[e['b']]
            if e['id'] in ('e01', 'e03'):
                self.assertLess(abs(p[0] - q[0]), 1e-6)

    def test_not_approved_changes_nothing(self):
        w = wp()
        adj = {'base_hash': adjust.hash_waypoints(w), 'proposals': [
            {'id': 'adj01', 'kind': 'shift_line', 'status': 'proposed', 'nodes': ['n01', 'n02', 'n03'], 'edges': [], 'd': [0.05, 0.0]},
            {'id': 'adj02', 'kind': 'remove_edge', 'status': 'rejected', 'edge': 'e08'}]}
        o = tempfile.mkdtemp(); self.run_gen(adj, o)
        self.assertTrue(filecmp.cmp(os.path.join(o, 'waypoints.yaml'), os.path.join(CAND, 'waypoints.yaml'), shallow=False))
        self.assertFalse(os.path.exists(os.path.join(o, 'adjustments_applied.json')))

    def test_stale_base_refused(self):
        adj = {'base_hash': 'deadbeef0000', 'proposals': [{'id': 'a', 'kind': 'remove_edge', 'status': 'approved', 'edge': 'e08'}]}
        with self.assertRaises(adjust.AdjustError):
            self.run_gen(adj)

    def test_shift_into_wall_rejected(self):
        w = wp()
        adj = {'base_hash': adjust.hash_waypoints(w), 'proposals': [
            {'id': 'adj01', 'kind': 'shift_line', 'status': 'approved', 'nodes': ['n01', 'n02', 'n03'], 'edges': [], 'd': [-0.9, 0.0]}]}
        r = self.run_gen(adj)
        self.assertEqual(r.adjust_info['applied'], [])
        self.assertEqual(r.adjust_info['rejected'][0]['id'], 'adj01')
        self.assertTrue(any('adj01' in x for x in r.warnings))
        self.assertAlmostEqual(r.graph.nodes['n01'][0], -3.158, places=3)

    def test_shift_that_only_adds_a_warning_is_rejected(self):
        """the real case: e02 y -0.05 m passes the FAIL limits but turns the end nodes n01 and n06 into WARN"""
        w = wp()
        base = self.run_gen()
        adj = {'base_hash': adjust.hash_waypoints(w), 'proposals': [
            {'id': 'adj01', 'kind': 'shift_line', 'status': 'approved', 'nodes': ['n01', 'n06'],
             'edges': ['e02'], 'd': [0.0, -0.05]}]}
        r = self.run_gen(adj)
        self.assertEqual(r.adjust_info['applied'], [])
        self.assertEqual(r.adjust_info['rejected'][0]['id'], 'adj01')
        reason = r.adjust_info['rejected'][0]['reason']
        self.assertIn('n01', reason)
        self.assertIn('새로 주의', reason)
        self.assertEqual(r.graph.nodes['n01'], base.graph.nodes['n01'])     # nothing moved
        self.assertEqual({k: r.count(k) for k in ('OK', 'WARN', 'FAIL')},
                         {k: base.count(k) for k in ('OK', 'WARN', 'FAIL')})

    def test_shift_that_changes_no_level_is_still_applied(self):
        w = wp()
        adj = {'base_hash': adjust.hash_waypoints(w), 'proposals': [
            {'id': 'adj01', 'kind': 'shift_line', 'status': 'approved', 'nodes': ['n01', 'n02', 'n03'],
             'edges': ['e01', 'e03'], 'd': [0.05, 0.0]}]}
        self.assertEqual(self.run_gen(adj).adjust_info['applied'], ['adj01'])

    def test_remove_edge_cuts_and_excludes(self):
        w = wp()
        adj = {'base_hash': adjust.hash_waypoints(w), 'proposals': [
            {'id': 'adj01', 'kind': 'remove_edge', 'status': 'approved', 'edge': 'e11'}]}
        base = self.run_gen(); r = self.run_gen(adj)
        self.assertLess(len(r.graph.edges), len(base.graph.edges))
        self.assertEqual(r.adjust_info['applied'], ['adj01'])
        for name, p in r.poses.items():
            self.assertIn(p['node'], r.graph.nodes)

    def test_adjusted_candidate_gets_no_new_shift(self):
        w = wp()
        r = base_report(w, e01=(rec(w, 'e01', True, left=0.9, right=0.4), rec(w, 'e01', False, left=0.4, right=0.9)))
        p = adjust.propose(w, r, applied_info={'applied': ['adj01']})
        self.assertFalse([x for x in p['proposals'] if x['kind'] in ('shift_line', 'remove_edge')])
        self.assertTrue(p['notes'])


if __name__ == '__main__':
    unittest.main()
