#!/usr/bin/env python3
"""C-4 (server half): drive-test state, proposals, decisions, adjusted generate/export.
Run: python3 tools/site_builder/tests/test_c4_server.py"""
import json, os, shutil, sys, tempfile, threading, http.client

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, '..'))
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from _fixtures import ensure_demo_fixtures   # noqa: E402
ensure_demo_fixtures()
from site_builder.server import App, serve   # noqa: E402

REPO = os.path.abspath(os.path.join(HERE, '..', '..', '..'))
MAP = os.path.join(REPO, 'maps', 'stores', 'demo_store', 'map.yaml')
EXAMPLE = os.path.join(HERE, '..', 'examples', 'labels_demo.json')
DEMO = json.load(open(EXAMPLE))
TOUR = json.load(open(os.path.join(HERE, '..', 'out', 'drive_demo', 'tour.json')))
TOUR['candidate_dir'] = os.path.abspath(os.path.join(HERE, '..', 'out', 'gen_demo'))   # portable


class Srv:
    def __init__(self, tmp, with_report=True):
        self.out = os.path.join(tmp, 'generated'); self.drive = os.path.join(tmp, 'drive')
        os.makedirs(self.drive)
        self.labels = os.path.join(tmp, 'work', 'labels.json')
        os.makedirs(os.path.dirname(self.labels))
        json.dump(DEMO, open(self.labels, 'w'))
        json.dump(TOUR, open(os.path.join(self.drive, 'tour.json'), 'w'))
        if with_report:
            steps = {}
            for st in TOUR['steps']:
                bad = st['edge'] == 'e08'                      # e08 fails both ways
                lopsided = st['edge'] in ('e01', 'e03')        # e01+e03 are off-centre
                fwd = st['from'] < st['to']
                l, r = (0.60, 0.40) if lopsided else (0.5, 0.5)
                if not fwd:
                    l, r = r, l
                steps[str(st['i'])] = {'i': st['i'], 'edge': st['edge'], 'from': st['from'], 'to': st['to'],
                    'verdict': 'BLOCKED' if bad else 'PASS', 'flags': [], 'reasons': ['alarm BLOCKED'] if bad else [],
                    'obs': {'min_left_m': l, 'min_right_m': r}}
            json.dump({'tour_hash': TOUR['hash'], 'steps': steps}, open(os.path.join(self.drive, 'test_report.json'), 'w'))
        self.app = App(MAP, self.labels, self.out, EXAMPLE, self.drive)
        self.httpd = serve(self.app, 0); self.port = self.httpd.server_address[1]
        threading.Thread(target=self.httpd.serve_forever, daemon=True).start()

    def j(self, method, path, body=None):
        c = http.client.HTTPConnection('127.0.0.1', self.port, timeout=90)
        c.request(method, path, None if body is None else json.dumps(body).encode(), {'Content-Type': 'application/json'})
        r = c.getresponse(); raw = r.read(); c.close()
        return r.status, json.loads(raw)

    def close(self):
        self.httpd.shutdown(); self.httpd.server_close()


def test_state_propose_decide_apply():
    with tempfile.TemporaryDirectory() as tmp:
        s = Srv(tmp)
        try:
            st, d = s.j('GET', '/api/drive')
            assert st == 200 and d['available'] and d['steps_done'] == d['steps_total'] == 32, d
            assert d['counts']['FAIL'] == 2 and not d['labels_changed'] and d['adjustments'] is None
            assert all(len(e['dirs']) == 2 for e in d['edges'])
            st, ad = s.j('POST', '/api/drive/propose', {})
            kinds = sorted(p['kind'] for p in ad['proposals'])
            assert st == 200 and kinds == ['remove_edge', 'shift_line'], ad
            assert all(p['status'] == 'proposed' for p in ad['proposals'])
            # nothing approved yet: adjusted generate equals the plain one
            plain = s.j('POST', '/api/generate', {'labels': DEMO})[1]
            same = s.j('POST', '/api/generate', {'labels': DEMO, 'adjusted': True})[1]
            assert same['adjustments'] is None and same['nodes'] == plain['nodes']
            # approve both, apply
            ids = {p['kind']: p['id'] for p in ad['proposals']}
            st, ad2 = s.j('PUT', '/api/drive/decide', {'decisions': {ids['shift_line']: 'approved', ids['remove_edge']: 'approved'}})
            assert st == 200
            adj = s.j('POST', '/api/generate', {'labels': DEMO, 'adjusted': True})[1]
            assert sorted(adj['adjustments']['applied']) == sorted(ids.values()), adj['adjustments']
            assert adj['nodes'] != plain['nodes'] and len(adj['edges']) < len(plain['edges'])
            # propose again: decisions on identical proposals are kept
            ad3 = s.j('POST', '/api/drive/propose', {})[1]
            assert sorted(p['status'] for p in ad3['proposals']) == ['approved', 'approved'], ad3
            # export applies them and records it; common is never written
            st, ex = s.j('POST', '/api/export', {'labels': DEMO, 'adjusted': True})
            assert st == 200 and ex['exported_to'], ex
            assert os.path.isfile(os.path.join(s.out, 'adjustments_applied.json'))
            # a plain export removes the record again
            s.j('POST', '/api/export', {'labels': DEMO})
            assert not os.path.exists(os.path.join(s.out, 'adjustments_applied.json'))
        finally:
            s.close()


def test_decide_rules_and_stale():
    with tempfile.TemporaryDirectory() as tmp:
        s = Srv(tmp)
        try:
            ad = s.j('POST', '/api/drive/propose', {})[1]
            pid = ad['proposals'][0]['id']
            assert s.j('PUT', '/api/drive/decide', {'decisions': {'nope': 'approved'}})[0] == 400
            assert s.j('PUT', '/api/drive/decide', {'decisions': {pid: 'maybe'}})[0] == 400
            assert s.j('PUT', '/api/drive/decide', {'decisions': 5})[0] == 400
            s.j('PUT', '/api/drive/decide', {'decisions': {pid: 'approved'}})
            # the labels change -> the old proposals are stale and refused
            other = json.loads(json.dumps(DEMO))
            other['labels'] = [l for l in other['labels'] if l['id'] != other['labels'][-1]['id']]
            json.dump(other, open(s.labels, 'w'))
            d = s.j('GET', '/api/drive')[1]
            assert d['labels_changed'] and d['adjustments_stale'], d
            st, err = s.j('POST', '/api/generate', {'labels': other, 'adjusted': True})
            assert st == 400 and 'another graph' in err['error'], err
        finally:
            s.close()


def test_review_edge_cannot_be_approved():
    with tempfile.TemporaryDirectory() as tmp:
        s = Srv(tmp)
        try:
            rp = os.path.join(s.drive, 'test_report.json')
            rep = json.load(open(rp))
            one = next(k for k, r in rep['steps'].items() if r['edge'] == 'e05' and r['from'] < r['to'])
            rep['steps'][one].update(verdict='HARDSTOP', reasons=['x'])
            json.dump(rep, open(rp, 'w'))
            ad = s.j('POST', '/api/drive/propose', {})[1]
            rv = [p for p in ad['proposals'] if p['kind'] == 'review_edge']
            assert len(rv) == 1 and rv[0]['edge'] == 'e05', ad
            st, err = s.j('PUT', '/api/drive/decide', {'decisions': {rv[0]['id']: 'approved'}})
            assert st == 400 and 'cannot be approved' in err['error'], err
        finally:
            s.close()


def test_no_report_and_no_drive():
    with tempfile.TemporaryDirectory() as tmp:
        s = Srv(tmp, with_report=False)
        try:
            d = s.j('GET', '/api/drive')[1]
            assert d['available'] and d['steps_done'] == 0
            assert s.j('POST', '/api/drive/propose', {})[0] == 400
            assert s.j('PUT', '/api/drive/decide', {'decisions': {}})[0] == 400
            assert s.j('POST', '/api/generate', {'labels': DEMO, 'adjusted': True})[0] == 400
        finally:
            s.close()
    with tempfile.TemporaryDirectory() as tmp:
        app = App(MAP, os.path.join(tmp, 'l.json'), os.path.join(tmp, 'o'), EXAMPLE, None)
        httpd = serve(app, 0)
        threading.Thread(target=httpd.serve_forever, daemon=True).start()
        c = http.client.HTTPConnection('127.0.0.1', httpd.server_address[1]); c.request('GET', '/api/drive')
        assert json.loads(c.getresponse().read())['available'] is False
        httpd.shutdown()


if __name__ == '__main__':
    for n, f in sorted(globals().items()):
        if n.startswith('test_'):
            f(); print('ok', n)
    print('all passed')
