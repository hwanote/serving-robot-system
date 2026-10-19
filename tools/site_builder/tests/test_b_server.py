#!/usr/bin/env python3
"""B: the local server behind the labelling screen.
Run: python3 tools/site_builder/tests/test_b_server.py"""
import http.client
import json
import os
import sys
import tempfile
import threading

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, '..'))
from site_builder.server import App, serve   # noqa: E402

REPO = os.path.abspath(os.path.join(HERE, '..', '..', '..'))
DEMO_MAP = os.path.join(REPO, 'maps', 'stores', 'demo_store', 'map.yaml')
DEMO_LABELS = json.load(open(os.path.join(HERE, '..', 'examples', 'labels_demo.json')))


class Srv:
    def __init__(self, tmp):
        self.out = os.path.join(tmp, 'generated')
        self.labels = os.path.join(tmp, 'work', 'labels.json')
        self.app = App(DEMO_MAP, self.labels, self.out,
                       os.path.join(HERE, '..', 'examples', 'labels_demo.json'))
        self.httpd = serve(self.app, 0)
        self.port = self.httpd.server_address[1]
        threading.Thread(target=self.httpd.serve_forever, daemon=True).start()

    def call(self, method, path, body=None, ctype='application/json', host=None):
        c = http.client.HTTPConnection('127.0.0.1', self.port, timeout=60)
        headers = {'Content-Type': ctype}
        if host:
            headers['Host'] = host
        data = body if isinstance(body, (bytes, type(None))) else json.dumps(body).encode()
        c.request(method, path, data, headers)
        r = c.getresponse()
        raw = r.read()
        c.close()
        return r.status, raw

    def json(self, *a, **k):
        st, raw = self.call(*a, **k)
        return st, json.loads(raw)

    def close(self):
        self.httpd.shutdown()
        self.httpd.server_close()


def with_server(fn):
    def run():
        with tempfile.TemporaryDirectory() as tmp:
            s = Srv(tmp)
            try:
                fn(s)
            finally:
                s.close()
    run.__name__ = fn.__name__
    return run


@with_server
def test_map_info_and_picture(s):
    st, m = s.json('GET', '/api/map')
    assert st == 200 and m['width_px'] == 180 and m['height_px'] == 140, m
    assert m['resolution_m'] == 0.05 and m['origin'] == {'x_m': -4.5, 'y_m': -3.5}
    st, png = s.call('GET', '/api/map.png')
    assert st == 200 and png[:8] == b'\x89PNG\r\n\x1a\n'
    st, page = s.call('GET', '/')
    assert st == 200 and b'<html' in page.lower() or b'<!doctype' in page.lower()


@with_server
def test_foreign_host_and_bad_requests_are_refused(s):
    assert s.call('GET', '/api/map', host='evil.example')[0] == 403
    assert s.call('GET', '/nothing')[0] == 404
    assert s.call('POST', '/api/generate', b'{}', ctype='text/plain')[0] == 400
    assert s.call('POST', '/api/generate', b'not json')[0] == 400
    assert s.call('POST', '/api/generate', {'labels': 5})[0] == 400
    big = b'{"labels": {"x": "' + b'a' * (1 << 21) + b'"}}'
    try:
        assert s.call('POST', '/api/generate', big)[0] == 400
    except (BrokenPipeError, ConnectionResetError):
        pass      # the server refused before the whole body was sent: fine
    assert s.call('GET', '/api/map')[0] == 200      # and it is still alive


@with_server
def test_labels_roundtrip_including_an_unfinished_set(s):
    st, d = s.json('GET', '/api/labels')
    assert st == 200 and d['labels'] == []
    half = {'schema_version': 1, 'labels': [
        {'id': 'table_01', 'kind': 'table', 'x_m': -1.6, 'y_m': 0.65}]}
    assert s.json('PUT', '/api/labels', half)[0] == 200          # no standby yet: still saved
    assert s.json('GET', '/api/labels')[1] == half
    bad = {'schema_version': 1, 'labels': [{'id': 'x', 'kind': 'chair', 'x_m': 0, 'y_m': 0}]}
    st, e = s.json('PUT', '/api/labels', bad)
    assert st == 400 and 'kind' in e['error']
    assert s.json('GET', '/api/labels')[1] == half               # the bad one did not overwrite


@with_server
def test_generate_does_not_write_and_export_does(s):
    st, ex = s.json('GET', '/api/example')
    assert st == 200 and ex == DEMO_LABELS
    st, pv = s.json('POST', '/api/generate', {'labels': DEMO_LABELS})
    assert st == 200 and pv['summary']['FAIL'] == 0 and len(pv['poses']) == 6, pv.get('error')
    assert not os.path.exists(s.out) and not os.path.exists(s.labels)
    st, pv = s.json('POST', '/api/export', {'labels': DEMO_LABELS})
    assert st == 200 and pv['exported_to'], pv
    for f in ('waypoints.yaml', 'entities.yaml', 'zones.yaml', 'preview.json'):
        assert os.path.isfile(os.path.join(s.out, f)), f
    assert os.path.isfile(s.labels)
    # a labels set with no standby gives a readable error, nothing new is written
    before = sorted(os.listdir(s.out))
    st, e = s.json('POST', '/api/export', {'labels': {'schema_version': 1, 'labels': []}})
    assert st == 400 and '대기(홈) 점' in e['error'] and sorted(os.listdir(s.out)) == before


def test_common_folder_is_protected():
    with tempfile.TemporaryDirectory() as tmp:
        try:
            App(DEMO_MAP, os.path.join(tmp, 'l.json'), os.path.join(tmp, 'config', 'common'))
        except ValueError as e:
            assert 'refusing' in str(e)
        else:
            raise AssertionError('export into config/common must be refused')


if __name__ == '__main__':
    n = 0
    for k, v in sorted(globals().items()):
        if k.startswith('test_') and callable(v):
            v()
            n += 1
            print('ok', k)
    print('all passed' if n else 'no tests')
