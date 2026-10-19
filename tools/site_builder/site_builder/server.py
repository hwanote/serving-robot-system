"""Small local server for the labelling screen (stdlib only, 127.0.0.1 only).

    python -m site_builder.server --map maps/stores/demo_store/map.yaml

GET  /                 the screen (web/index.html)
GET  /api/map          map size / resolution / origin / revision
GET  /api/map.png      the map as a picture (white free, black wall, grey unknown)
GET  /api/labels       the saved labels.json (or an empty document)
GET  /api/example      example labels for the demo map (--example)
PUT  /api/labels       save labels.json
POST /api/generate     {labels} -> preview.json content (nothing is written)
GET  /api/drive        drive-test state: corridors with verdicts, proposals, stale flags
POST /api/drive/propose  make proposals from test_report.json (status proposed)
PUT  /api/drive/decide   {decisions: {adj01: approved|rejected|proposed}}
POST /api/generate and /api/export also take "adjusted": true (approved proposals applied)
POST /api/export       {labels} -> writes labels.json + waypoints/entities/zones/preview.json
                       into --out-dir (default config/generated), never config/common
"""
import argparse
import json
import os
import sys
import threading
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import numpy as np

from .labels import LabelError, NoStandbyError, load_labels
from .mapio import MapData, MapError, png_bytes
from .params import REPO, load_params
from . import adjust as adj
from .adjust import AdjustError
from .pipeline import PipelineError, generate, to_preview, write_outputs

WEB = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'web')
MAX_BODY = 1 << 20
PROTECTED = ('common',)


class App:
    def __init__(self, map_yaml, labels_path, out_dir, example_path=None, drive_dir=None):
        self.map_yaml = os.path.abspath(map_yaml)
        self.labels_path = os.path.abspath(labels_path)
        self.out_dir = os.path.abspath(out_dir)
        self.example_path = example_path
        self.drive_dir = os.path.abspath(drive_dir) if drive_dir else None
        if os.path.basename(self.out_dir) in PROTECTED:
            raise ValueError('refusing to write generated files into %s' % self.out_dir)
        self.mp = MapData(self.map_yaml)
        self.lock = threading.Lock()
        img = np.full((self.mp.h, self.mp.w, 3), 128, np.uint8)
        img[self.mp.free] = 255
        img[self.mp.occupied] = 0
        self.png = png_bytes(img)

    def info(self):
        m = self.mp
        return {'name': os.path.basename(os.path.dirname(self.map_yaml)),
                'resolution_m': m.res, 'origin': {'x_m': m.ox, 'y_m': m.oy},
                'width_px': m.w, 'height_px': m.h, 'map_revision': m.revision,
                'out_dir': os.path.relpath(self.out_dir, REPO)
                if self.out_dir.startswith(REPO) else self.out_dir}

    def labels(self):
        if os.path.isfile(self.labels_path):
            with open(self.labels_path, 'r', encoding='utf-8') as f:
                return json.load(f)
        return {'schema_version': 1, 'map_revision': self.mp.revision, 'labels': []}

    def example(self):
        if not self.example_path or not os.path.isfile(self.example_path):
            raise ValueError('no example labels for this map')
        with open(self.example_path, 'r', encoding='utf-8') as f:
            return json.load(f)

    def save_labels(self, doc):
        os.makedirs(os.path.dirname(self.labels_path), exist_ok=True)
        tmp = self.labels_path + '.tmp'
        with open(tmp, 'w', encoding='utf-8') as f:
            json.dump(doc, f, indent=1, ensure_ascii=False)
        os.replace(tmp, self.labels_path)

    def run(self, doc, adjusted=False):
        doc = dict(doc)
        doc.setdefault('schema_version', 1)
        doc.setdefault('map_revision', self.mp.revision)
        path = None
        if adjusted:
            path = self._p('adjustments.json')
            if not path or not os.path.isfile(path):
                raise ValueError('there is no adjustments.json yet: make proposals first')
        with self.lock:
            return generate(self.map_yaml, doc, load_params(), adjustments=path)

    # ---- drive test
    def _p(self, name):
        return os.path.join(self.drive_dir, name) if self.drive_dir else None

    def _read(self, name):
        p = self._p(name)
        if p and os.path.isfile(p):
            with open(p, 'r', encoding='utf-8') as f:
                return json.load(f)
        return None

    def drive_state(self):
        tour, rep = self._read('tour.json'), self._read('test_report.json')
        if not tour:
            return {'available': False, 'hint': 'no drive test yet: python -m site_builder.drive plan ...'}
        import yaml
        with open(os.path.join(tour['candidate_dir'], 'waypoints.yaml'), 'r', encoding='utf-8') as f:
            wp = yaml.safe_load(f)
        try:
            with open(self.labels_path, 'r', encoding='utf-8') as f:
                doc = json.load(f)
            with self.lock:
                now = adj.hash_of_graph(generate(self.map_yaml, doc, load_params()).graph)
        except Exception:      # noqa: BLE001 - the state is still useful without the comparison
            now = None
        tested = adj.hash_waypoints(wp)
        steps = (rep or {}).get('steps', {})
        edges = []
        for e in wp['edges']:
            a, b = wp['nodes'][e['from']], wp['nodes'][e['to']]
            dirs = []
            for st in tour['steps']:
                if st['edge'] != e['id']:
                    continue
                r = steps.get(str(st['i']))
                dirs.append({'i': st['i'], 'from': st['from'], 'to': st['to'],
                             'verdict': r['verdict'] if r else None,
                             'level': adj.level(r['verdict'], r['flags']) if r else None,
                             'flags': r['flags'] if r else [], 'reasons': r['reasons'] if r else []})
            edges.append({'id': e['id'], 'from': e['from'], 'to': e['to'],
                          'a': {'x': a['x'], 'y': a['y']}, 'b': {'x': b['x'], 'y': b['y']}, 'dirs': dirs})
        done = [d for e in edges for d in e['dirs'] if d['verdict']]
        counts = {'OK': 0, 'WARN': 0, 'FAIL': 0}
        for d in done:
            counts[d['level']] += 1
        ad = self._read('adjustments.json')
        return {'available': True, 'tour_hash': tour['hash'], 'steps_total': len(tour['steps']),
                'steps_done': len(done), 'counts': counts, 'edges': edges,
                'tested_hash': tested, 'current_hash': now,
                'labels_changed': now is not None and now != tested,
                'adjustments': ad,
                'adjustments_stale': bool(ad) and now is not None and ad.get('base_hash') != now}

    def propose(self):
        from .drive import make_proposals
        if not self._read('test_report.json'):
            raise ValueError('there is no test_report.json yet: run the drive test first')
        with self.lock:
            return make_proposals(self._p('tour.json'), self._p('test_report.json'), self._p('adjustments.json'))

    def decide(self, decisions):
        ad = self._read('adjustments.json')
        if not ad:
            raise ValueError('there is no adjustments.json yet: make proposals first')
        if not isinstance(decisions, dict):
            raise ValueError('decisions must be {id: status}')
        by_id = {p['id']: p for p in ad['proposals']}
        for pid, st in decisions.items():
            if pid not in by_id:
                raise ValueError('unknown proposal %s' % pid)
            if st not in ('proposed', 'approved', 'rejected'):
                raise ValueError('status must be proposed, approved or rejected')
            if st == 'approved' and by_id[pid]['kind'] == 'review_edge':
                raise ValueError('%s is only a note for a person: it cannot be approved' % pid)
        for pid, st in decisions.items():
            by_id[pid]['status'] = st
        tmp = self._p('adjustments.json') + '.tmp'
        with open(tmp, 'w', encoding='utf-8') as f:
            json.dump(ad, f, indent=1, ensure_ascii=False)
        os.replace(tmp, self._p('adjustments.json'))
        return ad


def make_handler(app):
    class Handler(BaseHTTPRequestHandler):
        server_version = 'site_builder'

        def log_message(self, fmt, *args):
            sys.stderr.write('%s\n' % (fmt % args))

        # ---- helpers
        def _host_ok(self):
            host = (self.headers.get('Host') or '').split(':')[0]
            return host in ('127.0.0.1', 'localhost')

        def _send(self, code, body, ctype):
            self.send_response(code)
            self.send_header('Content-Type', ctype)
            self.send_header('Content-Length', str(len(body)))
            self.send_header('Cache-Control', 'no-store')
            self.end_headers()
            self.wfile.write(body)

        def _json(self, obj, code=200):
            self._send(code, json.dumps(obj, ensure_ascii=False).encode('utf-8'),
                       'application/json; charset=utf-8')

        def _err(self, code, msg):
            self.close_connection = True
            self._json({'error': msg}, code)

        def _body(self):
            if 'application/json' not in (self.headers.get('Content-Type') or ''):
                raise ValueError('Content-Type must be application/json')
            n = int(self.headers.get('Content-Length') or 0)
            if n > MAX_BODY:
                raise ValueError('body too large')
            return json.loads(self.rfile.read(n).decode('utf-8') or '{}')

        def _guard(self):
            if not self._host_ok():
                self._err(403, 'bad host')
                return False
            return True

        # ---- routes
        def do_GET(self):
            if not self._guard():
                return
            path = self.path.split('?')[0]
            if path in ('/', '/index.html'):
                with open(os.path.join(WEB, 'index.html'), 'rb') as f:
                    self._send(200, f.read(), 'text/html; charset=utf-8')
            elif path == '/api/map':
                self._json(app.info())
            elif path == '/api/map.png':
                self._send(200, app.png, 'image/png')
            elif path == '/api/labels':
                self._json(app.labels())
            elif path == '/api/drive':
                try:
                    self._json(app.drive_state() if app.drive_dir else {'available': False})
                except (OSError, ValueError, KeyError) as e:
                    self._err(500, 'cannot read the drive test: %s' % e)
            elif path == '/api/example':
                try:
                    self._json(app.example())
                except ValueError as e:
                    self._err(404, str(e))
            else:
                self._err(404, 'not found')

        def do_PUT(self):
            if not self._guard():
                return
            if self.path == '/api/drive/decide':
                try:
                    self._json(app.decide(self._body().get('decisions')))
                except (ValueError, KeyError, TypeError) as e:
                    self._err(400, str(e))
                return
            if self.path != '/api/labels':
                return self._err(404, 'not found')
            try:
                doc = self._body()
                try:
                    load_labels(doc, app.mp.revision)
                except NoStandbyError:
                    pass      # a half-finished set is still saved
                app.save_labels(doc)
                self._json({'saved': True})
            except (ValueError, LabelError, KeyError, TypeError) as e:
                self._err(400, str(e))

        def do_POST(self):
            if not self._guard():
                return
            if self.path == '/api/drive/propose':
                try:
                    self._body()
                    self._json(app.propose())
                except (ValueError, KeyError, OSError) as e:
                    self._err(400, str(e))
                return
            if self.path not in ('/api/generate', '/api/export'):
                return self._err(404, 'not found')
            try:
                body = self._body()
                doc = body.get('labels')
                if not isinstance(doc, dict):
                    raise ValueError("body must be {\"labels\": <labels.json content>}")
                res = app.run(doc, adjusted=bool(body.get('adjusted')))
                pv = to_preview(res)
                if self.path == '/api/export':
                    if pv['summary']['FAIL']:
                        raise ValueError('result has FAIL items: fix them before exporting')
                    app.save_labels(doc)
                    with app.lock:
                        write_outputs(res, app.out_dir)
                    pv['exported_to'] = app.info()['out_dir']
                self._json(pv)
            except (ValueError, LabelError, KeyError, TypeError, PipelineError, MapError, AdjustError) as e:
                self._err(400, str(e))
            except Exception as e:      # noqa: BLE001 - the screen shows the text
                self._err(500, '%s: %s' % (type(e).__name__, e))

    return Handler


def serve(app, port=8765):
    return ThreadingHTTPServer(('127.0.0.1', port), make_handler(app))


def main(argv=None):
    ap = argparse.ArgumentParser(prog='site_builder.server')
    ap.add_argument('--map', default=os.path.join(REPO, 'maps', 'stores', 'demo_store', 'map.yaml'))
    ap.add_argument('--labels', default=os.path.join(REPO, 'tools', 'site_builder', 'out', 'labels.json'))
    ap.add_argument('--out-dir', default=os.path.join(REPO, 'config', 'generated'))
    ap.add_argument('--example', default=os.path.join(REPO, 'tools', 'site_builder', 'examples', 'labels_demo.json'))
    ap.add_argument('--drive-dir', default=os.path.join(REPO, 'tools', 'site_builder', 'out', 'drive_demo'),
                    help='folder with tour.json / test_report.json / adjustments.json')
    ap.add_argument('--port', type=int, default=8765)
    a = ap.parse_args(argv)
    app = App(a.map, a.labels, a.out_dir, a.example, a.drive_dir)
    srv = serve(app, a.port)
    print('site_builder: http://127.0.0.1:%d/   (map %s, Ctrl+C to stop)' % (a.port, a.map))
    try:
        srv.serve_forever()
    except KeyboardInterrupt:
        pass
    return 0


if __name__ == '__main__':
    sys.exit(main())
