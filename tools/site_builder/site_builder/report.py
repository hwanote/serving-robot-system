"""test_report.json: written after every step (so a crash loses nothing), resumable."""
import json
import os
import time

from .verdict import ORDER, level


class ReportError(ValueError):
    pass


class Report:
    def __init__(self, path, tour):
        self.path = path
        self.tour = tour
        self.data = {'schema_version': 1, 'tour_hash': tour['hash'], 'start': tour['start'],
                     'config_dir': tour.get('test_config_dir'), 'created': time.strftime('%Y-%m-%d %H:%M:%S'),
                     'steps': {}}
        if os.path.isfile(path):
            with open(path, 'r', encoding='utf-8') as f:
                old = json.load(f)
            if old.get('tour_hash') != tour['hash']:
                raise ReportError('%s belongs to another tour (hash %s, this tour %s): '
                                  'use a new report file or the same tour.json'
                                  % (path, old.get('tour_hash'), tour['hash']))
            self.data = old

    def done(self, i):
        return str(i) in self.data['steps']

    def get(self, i):
        return self.data['steps'].get(str(i))

    def add(self, rec):
        self.data['steps'][str(rec['i'])] = rec
        self._write()

    def _write(self):
        d = os.path.dirname(os.path.abspath(self.path))
        os.makedirs(d, exist_ok=True)
        tmp = self.path + '.tmp'
        with open(tmp, 'w', encoding='utf-8') as f:
            json.dump(self.data, f, indent=1, ensure_ascii=False)
        os.replace(tmp, self.path)

    def last_ok_position(self):
        """Node the robot should be at if the newest recorded step passed and nothing followed."""
        idx = [int(k) for k in self.data['steps']]
        if not idx:
            return None
        rec = self.data['steps'][str(max(idx))]
        return rec['to'] if rec['verdict'] == 'PASS' and rec['i'] == max(idx) else None

    def edge_table(self):
        """{edge id: {'verdict': worst, 'level', 'dirs': {'a>b': rec}}} for the steps run so far."""
        out = {}
        for rec in sorted(self.data['steps'].values(), key=lambda r: r['i']):
            e = out.setdefault(rec['edge'], {'dirs': {}})
            e['dirs']['%s>%s' % (rec['from'], rec['to'])] = rec
        for e in out.values():
            vs = [r['verdict'] for r in e['dirs'].values()]
            bad = [v for v in ORDER if v in vs]
            e['verdict'] = bad[0] if bad else 'PASS'
            flags = sorted({f for r in e['dirs'].values() for f in r['flags']})
            e['flags'] = flags
            e['level'] = level(e['verdict'], flags)
        return out

    def summary(self):
        total = len(self.tour['steps'])
        rows = self.edge_table()
        lines = ['tour %s: %d of %d steps done' % (self.tour['hash'], len(self.data['steps']), total)]
        for eid in sorted(rows):
            e = rows[eid]
            parts = []
            for d, r in e['dirs'].items():
                parts.append('%s %s%s' % (d, r['verdict'], ('+' + '+'.join(r['flags'])) if r['flags'] else ''))
            lines.append('  %-5s %-4s  %s' % (eid, e['level'], ';  '.join(parts)))
        cnt = {'OK': 0, 'WARN': 0, 'FAIL': 0}
        for e in rows.values():
            cnt[e['level']] += 1
        lines.append('edges: OK %(OK)d / WARN %(WARN)d / FAIL %(FAIL)d' % cnt)
        return '\n'.join(lines)
