"""Saved Semantic Map of a store: a draft and the part of it the node generator accepted.

draft    what the person last saved from the map-setup screen
         data/semantics/<store>.json
applied  what tools/site_builder (automatic nodes and corridors) made from that draft
         data/generated/<store>/   status: provisional

Saving a draft or applying it never changes where the robots drive: they read config/common.
Putting an applied result there is a separate step with dated backups (promote.py).
"""
import hashlib
import json
import math
import os
import re
import shutil
import sys
import threading

from .util import now_iso

ENTITY_TYPES = ('table', 'counter', 'kitchen', 'robot_wait')
ZONE_TYPES = ('exclusive', 'no_go', 'slow')
ID_RE = re.compile(r'^[a-z][a-z0-9_]*$')
MAX_ITEMS = 1000
LABEL_KIND = {'table': 'table', 'kitchen': 'pickup', 'counter': 'pickup', 'robot_wait': 'standby'}


class SemanticError(Exception):
    def __init__(self, status, code, message, details=None):
        super().__init__(message)
        self.status, self.code, self.message, self.details = status, code, message, details


def _num(v):
    return isinstance(v, (int, float)) and not isinstance(v, bool) and math.isfinite(v)


def _pose_problem(p, where):
    if not isinstance(p, dict) or not _num(p.get('x')) or not _num(p.get('y')):
        return '%s: x, y 는 숫자여야 합니다.' % where
    if 'yaw' in p and not _num(p['yaw']):
        return '%s: yaw 는 숫자여야 합니다.' % where
    return None


def _area(poly):
    s = 0.0
    for (ax, ay), (bx, by) in zip(poly, poly[1:] + poly[:1]):
        s += ax * by - bx * ay
    return abs(s) / 2


def _check_items(doc, problems):
    ids = set()
    for i, e in enumerate(doc['entities']):
        where = 'entities[%d]' % i
        if not isinstance(e, dict):
            problems.append('%s: 객체여야 합니다.' % where)
            continue
        eid = e.get('id')
        if not isinstance(eid, str) or not ID_RE.match(eid):
            problems.append('%s: id 는 영소문자로 시작하고 영소문자·숫자·_ 만 쓸 수 있습니다.' % where)
            continue
        where = '%s(%s)' % (where, eid)
        if eid in ids:
            problems.append('%s: id 가 중복됩니다.' % where)
        ids.add(eid)
        if e.get('type') not in ENTITY_TYPES:
            problems.append('%s: type 은 %s 중 하나여야 합니다.' % (where, ', '.join(ENTITY_TYPES)))
        if 'label' in e and not isinstance(e['label'], str):
            problems.append('%s: label 은 문자열이어야 합니다.' % where)
        if e.get('pose') is not None:
            p = _pose_problem(e['pose'], where + '.pose')
            if p:
                problems.append(p)
        top = e.get('top_size_m')
        if top is not None and not (isinstance(top, dict) and _num(top.get('x'))
                                    and _num(top.get('y')) and 0 < top['x'] <= 10
                                    and 0 < top['y'] <= 10):
            problems.append('%s: top_size_m 은 {x, y} (0 초과 10 m 이하)여야 합니다.' % where)
        chairs = e.get('chairs')
        if chairs is not None and not (isinstance(chairs, list) and all(
                isinstance(c, dict) and _num(c.get('dx')) and _num(c.get('dy'))
                and _num(c.get('size_m')) and 0 < c['size_m'] <= 3 for c in chairs)):
            problems.append('%s: chairs 는 {dx, dy, size_m} 목록이어야 합니다.' % where)
    wids = set()
    for i, w in enumerate(doc['waypoints']):
        where = 'waypoints[%d]' % i
        if not isinstance(w, dict) or not isinstance(w.get('id'), str):
            problems.append('%s: id 가 필요합니다.' % where)
            continue
        where = '%s(%s)' % (where, w['id'])
        if w['id'] in wids:
            problems.append('%s: id 가 중복됩니다.' % where)
        wids.add(w['id'])
        if w.get('entity_id') not in ids:
            problems.append('%s: entity_id 가 가리키는 객체가 없습니다.' % where)
        p = _pose_problem(w.get('pose'), where + '.pose')
        if p:
            problems.append(p)
        if 'manual' in w and not isinstance(w['manual'], bool):
            problems.append('%s: manual 은 true/false 여야 합니다.' % where)
    zids = set()
    for i, z in enumerate(doc['zones']):
        where = 'zones[%d]' % i
        if not isinstance(z, dict) or not isinstance(z.get('id'), str):
            problems.append('%s: id 가 필요합니다.' % where)
            continue
        where = '%s(%s)' % (where, z['id'])
        if z['id'] in zids:
            problems.append('%s: id 가 중복됩니다.' % where)
        zids.add(z['id'])
        if z.get('type') not in ZONE_TYPES:
            problems.append('%s: type 은 %s 중 하나여야 합니다.' % (where, ', '.join(ZONE_TYPES)))
        poly = z.get('polygon')
        if not (isinstance(poly, list) and len(poly) >= 3 and all(
                isinstance(q, (list, tuple)) and len(q) == 2 and _num(q[0]) and _num(q[1])
                for q in poly)):
            problems.append('%s: polygon 은 꼭짓점 3개 이상의 [x, y] 목록이어야 합니다.' % where)
        elif _area([tuple(q) for q in poly]) < 1e-6:
            problems.append('%s: polygon 의 넓이가 0 입니다(꼭짓점이 한 직선 위에 있음).' % where)


def validate(doc, store_id, revision):
    """Raise SemanticError (400 shape, 409 old map revision) or return."""
    if not isinstance(doc, dict):
        raise SemanticError(400, 'INVALID_SEMANTICS', 'Semantic Map 은 JSON 객체여야 합니다.')
    problems = []
    if doc.get('store_id') != store_id:
        problems.append('store_id 는 %s 여야 합니다.' % store_id)
    rev = doc.get('map_revision')
    if isinstance(rev, bool) or not isinstance(rev, int):
        problems.append('map_revision 은 정수여야 합니다.')
    for key in ('entities', 'waypoints', 'zones'):
        if not isinstance(doc.get(key), list):
            problems.append('%s 는 목록이어야 합니다.' % key)
        elif len(doc[key]) > MAX_ITEMS:
            problems.append('%s 가 너무 많습니다(최대 %d).' % (key, MAX_ITEMS))
    if not problems and rev != revision:
        raise SemanticError(
            409, 'MAP_REVISION_MISMATCH',
            '지도 revision(%s)이 서버의 지도(%s)와 달라요. 현재 지도 기준으로 위치를 확인한 뒤 '
            '다시 저장해 주세요.' % (rev, revision))
    if not problems:
        _check_items(doc, problems)
    if problems:
        raise SemanticError(400, 'INVALID_SEMANTICS', 'Semantic Map 을 저장할 수 없습니다: '
                            + ' / '.join(problems[:5]) + (' …' if len(problems) > 5 else ''),
                            {'problems': problems})


def digest(doc):
    text = json.dumps(doc, sort_keys=True, separators=(',', ':'), ensure_ascii=False)
    return hashlib.sha256(text.encode('utf-8')).hexdigest()[:16]


def _main_kitchen_id(doc):
    """Id of the entity that becomes the pickup named 'kitchen': the one already called that,
    else the first kitchen (the robots and the backend look the pickup up by that name)."""
    kitchens = [e['id'] for e in doc['entities'] if e['type'] == 'kitchen']
    if 'kitchen' in kitchens:
        return 'kitchen'
    return kitchens[0] if kitchens else None


def _label_id(e, kitchen_id=None):
    """Id of the site_builder label made from entity e (robot_wait_NN becomes standby_NN and the
    kitchen becomes 'kitchen': the names the robot side expects)."""
    if e['type'] == 'robot_wait' and e['id'].startswith('robot_wait_'):
        return 'standby_' + e['id'][len('robot_wait_'):]
    if e['type'] == 'kitchen' and kitchen_id is not None and e['id'] == kitchen_id:
        return 'kitchen'
    return e['id']


def to_labels(doc):
    """Semantic Map -> site_builder labels. Returns (labels document, notes)."""
    wp = {w['entity_id']: w for w in doc['waypoints']}
    has_kitchen = any(e['type'] == 'kitchen' for e in doc['entities'])
    kitchen_id = _main_kitchen_id(doc)
    labels, notes, no_pose, no_top = [], [], [], []
    for e in doc['entities']:
        if e['type'] == 'counter' and has_kitchen:
            notes.append('counter(%s)는 kitchen 과 같은 곳으로 보고 건너뛰었어요.' % e['id'])
            continue
        pose = e.get('pose') or (wp.get(e['id']) or {}).get('pose')
        if not pose:
            no_pose.append(e['id'])
            continue
        kind = LABEL_KIND[e['type']]
        lid = _label_id(e, kitchen_id)
        if lid != e['id']:
            notes.append('%s 는 %s 로 이름을 바꿔 생성했어요.' % (e['id'], lid))
        lab = {'id': lid, 'kind': kind, 'x_m': float(pose['x']), 'y_m': float(pose['y'])}
        if kind == 'table':
            if e.get('top_size_m'):
                lab['top_size_m'] = {'x': float(e['top_size_m']['x']),
                                     'y': float(e['top_size_m']['y'])}
            else:
                no_top.append(e['id'])
            if e.get('chairs'):
                lab['chairs'] = [{'dx': float(c['dx']), 'dy': float(c['dy']),
                                  'size_m': float(c['size_m'])} for c in e['chairs']]
        w = wp.get(e['id'])
        if w and w.get('manual') is True and w.get('pose'):
            # placed by hand (dragged on the map): the generator must stand here, not choose
            lab['stop'] = {'x_m': float(w['pose']['x']), 'y_m': float(w['pose']['y']),
                           'yaw_deg': round(math.degrees(float(w['pose']['yaw'])), 3)}
        labels.append(lab)
    if not any(l['kind'] == 'pickup' for l in labels):
        notes.append('주방(kitchen)이 없어요: 로봇이 음료를 실을 곳이 정해지지 않았어요.')
    if no_top:
        raise SemanticError(422, 'MISSING_TOP_SIZE',
                            '테이블 상단 크기(가로·세로)가 없는 테이블이 있어요: %s. '
                            '지도 설정에서 테이블을 선택해 mm 단위로 입력해 주세요.' % ', '.join(no_top),
                            {'ids': no_top})
    if no_pose:
        raise SemanticError(422, 'MISSING_POSE', '위치(pose)가 없는 객체가 있어요: %s'
                            % ', '.join(no_pose), {'ids': no_pose})
    return {'schema_version': 1, 'labels': labels}, notes


def _ko_reason(reason):
    if reason.startswith('no corridor point faces it'):
        return ('복도에서 이 점을 정면으로 바라보며 설 만한 자리를 찾지 못했어요'
                '(앞 간격 0.20~0.90 m). 점을 복도 쪽으로 옮기거나 지워 주세요.')
    if 'fails the HardStop turn check' in reason:
        return '정지할 자리에 제자리 회전 여유가 부족해요. 주변 물건/벽과 거리를 확인해 주세요.'
    if reason.startswith('not connected'):
        return '첫 번째 대기 위치에서 갈 수 있는 복도와 이어지지 않아요.'
    if reason.startswith('pinned direction'):
        return '지정한 방향이 동·서·남·북 중 하나가 아니에요. 4방향 중에서 골라 주세요.'
    if reason.startswith('pinned stop: no corridor'):
        return '지정한 위치 가까이(1.5 m 안)에 통로가 없어요. 통로 쪽으로 옮겨 주세요.'
    if reason.startswith('pinned stop: nothing in front'):
        return '지정한 방향 앞에 테이블이나 벽이 없어요. 방향을 바꿔 주세요.'
    if reason.startswith('pinned stop: front gap'):
        m = re.search(r'front gap ([0-9.]+) m', reason)
        return ('지정한 위치는 앞 간격%s이 기준(0.20~0.90 m)을 벗어나요. 대상에 더 가깝거나 멀게 옮겨 주세요.'
                % (' %.2f m' % float(m.group(1)) if m else ''))
    if reason.startswith('pinned stop does not face'):
        return '지정한 위치와 방향이 대상을 바라보지 않아요. 방향을 바꾸거나 대상 쪽으로 옮겨 주세요.'
    return reason


def _approach_notes(approach):
    """One line per table: a branch was added (how long), or why the robot stays at the corridor."""
    out = []
    for tid, a in sorted(approach.items()):
        if a.get('branch_m'):
            out.append('%s: 테이블 쪽으로 접근 가지 %d cm 를 추가했어요(서빙 위치에서 테이블까지 약 %d cm).'
                       % (tid, round(a['branch_m'] * 100), round(a.get('gap_m', 0) * 100)))
        else:
            out.append('%s: 더 붙을 수 없어 접근 가지를 만들지 않았어요(손님이 손을 뻗어 받는 거리).' % tid)
    return out


def _atomic_json(path, data):
    os.makedirs(os.path.dirname(path), exist_ok=True)
    tmp = path + '.tmp'
    with open(tmp, 'w', encoding='utf-8') as f:
        json.dump(data, f, indent=1, ensure_ascii=False)
    os.replace(tmp, path)


def _read_json(path):
    try:
        with open(path, encoding='utf-8') as f:
            return json.load(f)
    except (OSError, ValueError):
        return None


class SemanticStore:
    def __init__(self, site, repo, data_dir, require_drive_test=False):
        self.site, self.repo, self.data_dir = site, repo, data_dir
        self.require_drive_test = require_drive_test
        self.tools_dir = os.path.join(repo, 'tools', 'site_builder')
        self.lock = threading.Lock()

    # ---- paths
    @property
    def _draft_path(self):
        return os.path.join(self.data_dir, 'semantics', '%s.json' % self.site.store_id)

    @property
    def _gen_dir(self):
        return os.path.join(self.data_dir, 'generated', self.site.store_id)

    # ---- draft
    def load_draft(self):
        saved = _read_json(self._draft_path)
        return saved.get('semantics') if isinstance(saved, dict) else None

    def save_draft(self, doc):
        validate(doc, self.site.store_id, self.site.revision)
        with self.lock:
            _atomic_json(self._draft_path, {'saved_at': now_iso(), 'sha': digest(doc),
                                            'semantics': doc})
        return doc

    # ---- status
    def status(self):
        saved = _read_json(self._draft_path)
        applied = _read_json(os.path.join(self._gen_dir, 'applied.json'))
        draft = ({'saved_at': saved.get('saved_at'), 'sha': saved.get('sha'),
                  'map_revision': (saved.get('semantics') or {}).get('map_revision')}
                 if isinstance(saved, dict) else None)
        return {'store_id': self.site.store_id, 'draft': draft, 'applied': applied,
                'in_sync': bool(draft and applied and draft['sha'] == applied.get('source_sha')),
                'require_drive_test': self.require_drive_test}

    # ---- apply: automatic nodes from the draft
    def _site_builder(self):
        if not os.path.isdir(os.path.join(self.tools_dir, 'site_builder')):
            raise SemanticError(503, 'SITE_BUILDER_MISSING',
                                '자동 노드 생성 도구(tools/site_builder)를 찾지 못했어요: %s'
                                % self.tools_dir)
        if self.tools_dir not in sys.path:
            sys.path.insert(0, self.tools_dir)
        from site_builder import labels as sb_labels
        from site_builder import params as sb_params
        from site_builder import pipeline as sb_pipeline
        return sb_labels, sb_params, sb_pipeline

    def apply(self, doc=None):
        """Generate nodes and corridors from the draft. Writes data/generated/<store>/ only
        when nothing FAILs and no point had to be left out; otherwise raises 422."""
        doc = doc if doc is not None else self.load_draft()
        if doc is None:
            raise SemanticError(409, 'NO_DRAFT', '저장된 초안이 없어요. 먼저 저장해 주세요.')
        if self.require_drive_test:
            raise SemanticError(409, 'DRIVE_TEST_REQUIRED',
                                '적용하려면 시험 주행 통과가 필요해요(아직 시험 도구가 연결되지 않았어요). '
                                '초안은 저장돼 있어요.')
        sb_labels, sb_params, sb_pipeline = self._site_builder()
        labels, notes = to_labels(doc)
        with self.lock:
            try:
                res = sb_pipeline.generate(self.site.map_yaml_path, labels,
                                           sb_params.load_params(self.repo))
            except sb_labels.LabelError as e:
                raise SemanticError(422, 'INVALID_LABELS', str(e))
            except sb_pipeline.PipelineError as e:
                raise SemanticError(422, 'GENERATION_FAILED', '자동 노드 생성에 실패했어요: %s' % e)
            pv = sb_pipeline.to_preview(res)
            self._reject_if_bad(res, pv)
            notes = notes + _approach_notes(pv.get('approach') or {})
            return self._write(res, pv, labels, notes, doc)

    def graph(self, doc=None):
        """The nodes, corridors and stop poses the generator makes from `doc` (an unsaved edit), else
        from the saved draft (from config/common when nothing is saved), WITHOUT writing anything.
        A waypoint with manual: true is a stop the person placed by hand; the generator must stand
        there (snapped to the nearest corridor point) and says why when it cannot.

        Unlike apply it also answers when apply would reject the draft: ok is False and the
        reasons are listed, so the person can see on the map where the problem is. Only when
        no graph can be built at all (bad labels, no corridor) is it an error (422)."""
        source = 'draft'
        if doc is None:
            doc = self.load_draft()
        if doc is None:
            doc, source = self.site.semantics, 'config'
        sb_labels, sb_params, sb_pipeline = self._site_builder()
        labels, notes = to_labels(doc)
        try:
            res = sb_pipeline.generate(self.site.map_yaml_path, labels,
                                       sb_params.load_params(self.repo))
        except sb_labels.LabelError as e:
            raise SemanticError(422, 'INVALID_LABELS', str(e))
        except sb_pipeline.PipelineError as e:
            raise SemanticError(422, 'GENERATION_FAILED', '자동 노드 생성에 실패했어요: %s' % e)
        pv = sb_pipeline.to_preview(res)
        fails, excluded, lines = self._collect_problems(res, pv)
        notes = notes + _approach_notes(pv.get('approach') or {})
        kitchen_id = _main_kitchen_id(doc)
        entity_of = {_label_id(e, kitchen_id): e['id'] for e in doc['entities']}
        applied = _read_json(os.path.join(self._gen_dir, 'applied.json'))
        sha = digest(doc)
        stops = []
        for lid, p in pv['poses'].items():
            stops.append({'id': lid, 'entity_id': entity_of.get(lid), 'node': p['node'],
                          'x': p['x'], 'y': p['y'], 'yaw_deg': p['yaw_deg'],
                          'yaw': round(math.radians(p['yaw_deg']), 4), 'kind': p['kind'],
                          'level': p['level'], 'hardstop_margin_m': p['hardstop_margin_m'],
                          'faces': p.get('faces'), 'pinned': bool(p.get('pinned'))})
        for x in excluded:
            x['entity_id'] = entity_of.get(x['id'])
        return {'store_id': self.site.store_id, 'source': source,
                'map_revision': doc['map_revision'], 'draft_sha': sha,
                'matches_applied': bool(applied and applied.get('source_sha') == sha),
                'ok': not fails and not excluded,
                'nodes': [{'id': k, **v} for k, v in pv['nodes'].items()],
                'edges': pv['edges'], 'stops': stops, 'excluded': excluded,
                'problems': lines, 'warnings': pv['warnings'], 'notes': notes,
                'summary': {k: pv['summary'].get(k, 0) for k in ('OK', 'WARN', 'FAIL')}}

    def _collect_problems(self, res, pv):
        pts = [(l['id'], l['x'], l['y']) for l in res.labels]
        at = {i: (x, y) for i, x, y in pts}
        pin_of = {l['id']: l['stop'] for l in res.labels if l.get('stop')}

        def near(a, b=None):
            """The three points closest to a node (a) or a corridor (segment a-b): where to look."""
            b = b or a
            def dist(q):
                (ax, ay), (bx, by) = a, b
                dx, dy = bx - ax, by - ay
                t = 0.0 if dx == dy == 0 else max(0.0, min(1.0, (
                    (q[1] - ax) * dx + (q[2] - ay) * dy) / (dx * dx + dy * dy)))
                return math.hypot(q[1] - (ax + t * dx), q[2] - (ay + t * dy))
            return ['%s(%.1f m)' % (q[0], dist(q)) for q in sorted(pts, key=dist)[:3]]

        fails = []
        for k, v in pv['nodes'].items():
            if v['level'] == 'FAIL':
                fails.append({'kind': 'node', 'id': k, 'x': v['x'], 'y': v['y'],
                              'margin_m': v['margin_m'], 'near': near((v['x'], v['y']))})
        for e in pv['edges']:
            if e['level'] == 'FAIL':
                a, b = pv['nodes'][e['from']], pv['nodes'][e['to']]
                fails.append({'kind': 'edge', 'id': e['id'], 'clearance_m': e['clearance_m'],
                              'near': near((a['x'], a['y']), (b['x'], b['y']))})
        for n, p in pv['poses'].items():
            if p['level'] == 'FAIL':
                fails.append({'kind': 'pose', 'id': n, 'near': [n]})
        excluded = [{'id': x['id'], 'reason': x['reason'], 'reason_ko': _ko_reason(x['reason']),
                     **({'x': at[x['id']][0], 'y': at[x['id']][1]} if x['id'] in at else {}),
                     **({'pin': {'x': pin_of[x['id']]['x'], 'y': pin_of[x['id']]['y'],
                                 'yaw_deg': pin_of[x['id']]['yaw_deg']}} if x['id'] in pin_of else {})}
                    for x in pv['excluded']]
        lines = ['· %s: %s' % (x['id'], x['reason_ko']) for x in excluded]
        for f in fails:
            what = {'node': '노드 %s(%.2f, %.2f)의 회전 여유가 부족해요' % (f['id'], f['x'], f['y'])
                    if f['kind'] == 'node' else '',
                    'edge': '통로 %s 가 로봇 폭에 비해 좁아요' % f['id'],
                    'pose': '%s 의 정지 위치가 안전 기준을 통과하지 못했어요' % f['id']}[f['kind']]
            lines.append('· %s (가까운 점: %s)' % (what, ', '.join(f['near'])))
        return fails, excluded, lines

    def _reject_if_bad(self, res, pv):
        fails, excluded, lines = self._collect_problems(res, pv)
        if not fails and not excluded:
            return
        raise SemanticError(
            422, 'GENERATION_REJECTED',
            '자동 노드 생성에서 문제가 있어 적용하지 않았어요(초안은 저장돼 있어요). '
            '아래 점을 지우거나 옮긴 뒤 다시 적용해 주세요(방금 추가하거나 옮긴 점부터 확인해 보세요).\n'
            + '\n'.join(lines),
            {'fails': fails, 'excluded': excluded, 'warnings': pv['warnings'],
             'summary': pv['summary']})

    def _write(self, res, pv, labels, notes, doc):
        sb_pipeline = sys.modules['site_builder.pipeline']
        tmp, old = self._gen_dir + '.tmp', self._gen_dir + '.old'
        os.makedirs(os.path.dirname(self._gen_dir), exist_ok=True)
        shutil.rmtree(tmp, ignore_errors=True)
        sb_pipeline.write_outputs(res, tmp)
        with open(os.path.join(tmp, 'labels.json'), 'w', encoding='utf-8') as f:
            json.dump(labels, f, indent=1, ensure_ascii=False)
        meta = {'store_id': self.site.store_id, 'applied_at': now_iso(),
                'map_revision': doc['map_revision'], 'source_sha': digest(doc),
                'summary': pv['summary'], 'warnings': pv['warnings'], 'notes': notes,
                'status': 'provisional', 'drive_test': {'status': 'not_run'},
                'output_dir': self._gen_dir}
        with open(os.path.join(tmp, 'applied.json'), 'w', encoding='utf-8') as f:
            json.dump(meta, f, indent=1, ensure_ascii=False)
        shutil.rmtree(old, ignore_errors=True)
        if os.path.isdir(self._gen_dir):
            os.replace(self._gen_dir, old)
        os.replace(tmp, self._gen_dir)
        shutil.rmtree(old, ignore_errors=True)
        return meta
