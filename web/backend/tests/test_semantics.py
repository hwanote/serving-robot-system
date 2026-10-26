"""Saved Semantic Map: draft save, apply (automatic nodes), rejection and retry."""
import copy
import hashlib
import math
import os

import pytest
from fastapi.testclient import TestClient

from .test_api import make

URL = '/api/stores/demo_store/semantics'
COMMON = os.path.join(os.path.dirname(__file__), '..', '..', '..', 'config', 'common')


@pytest.fixture()
def client(tmp_path):
    with TestClient(make(data_dir=str(tmp_path))) as c:
        c.data_dir = tmp_path
        yield c


@pytest.fixture()
def base(client):
    return client.get(URL).json()


def common_hash():
    h = hashlib.sha256()
    for name in sorted(os.listdir(COMMON)):
        with open(os.path.join(COMMON, name), 'rb') as f:
            h.update(name.encode() + f.read())
    return h.hexdigest()


def table(tid, x, y, **extra):
    return {'id': tid, 'type': 'table', 'label': tid, 'pose': {'x': x, 'y': y, 'yaw': 0.0},
            'top_size_m': {'x': 0.8, 'y': 0.8}, **extra}


def wp(entity_id, x, y):
    return {'id': 'wp_' + entity_id, 'entity_id': entity_id, 'pose': {'x': x, 'y': y, 'yaw': 1.57}}


def with_entity(doc, ent, waypoint=None):
    d = copy.deepcopy(doc)
    d['entities'].append(ent)
    if waypoint:
        d['waypoints'].append(waypoint)
    return d


def without_entity(doc, eid):
    d = copy.deepcopy(doc)
    d['entities'] = [e for e in d['entities'] if e['id'] != eid]
    d['waypoints'] = [w for w in d['waypoints'] if w['entity_id'] != eid]
    return d


def blocking_table(base):
    """A wide table put right against the corridors: the generator cannot give the robots a
    safe way through, so it leaves points out and apply is rejected."""
    return with_entity(base, table('table_05', -0.8, 0.3, top_size_m={'x': 1.4, 'y': 1.0}),
                       wp('table_05', -0.8, 0.9))


# ---------------------------------------------------------------- draft
def test_get_falls_back_to_config_common(client, base):
    assert base['store_id'] == 'demo_store' and base['map_revision'] == 1
    assert {e['id'] for e in base['entities']} >= {'table_01', 'kitchen', 'standby_01'}
    assert client.get(URL + '/status').json()['draft'] is None
    assert client.get('/api/stores/other/semantics').status_code == 404


def test_draft_roundtrip_keeps_extra_fields_and_survives_restart(client, base, tmp_path):
    d = with_entity(base, table('table_05', 3.0, 0.0, top_size_m={'x': 1.0, 'y': 0.6}),
                    wp('table_05', 3.0, -0.4))
    d['zones'] = [{'id': 'zone_exclusive_01', 'type': 'exclusive',
                   'polygon': [[-3.5, -1.0], [-2.9, -1.0], [-3.0, 0.5], [-3.6, 0.5]]}]
    r = client.put(URL, json=d)
    assert r.status_code == 200 and r.json() == d
    st = client.get(URL + '/status').json()
    assert st['draft']['map_revision'] == 1 and st['applied'] is None and st['in_sync'] is False
    assert (tmp_path / 'semantics' / 'demo_store.json').is_file()
    # a new backend process reads the same draft back
    with TestClient(make(data_dir=str(tmp_path))) as c2:
        got = c2.get(URL).json()
    assert got == d and got['entities'][-1]['top_size_m'] == {'x': 1.0, 'y': 0.6}


def test_saving_a_draft_does_not_touch_config_common(client, base):
    before = common_hash()
    assert client.put(URL, json=with_entity(base, table('table_05', 3.0, 0.0))).status_code == 200
    assert client.put(URL + '?apply=true', json=base).status_code == 200
    assert common_hash() == before


def test_validation_errors(client, base):
    def put(doc):
        return client.put(URL, json=doc)

    r = client.put(URL, content='not json')
    assert r.status_code == 400 and r.json()['code'] == 'INVALID_JSON'
    assert put([]).json()['code'] == 'INVALID_SEMANTICS'
    assert put({'store_id': 'demo_store'}).status_code == 400
    bad = copy.deepcopy(base)
    bad['entities'][0]['type'] = 'sofa'
    assert put(bad).status_code == 400
    dup = with_entity(base, table('table_01', 0, 0))
    assert put(dup).status_code == 400
    bad_id = with_entity(base, table('Table 9', 0, 0))
    assert put(bad_id).status_code == 400
    no_xy = with_entity(base, {'id': 'table_09', 'type': 'table', 'pose': {'x': 'a', 'y': 1}})
    assert put(no_xy).status_code == 400
    dangling = copy.deepcopy(base)
    dangling['waypoints'].append(wp('nobody', 0, 0))
    assert put(dangling).status_code == 400
    two_points = copy.deepcopy(base)
    two_points['zones'] = [{'id': 'z1', 'type': 'slow', 'polygon': [[0, 0], [1, 1]]}]
    r = put(two_points)
    assert r.status_code == 400 and '꼭짓점 3개' in r.json()['message']
    line = copy.deepcopy(base)
    line['zones'] = [{'id': 'z1', 'type': 'no_go', 'polygon': [[0, 0], [1, 1], [2, 2]]}]
    assert put(line).status_code == 400
    bad_zone_type = copy.deepcopy(base)
    bad_zone_type['zones'] = [{'id': 'z1', 'type': 'party', 'polygon': [[0, 0], [1, 0], [0, 1]]}]
    assert put(bad_zone_type).status_code == 400
    bad_top = with_entity(base, table('table_05', 3, 0, top_size_m={'x': -1, 'y': 1}))
    assert put(bad_top).status_code == 400
    assert client.get(URL + '/status').json()['draft'] is None      # nothing was saved


def test_old_map_revision_is_409(client, base):
    old = dict(base, map_revision=7)
    r = client.put(URL, json=old)
    assert r.status_code == 409 and r.json()['code'] == 'MAP_REVISION_MISMATCH'
    assert client.get(URL + '/status').json()['draft'] is None


def test_other_store_is_404(client, base):
    assert client.put('/api/stores/other/semantics', json=base).status_code == 404
    assert client.post('/api/stores/other/semantics/apply').status_code == 404


# ---------------------------------------------------------------- apply
def test_apply_generates_nodes_and_records_it(client, base, tmp_path):
    before = common_hash()
    r = client.put(URL + '?apply=true', json=base)
    assert r.status_code == 200, r.text
    res = r.json()['apply_result']
    assert res['summary']['FAIL'] == 0 and res['summary']['OK'] > 0
    assert res['status'] == 'provisional' and res['drive_test'] == {'status': 'not_run'}
    out = tmp_path / 'generated' / 'demo_store'
    for name in ('waypoints.yaml', 'entities.yaml', 'zones.yaml', 'preview.json', 'labels.json',
                 'applied.json'):
        assert (out / name).is_file(), name
    text = (out / 'waypoints.yaml').read_text(encoding='utf-8')
    assert 'status: provisional' in text and 'standby_01' in text
    st = client.get(URL + '/status').json()
    assert st['in_sync'] is True and st['applied']['source_sha'] == st['draft']['sha']
    assert common_hash() == before                       # never written to config/common
    # a later edit makes the applied result out of date until it is applied again
    client.put(URL, json=with_entity(base, table('table_05', 3.0, 0.0)))
    assert client.get(URL + '/status').json()['in_sync'] is False


def test_apply_the_saved_draft_without_sending_it_again(client, base):
    assert client.post(URL + '/apply').status_code == 409          # nothing saved yet
    assert client.post(URL + '/apply').json()['code'] == 'NO_DRAFT'
    client.put(URL, json=base)
    r = client.post(URL + '/apply')
    assert r.status_code == 200 and r.json()['summary']['FAIL'] == 0


def test_rejected_when_a_table_blocks_the_way_then_delete_the_point_and_apply(client, base, tmp_path):
    bad = blocking_table(base)
    r = client.put(URL + '?apply=true', json=bad)
    assert r.status_code == 422 and r.json()['code'] == 'GENERATION_REJECTED'
    body = r.json()
    assert body['details']['excluded'] and all(x['reason_ko'] for x in body['details']['excluded'])
    assert '다시 적용' in body['message']
    # the draft is kept (nothing is lost), but nothing was applied
    assert any(e['id'] == 'table_05' for e in client.get(URL).json()['entities'])
    st = client.get(URL + '/status').json()
    assert st['draft'] and st['applied'] is None
    assert not (tmp_path / 'generated' / 'demo_store').exists()
    # delete the offending point and apply again
    fixed = without_entity(bad, 'table_05')
    r = client.put(URL + '?apply=true', json=fixed)
    assert r.status_code == 200 and r.json()['apply_result']['summary']['FAIL'] == 0
    st = client.get(URL + '/status').json()
    assert st['applied'] is not None and st['in_sync'] is True


def test_rejected_when_a_point_cannot_be_placed_on_a_corridor(client, base):
    bad = copy.deepcopy(base)
    for e in bad['entities']:
        if e['id'] == 'kitchen':
            e['pose'] = {'x': 3.9, 'y': 3.0, 'yaw': 0.0}      # far from every corridor
    r = client.put(URL + '?apply=true', json=bad)
    assert r.status_code == 422 and r.json()['code'] == 'GENERATION_REJECTED'
    ex = r.json()['details']['excluded']
    assert [x['id'] for x in ex] == ['kitchen'] and '복도' in ex[0]['reason_ko']
    assert 'kitchen' in r.json()['message']
    # moving it back onto the old place works
    assert client.put(URL + '?apply=true', json=base).status_code == 200


def test_robot_wait_ids_are_renamed_for_the_robot_side(client, base):
    d = copy.deepcopy(base)
    for e in d['entities']:
        if e['id'] in ('standby_01', 'standby_02'):
            e['id'] = e['id'].replace('standby_', 'robot_wait_')
    for w in d['waypoints']:
        w['entity_id'] = w['entity_id'].replace('standby_', 'robot_wait_')
        w['id'] = 'wp_' + w['entity_id']
    res = client.put(URL + '?apply=true', json=d).json()['apply_result']
    assert any('standby_01' in n for n in res['notes'])
    pre = client.app.state.semantics._gen_dir
    assert 'standby_01' in open(os.path.join(pre, 'waypoints.yaml'), encoding='utf-8').read()


def test_counter_next_to_a_kitchen_is_skipped_and_kitchen_is_required_note(client, base):
    both = with_entity(base, {'id': 'counter_01', 'type': 'counter', 'label': 'C',
                              'pose': {'x': -2.3, 'y': 2.4, 'yaw': 0.0}})
    res = client.put(URL + '?apply=true', json=both).json()['apply_result']
    assert any('counter_01' in n for n in res['notes'])
    nokitchen = without_entity(base, 'kitchen')
    res = client.put(URL + '?apply=true', json=nokitchen).json()['apply_result']
    assert any('주방' in n for n in res['notes'])


def test_missing_pose_and_missing_standby(client, base):
    nopose = with_entity(base, {'id': 'table_09', 'type': 'table', 'label': 'x'})
    r = client.put(URL + '?apply=true', json=nopose)
    assert r.status_code == 422 and r.json()['code'] == 'MISSING_POSE'
    nostandby = copy.deepcopy(base)
    nostandby['entities'] = [e for e in nostandby['entities'] if e['type'] != 'robot_wait']
    nostandby['waypoints'] = [w for w in nostandby['waypoints']
                              if w['entity_id'] in {e['id'] for e in nostandby['entities']}]
    r = client.put(URL + '?apply=true', json=nostandby)
    assert r.status_code == 422 and r.json()['code'] == 'INVALID_LABELS'


def test_drive_test_requirement_blocks_apply_but_keeps_the_draft(tmp_path):
    with TestClient(make(data_dir=str(tmp_path), require_drive_test=True)) as c:
        base = c.get(URL).json()
        r = c.put(URL + '?apply=true', json=base)
        assert r.status_code == 409 and r.json()['code'] == 'DRIVE_TEST_REQUIRED'
        st = c.get(URL + '/status').json()
        assert st['draft'] and st['applied'] is None and st['require_drive_test'] is True


def test_site_builder_missing_is_503(client, base, tmp_path):
    client.app.state.semantics.tools_dir = str(tmp_path / 'nowhere')
    r = client.put(URL + '?apply=true', json=base)
    assert r.status_code == 503 and r.json()['code'] == 'SITE_BUILDER_MISSING'
    assert client.get(URL + '/status').json()['draft']          # still saved


def test_mapping_start_is_off_unless_enabled(client):
    r = client.post('/api/mapping/start')
    assert r.status_code == 503 and r.json()['code'] == 'MAPPING_DISABLED'


# ---- table top size (top_size_m) ------------------------------------------------------

def test_demo_semantics_carry_the_real_top_size_and_chairs(base):
    tables = [e for e in base['entities'] if e['type'] == 'table']
    assert tables, 'demo has tables'
    for t in tables:                      # real values from entities.yaml, not a default
        assert t['top_size_m'] == {'x': 0.8, 'y': 0.8}
        assert t['chairs'] and set(t['chairs'][0]) == {'dx', 'dy', 'size_m'}
    assert all('top_size_m' not in e for e in base['entities'] if e['type'] != 'table')


def test_apply_rejects_a_table_without_top_size_but_the_draft_is_kept(client, base):
    ent = table('table_05', -0.8, -0.2)
    del ent['top_size_m']
    doc = with_entity(base, ent, wp('table_05', -0.8, -0.9))
    r = client.put(URL + '?apply=true', json=doc)
    assert r.status_code == 422 and r.json()['code'] == 'MISSING_TOP_SIZE'
    assert r.json()['details']['ids'] == ['table_05']
    assert 'table_05' in r.json()['message']
    assert any(e['id'] == 'table_05' for e in client.get(URL).json()['entities'])   # draft kept
    assert client.get(URL + '/status').json()['applied'] is None                   # nothing applied


def test_draft_save_does_not_require_top_size(client, base):
    ent = table('table_05', -0.8, -0.2)
    del ent['top_size_m']
    assert client.put(URL, json=with_entity(base, ent, wp('table_05', -0.8, -0.9))).status_code == 200


def test_adding_the_top_size_lets_the_apply_through(client, base):
    ent = table('table_05', -0.8, -0.2)
    del ent['top_size_m']
    doc = with_entity(base, ent, wp('table_05', -0.8, -0.9))
    assert client.put(URL + '?apply=true', json=doc).status_code == 422
    doc['entities'][-1]['top_size_m'] = {'x': 0.6, 'y': 0.9}
    r = client.put(URL + '?apply=true', json=doc)
    assert r.status_code in (200, 422) and r.json().get('code') != 'MISSING_TOP_SIZE'


def test_chairs_and_top_size_survive_a_frontend_style_roundtrip(client, base):
    got = client.get(URL).json()
    assert client.put(URL, json=got).status_code == 200
    again = client.get(URL).json()
    assert [e.get('top_size_m') for e in again['entities']] == [e.get('top_size_m') for e in got['entities']]
    assert [e.get('chairs') for e in again['entities']] == [e.get('chairs') for e in got['entities']]


def test_apply_has_no_approach_notes_while_the_branch_step_is_off(client, base):
    # the approach branch step is off by default (not yet run on a robot)
    res = client.put(URL + '?apply=true', json=base).json()['apply_result']
    assert not [n for n in res['notes'] if '접근 가지' in n], res['notes']


def test_approach_note_for_a_built_branch():
    from app.semantics import _approach_notes
    out = _approach_notes({'table_01': {'branch_m': 0.411, 'gap_m': 0.309}})
    assert out == ['table_01: 테이블 쪽으로 접근 가지 41 cm 를 추가했어요(서빙 위치에서 테이블까지 약 31 cm).']


# ---------------------------------------------------------------- graph preview (writes nothing)
def test_graph_shows_corridors_and_stops_and_writes_nothing(client, base, tmp_path):
    before = common_hash()
    r = client.get(URL + '/graph')
    assert r.status_code == 200, r.text
    g = r.json()
    assert g['ok'] is True and g['source'] == 'config' and g['matches_applied'] is False
    assert g['nodes'] and g['edges'] and g['summary']['FAIL'] == 0
    ids = {n['id'] for n in g['nodes']}
    for e in g['edges']:
        assert e['from'] in ids and e['to'] in ids and e['length_m'] > 0
    by_entity = {s['entity_id']: s for s in g['stops']}
    assert {'table_01', 'kitchen', 'standby_01'} <= set(by_entity)
    for s in g['stops']:
        assert s['node'] in ids and s['level'] in ('OK', 'WARN', 'FAIL') and -3.2 <= s['yaw'] <= 3.2
    assert not (tmp_path / 'generated').exists() and not (tmp_path / 'semantics').exists()
    assert common_hash() == before


def test_graph_follows_the_saved_draft_and_knows_when_it_was_applied(client, base):
    client.put(URL, json=base)
    g = client.get(URL + '/graph').json()
    assert g['source'] == 'draft' and g['matches_applied'] is False
    assert g['draft_sha'] == client.get(URL + '/status').json()['draft']['sha']
    assert client.post(URL + '/apply').status_code == 200
    assert client.get(URL + '/graph').json()['matches_applied'] is True
    client.put(URL, json=with_entity(base, table('table_05', 3.0, 0.0)))
    assert client.get(URL + '/graph').json()['matches_applied'] is False


def test_graph_still_answers_when_apply_would_reject(client, base):
    client.put(URL, json=blocking_table(base))
    assert client.post(URL + '/apply').status_code == 422
    g = client.get(URL + '/graph').json()
    assert g['ok'] is False and g['problems'] and g['excluded']
    assert g['nodes'] and g['edges'] and g['stops']        # the rest of the map is still drawn


def test_graph_gives_the_position_of_a_point_that_cannot_be_placed(client, base):
    bad = copy.deepcopy(base)
    for e in bad['entities']:
        if e['id'] == 'kitchen':
            e['pose'] = {'x': 3.9, 'y': 3.0, 'yaw': 0.0}
    client.put(URL, json=bad)
    g = client.get(URL + '/graph').json()
    assert g['ok'] is False
    ex = g['excluded'][0]
    assert ex['id'] == 'kitchen' and ex['entity_id'] == 'kitchen'
    assert (ex['x'], ex['y']) == (3.9, 3.0) and '복도' in ex['reason_ko']
    assert all(s['entity_id'] != 'kitchen' for s in g['stops'])
    # the 422 of apply keeps working the same way and now also carries the position
    r = client.post(URL + '/apply')
    assert r.status_code == 422 and r.json()['details']['excluded'][0]['x'] == 3.9


def test_graph_names_the_editor_object_of_a_renamed_standby_point(client, base):
    d = copy.deepcopy(base)
    for e in d['entities']:
        if e['id'] in ('standby_01', 'standby_02'):
            e['id'] = e['id'].replace('standby_', 'robot_wait_')
    for w in d['waypoints']:
        w['entity_id'] = w['entity_id'].replace('standby_', 'robot_wait_')
        w['id'] = 'wp_' + w['entity_id']
    client.put(URL, json=d)
    stops = [s for s in client.get(URL + '/graph').json()['stops'] if s['kind'] == 'standby']
    assert {s['id'] for s in stops} == {'standby_01', 'standby_02'}
    assert {s['entity_id'] for s in stops} == {'robot_wait_01', 'robot_wait_02'}


def test_graph_errors(client, base):
    assert client.get('/api/stores/other_store/semantics/graph').status_code == 404
    t = table('table_05', 3.0, 0.0)
    del t['top_size_m']
    client.put(URL, json=with_entity(base, t))
    r = client.get(URL + '/graph')
    assert r.status_code == 422 and r.json()['code'] == 'MISSING_TOP_SIZE'


# ---------------------------------------------------------------- stops placed by hand (manual waypoints)
def manual_at(doc, eid, x, y, yaw_deg):
    """doc with the waypoint of eid replaced by one the person placed (dragged) by hand."""
    d = copy.deepcopy(doc)
    for w in d['waypoints']:
        if w['entity_id'] == eid:
            w['pose'] = {'x': x, 'y': y, 'yaw': math.radians(yaw_deg)}
            w['manual'] = True
            return d
    raise AssertionError(eid)


def stop_of(g, eid):
    return next(s for s in g['stops'] if s['entity_id'] == eid)


def test_a_waypoint_not_marked_manual_is_ignored(client, base):
    d = copy.deepcopy(base)
    for w in d['waypoints']:
        if w['entity_id'] == 'table_01':
            w['pose'] = {'x': w['pose']['x'] - 0.3, 'y': w['pose']['y'] + 0.2, 'yaw': 0.0}   # frontend default
    a = client.post(URL + '/graph', json=base).json()
    b = client.post(URL + '/graph', json=d).json()
    assert [(s['id'], s['x'], s['y'], s['yaw_deg']) for s in a['stops']] == \
           [(s['id'], s['x'], s['y'], s['yaw_deg']) for s in b['stops']]
    assert not any(s['pinned'] for s in b['stops'])


def test_a_manual_waypoint_becomes_the_stop(client, base):
    g0 = client.post(URL + '/graph', json=base).json()
    s0 = stop_of(g0, 'table_01')
    d = manual_at(base, 'table_01', s0['x'] - 0.3, s0['y'] - 0.02, s0['yaw_deg'])
    g = client.post(URL + '/graph', json=d).json()
    s = stop_of(g, 'table_01')
    assert s['pinned'] is True and abs(s['x'] - (s0['x'] - 0.3)) < 0.06 and s['yaw_deg'] == s0['yaw_deg'], (s0, s)
    assert g['ok'] is True and not g['excluded']
    for other in g0['stops']:
        if other['entity_id'] != 'table_01':
            now = stop_of(g, other['entity_id'])
            assert (round(now['x'], 3), round(now['y'], 3), now['yaw_deg']) == \
                   (round(other['x'], 3), round(other['y'], 3), other['yaw_deg']), other['id']
            assert now['pinned'] is False


def test_a_manual_stop_that_fails_is_reported_with_its_reason_and_place(client, base):
    s0 = stop_of(client.post(URL + '/graph', json=base).json(), 'table_01')
    away = (s0['yaw_deg'] + 180.0 + 180.0) % 360.0 - 180.0           # looking the other way
    g = client.post(URL + '/graph', json=manual_at(base, 'table_01', s0['x'], s0['y'], away)).json()
    assert g['ok'] is False
    ex = next(x for x in g['excluded'] if x['entity_id'] == 'table_01')
    assert '지정한' in ex['reason_ko'] and ex['pin']['x'] == s0['x'] and ex['pin']['yaw_deg'] == pytest.approx(away, abs=0.01)
    assert all(s['entity_id'] != 'table_01' for s in g['stops'])      # nothing was put in its place
    far = client.post(URL + '/graph', json=manual_at(base, 'table_01', 40.0, 40.0, 90.0)).json()
    assert '통로' in next(x for x in far['excluded'] if x['entity_id'] == 'table_01')['reason_ko']


def test_post_graph_checks_an_unsaved_edit_and_saves_nothing(client, base, tmp_path):
    before = common_hash()
    s0 = stop_of(client.post(URL + '/graph', json=base).json(), 'table_01')
    d = manual_at(base, 'table_01', s0['x'] - 0.3, s0['y'], s0['yaw_deg'])
    assert client.post(URL + '/graph', json=d).status_code == 200
    assert not (tmp_path / 'generated').exists() and not (tmp_path / 'semantics').exists()
    assert client.get(URL + '/status').json()['draft'] is None
    assert common_hash() == before


def test_post_graph_errors(client, base):
    assert client.post('/api/stores/other_store/semantics/graph', json=base).status_code == 404
    assert client.post(URL + '/graph', content='{', headers={'content-type': 'application/json'}).status_code == 400
    r = client.post(URL + '/graph', json={**base, 'entities': 5})
    assert r.status_code == 400 and r.json()['code'] == 'INVALID_SEMANTICS'
    r = client.post(URL + '/graph', json={**base, 'map_revision': base['map_revision'] + 1})
    assert r.status_code == 409


def test_manual_flag_must_be_a_boolean(client, base):
    d = copy.deepcopy(base)
    d['waypoints'][0]['manual'] = 'yes'
    r = client.put(URL, json=d)
    assert r.status_code == 400 and 'manual' in r.json()['message']
    d['waypoints'][0]['manual'] = True
    assert client.put(URL, json=d).status_code == 200


def test_a_good_manual_stop_can_be_applied(client, base):
    s0 = stop_of(client.post(URL + '/graph', json=base).json(), 'table_01')
    d = manual_at(base, 'table_01', s0['x'] - 0.3, s0['y'], s0['yaw_deg'])
    assert client.put(URL, json=d).status_code == 200
    r = client.post(URL + '/apply')
    assert r.status_code == 200, r.text
    g = client.get(URL + '/graph').json()
    assert g['matches_applied'] is True and stop_of(g, 'table_01')['pinned'] is True


def test_a_drawn_kitchen_is_named_kitchen_for_the_robot_side(client, base):
    d = copy.deepcopy(base)
    for e in d['entities']:
        if e['id'] == 'kitchen':
            e['id'] = 'kitchen_01'
    for w in d['waypoints']:
        if w['entity_id'] == 'kitchen':
            w['entity_id'] = 'kitchen_01'
            w['id'] = 'wp_kitchen_01'
    res = client.put(URL + '?apply=true', json=d).json()['apply_result']
    assert any('kitchen_01' in n and 'kitchen' in n for n in res['notes'])
    gen = client.app.state.semantics._gen_dir
    text = open(os.path.join(gen, 'waypoints.yaml'), encoding='utf-8').read()
    assert 'kitchen:' in text and 'kitchen_01' not in text


def test_only_the_first_of_two_kitchens_is_named_kitchen():
    from app.semantics import _label_id, _main_kitchen_id
    doc = {'entities': [{'id': 'kitchen_02', 'type': 'kitchen'}, {'id': 'kitchen_03', 'type': 'kitchen'}]}
    kid = _main_kitchen_id(doc)
    assert [_label_id(e, kid) for e in doc['entities']] == ['kitchen', 'kitchen_03']
    doc['entities'].append({'id': 'kitchen', 'type': 'kitchen'})
    kid = _main_kitchen_id(doc)
    assert [_label_id(e, kid) for e in doc['entities']] == ['kitchen_02', 'kitchen_03', 'kitchen']
