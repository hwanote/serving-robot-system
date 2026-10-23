"""Putting an applied map into config/common, dated backups, restore.
Every test works on a copy of the repository files, never on the real config/common."""
import copy
import hashlib
import os
import shutil

import pytest
from fastapi.testclient import TestClient

from .test_api import make
from .test_semantics import without_entity

REAL = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..', '..'))
URL = '/api/stores/demo_store/semantics'


def tree_hash(folder):
    h = hashlib.sha256()
    for name in sorted(os.listdir(folder)):
        with open(os.path.join(folder, name), 'rb') as f:
            h.update(name.encode() + f.read())
    return h.hexdigest()


@pytest.fixture()
def repo(tmp_path):
    r = tmp_path / 'repo'
    for rel in ('config', 'maps', 'ros_ws/src/serving_robot_navigation/serving_robot_navigation',
                'tools/site_builder/site_builder'):
        shutil.copytree(os.path.join(REAL, rel), r / rel, ignore=shutil.ignore_patterns('__pycache__', 'out'))
    return r


@pytest.fixture()
def client(tmp_path, repo):
    with TestClient(make(repo=str(repo), data_dir=str(tmp_path / 'data'))) as c:
        c.repo = repo
        yield c


@pytest.fixture()
def base(client):
    return client.get(URL).json()


def changed(base, n=4):
    """A valid different map: one table less."""
    return without_entity(base, 'table_%02d' % n)


def apply(client, doc):
    r = client.put(URL + '?apply=true', json=doc)
    assert r.status_code == 200, r.text
    return r.json()


def live_dir(client):
    return str(client.repo / 'config' / 'common')


def test_promote_writes_config_common_and_keeps_a_dated_backup(client, base):
    real_before = tree_hash(os.path.join(REAL, 'config', 'common'))
    before = tree_hash(live_dir(client))
    apply(client, changed(base))
    r = client.post(URL + '/promote')
    assert r.status_code == 200, r.text
    out = r.json()
    assert out['live']['needs_robot_restart'] is True
    bid = out['backup']['id']
    assert len(bid) == 15 and bid[8] == '-'                      # YYYYMMDD-HHMMSS
    assert tree_hash(live_dir(client)) != before
    assert tree_hash(os.path.join(REAL, 'config', 'common')) == real_before   # tests never touch the real one
    backups = client.get(URL + '/backups').json()
    assert [b['id'] for b in backups] == [bid]
    assert backups[0]['kind'] == 'promote' and backups[0]['summary']['tables'] >= 4
    st = client.get(URL + '/live').json()
    assert st['matches_applied'] is True and st['last']['backup_id'] == bid and st['backup_count'] == 1
    # the backend itself uses the new map without a restart
    assert 'table_04' not in client.app.state.site.tables


def test_needs_an_applied_result_that_matches_the_draft(client, base):
    assert client.post(URL + '/promote').json()['code'] == 'NOT_APPLIED'
    apply(client, changed(base))
    client.put(URL, json=changed(base, 3))                         # draft moved on after the apply
    r = client.post(URL + '/promote')
    assert r.status_code == 409 and r.json()['code'] == 'NOT_IN_SYNC'


def test_same_map_is_not_written_again(client, base):
    apply(client, changed(base))
    assert client.post(URL + '/promote').status_code == 200
    r = client.post(URL + '/promote')
    assert r.status_code == 409 and r.json()['code'] == 'ALREADY_LIVE'


def test_refused_while_an_order_is_running(client, base):
    apply(client, changed(base))
    before = tree_hash(live_dir(client))
    assert client.post('/api/tasks', json={'type': 'SERVE', 'destination_id': 'table_01'}).status_code in (200, 201)
    r = client.post(URL + '/promote')
    assert r.status_code == 409 and r.json()['code'] == 'ORDERS_ACTIVE'
    assert tree_hash(live_dir(client)) == before


def test_several_backups_stay_and_any_can_be_restored(client, base):
    original = tree_hash(live_dir(client))
    apply(client, changed(base, 4))
    first = client.post(URL + '/promote').json()['backup']['id']
    second_map = tree_hash(live_dir(client))
    apply(client, changed(base, 3))
    second = client.post(URL + '/promote').json()['backup']['id']   # often the same second
    assert first != second
    ids = [b['id'] for b in client.get(URL + '/backups').json()]
    assert ids == sorted(ids, reverse=True) and set(ids) == {first, second}
    # bring back the very first (the original demo map), not just the previous one
    r = client.post('%s/backups/%s/restore' % (URL, first))
    assert r.status_code == 200, r.text
    assert tree_hash(live_dir(client)) == original
    assert 'table_04' in client.app.state.site.tables
    kinds = {b['id']: b['kind'] for b in client.get(URL + '/backups').json()}
    assert len(kinds) == 3 and list(kinds.values()).count('before_restore') == 1
    # the restore can be undone too: the map before it is kept
    undo = r.json()['backup']['id']
    assert client.post('%s/backups/%s/restore' % (URL, undo)).status_code == 200
    assert 'table_03' not in client.app.state.site.tables
    assert second_map != tree_hash(live_dir(client))


def test_unknown_or_tricky_backup_ids(client):
    for bid in ('nope', '..', '20250101-000000'):
        r = client.post('%s/backups/%s/restore' % (URL, bid))
        assert r.status_code in (404, 405), (bid, r.status_code)


def test_a_robot_without_a_standby_pose_blocks_it(tmp_path, repo):
    with TestClient(make(repo=str(repo), data_dir=str(tmp_path / 'd2'),
                         robots=[('robot_01', None), ('robot_09', None)])) as c:
        c.repo = repo
        base = c.get(URL).json()
        apply(c, changed(base))
        before = tree_hash(live_dir(c))
        r = c.post(URL + '/promote')
        assert r.status_code == 422 and r.json()['code'] == 'MISSING_STANDBY'
        assert 'robot_09' in r.json()['details']['robots']
        assert tree_hash(live_dir(c)) == before


def test_a_failure_while_switching_puts_the_old_map_back(client, base, monkeypatch):
    apply(client, changed(base))
    before = tree_hash(live_dir(client))
    site = client.app.state.site
    real_reload, calls = site.reload, []

    def broken():
        calls.append(1)
        if len(calls) == 1:
            raise RuntimeError('boom')
        real_reload()
    monkeypatch.setattr(site, 'reload', broken)
    r = client.post(URL + '/promote')
    assert r.status_code == 500 and r.json()['code'] == 'PROMOTE_FAILED'
    assert tree_hash(live_dir(client)) == before


def ids(doc):
    return sorted(e['id'] for e in doc['entities'])


def test_restore_brings_the_editor_draft_back_too(client, base):
    apply(client, changed(base, 4))
    first = client.post(URL + '/promote').json()['backup']
    assert first['has_semantics'] is False            # the original map had no saved draft
    apply(client, changed(base, 3))
    second = client.post(URL + '/promote').json()['backup']
    assert second['has_semantics'] is True            # = the draft that made table_04's map
    # the editor now shows something newer than anything live
    assert client.put(URL, json=base).status_code == 200
    r = client.post('%s/backups/%s/restore' % (URL, second['id']))
    assert r.status_code == 200, r.text
    body = r.json()
    assert body['semantics_restored'] is True and body['semantics_note'] is None
    assert ids(body['semantics']) == ids(changed(base, 4))
    assert ids(client.get(URL).json()) == ids(changed(base, 4))     # what the editor will load
    # the draft is "not applied" now (the applied result is the newer one)
    st = client.get(URL + '/status').json()
    assert st['in_sync'] is False
    # undo the restore: map and editor go back to what they were before it
    undo = body['backup']
    assert undo['has_semantics'] is True
    r2 = client.post('%s/backups/%s/restore' % (URL, undo['id']))
    assert r2.status_code == 200
    assert ids(client.get(URL).json()) == ids(base)


def test_an_old_backup_without_a_draft_rebuilds_the_editor_from_its_map_files(client, base):
    original = tree_hash(live_dir(client))
    moved = copy.deepcopy(base)
    home = next(e for e in moved['entities'] if e['type'] == 'robot_wait')
    home['pose']['x'] += 0.2                       # the robot's waiting spot was moved
    apply(client, moved)
    old = client.post(URL + '/promote').json()['backup']
    assert old['has_semantics'] is False           # made before drafts were kept
    assert ids(client.get(URL).json()) == ids(base)
    r = client.post('%s/backups/%s/restore' % (URL, old['id']))
    assert r.status_code == 200
    body = r.json()
    assert body['semantics_restored'] is True
    assert '지도 파일에서 다시 만들었어요' in body['semantics_note']
    assert tree_hash(live_dir(client)) == original
    back = client.get(URL).json()
    assert back['entities'] == base['entities']     # the waiting spot is where it was
    assert back['zones'] == []
    listed = {b['id']: b['has_semantics'] for b in client.get(URL + '/backups').json()}
    assert listed[old['id']] is False
    # and the restore can be undone: the moved spot comes back in the editor
    undo = body['backup']['id']
    assert client.post('%s/backups/%s/restore' % (URL, undo)).status_code == 200
    moved_back = next(e for e in client.get(URL).json()['entities'] if e['id'] == home['id'])
    assert moved_back['pose']['x'] == home['pose']['x']


def test_a_hand_edited_common_is_not_paired_with_a_stale_draft(client, base):
    apply(client, changed(base, 4))
    client.post(URL + '/promote')
    with open(os.path.join(live_dir(client), 'zones.yaml'), 'a', encoding='utf-8') as f:
        f.write('\n# edited by hand\n')
    apply(client, changed(base, 3))
    second = client.post(URL + '/promote').json()['backup']
    assert second['has_semantics'] is False                  # common no longer matches the draft
