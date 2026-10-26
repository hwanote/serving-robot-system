"""Start Mapping -> auto draft -> review -> Save Map -> the store's map is replaced.
Works on a copy of maps/ and config/, with the fake runner instead of ROS."""
import os
import shutil
import sys
import time

import pytest
import yaml
from fastapi.testclient import TestClient

from .test_api import make

REAL = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..', '..'))
RUNNER = os.path.join(os.path.dirname(__file__), 'fake_mapping_runner.py')


@pytest.fixture()
def repo(tmp_path):
    r = tmp_path / 'repo'
    for rel in ('config', 'maps', 'ros_ws/src/serving_robot_navigation/serving_robot_navigation',
                'tools/site_builder/site_builder'):
        shutil.copytree(os.path.join(REAL, rel), r / rel, ignore=shutil.ignore_patterns('__pycache__', 'out'))
    return r


def build(tmp_path, repo, mode='finish', **kw):
    os.environ['FAKE_MAP_MODE'] = mode
    app = make(repo=str(repo), data_dir=str(tmp_path / 'data'), mapping_enabled=True,
               mapping_runner=RUNNER, mapping_ros_python=sys.executable,
               mapping_require_ros_env=False, **kw)
    app.state.mapping.cfg.ready_check = lambda: None
    return app


@pytest.fixture()
def client(tmp_path, repo):
    with TestClient(build(tmp_path, repo)) as c:
        c.repo = repo
        yield c


def wait_ready(c, secs=15):
    end = time.time() + secs
    while time.time() < end:
        st = c.get('/api/mapping/status').json()
        if st['phase'] == 'stopped':
            return st
        time.sleep(0.2)
    raise AssertionError(st)


def runner_up(c, secs=10):
    """The hanging fake runner has written its log, so its Stop handler is installed."""
    end = time.time() + secs
    while time.time() < end:
        if c.get('/api/mapping/status').json()['progress']['goals_sent'] >= 4:
            return
        time.sleep(0.05)
    raise AssertionError('runner did not come up')


def test_status_contract_when_idle(client):
    st = client.get('/api/mapping/status').json()
    assert st['phase'] == 'idle' and st['store_id'] == 'demo_store'
    assert {'started_at', 'last_saved_revision', 'draft_ready', 'stage', 'progress'} <= set(st)


def test_disabled_backend_answers_503(tmp_path, repo):
    app = make(repo=str(repo), data_dir=str(tmp_path / 'd'))
    with TestClient(app) as c:
        r = c.post('/api/mapping/start')
        assert r.status_code == 503 and r.json()['code'] == 'MAPPING_DISABLED'


def test_full_flow_replaces_the_map(client):
    mdir = client.repo / 'maps' / 'stores' / 'demo_store'
    old_pgm = (mdir / 'map.pgm').read_bytes()
    old_rev = client.get('/api/stores/demo_store/map').json()['map_revision']
    # a saved editor draft exists before
    sem = client.get('/api/stores/demo_store/semantics').json()
    assert client.put('/api/stores/demo_store/semantics', json=sem).status_code == 200

    assert client.post('/api/mapping/start').status_code == 200
    st = wait_ready(client)
    assert st['draft_ready'] and st['error'] is None
    info = client.get('/api/stores/demo_store/map-review').json()
    assert info['width'] == 12 and client.get(info['image_url']).headers['content-type'] == 'image/png'

    # Save Map without the final confirmation is refused, nothing changes
    r = client.post('/api/mapping/save', json={'source_hash': info['source_hash'], 'strokes': []})
    assert r.status_code == 422
    assert (mdir / 'map.pgm').read_bytes() == old_pgm

    r = client.post('/api/mapping/save', json={'source_hash': info['source_hash'],
                                              'confirmed': True, 'strokes': []})
    assert r.status_code == 200, r.text
    out = r.json()
    assert out['adopted'] is True and out['needs_robot_restart'] is True and out['adopt_error'] is None
    assert '로봇을 재시작' in out['message']
    new_map = client.get('/api/stores/demo_store/map').json()
    assert new_map['width'] == 12 and new_map['map_revision'] == old_rev + 1
    assert yaml.safe_load((mdir / 'map.yaml').read_text())['origin'][:2] == [0.0, 0.0]
    assert (mdir / 'map.pgm').read_bytes() != old_pgm
    # the old map is kept, with the editor's draft
    b = client.get('/api/stores/demo_store/map/backups').json()
    assert len(b) == 1 and b[0]['has_semantics']
    bdir = os.path.join(str(client.repo.parent), 'data', 'map_backups', 'demo_store', b[0]['id'])
    assert (open(os.path.join(bdir, 'map', 'map.pgm'), 'rb').read() == old_pgm)
    assert os.path.isfile(os.path.join(bdir, 'semantics.json'))
    # the editor starts empty on the new map, old drafts for the old revision are refused
    ed = client.get('/api/stores/demo_store/semantics').json()
    assert ed['entities'] == [] and ed['map_revision'] == new_map['map_revision']
    r = client.put('/api/stores/demo_store/semantics', json={**sem})
    assert r.status_code == 409
    # back to idle and a new mapping can start
    assert client.get('/api/mapping/status').json()['phase'] == 'idle'
    assert client.post('/api/mapping/start').status_code == 200
    wait_ready(client)


def test_origin_shift_is_applied(tmp_path, repo):
    with TestClient(build(tmp_path, repo, mapping_shift_origin=[-3.2, -2.4])) as c:
        c.post('/api/mapping/start')
        info = None
        wait_ready(c)
        info = c.get('/api/stores/demo_store/map-review').json()
        r = c.post('/api/mapping/save', json={'source_hash': info['source_hash'], 'confirmed': True, 'strokes': []})
        assert r.json()['adopted']
        o = c.get('/api/stores/demo_store/map').json()['origin']
        assert (round(o['x'], 3), round(o['y'], 3)) == (-3.2, -2.4)


def test_no_orders_and_no_idle_home_while_mapping(tmp_path, repo):
    with TestClient(build(tmp_path, repo, mode='hang')) as c:
        assert c.post('/api/mapping/start').status_code == 200
        r = c.post('/api/tasks', json={'type': 'SERVE', 'destination_id': 'table_01'})
        assert r.status_code == 409 and r.json()['code'] == 'MAPPING_BUSY'
        assert c.get('/api/tasks').json() == []
        runner_up(c)
        c.post('/api/mapping/stop')
        wait_ready(c)
        r = c.post('/api/tasks', json={'type': 'SERVE', 'destination_id': 'table_01'})
        assert r.status_code in (200, 201)


def test_start_refused_while_orders_active(tmp_path, repo):
    with TestClient(build(tmp_path, repo)) as c:
        assert c.post('/api/tasks', json={'type': 'SERVE', 'destination_id': 'table_01'}).status_code in (200, 201)
        r = c.post('/api/mapping/start')
        assert r.status_code == 409 and r.json()['code'] == 'ORDERS_ACTIVE'


def test_adopt_refused_while_orders_active_then_retry(client):
    client.post('/api/mapping/start')
    info = (wait_ready(client), client.get('/api/stores/demo_store/map-review').json())[1]
    fleet = client.app.state.fleet
    fleet.queue.append('fake')                       # an order is waiting
    r = client.post('/api/mapping/save', json={'source_hash': info['source_hash'],
                                              'confirmed': True, 'strokes': []})
    assert r.status_code == 200
    assert r.json()['adopted'] is False and r.json()['adopt_error']['code'] == 'ORDERS_ACTIVE'
    assert client.get('/api/stores/demo_store/map').json()['width'] == 180        # untouched
    fleet.queue.clear()
    r = client.post('/api/mapping/adopt')
    assert r.status_code == 200 and r.json()['adopted']
    assert client.get('/api/stores/demo_store/map').json()['width'] == 12


def test_review_and_save_without_a_draft(client):
    assert client.get('/api/stores/demo_store/map-review').status_code == 409
    r = client.post('/api/mapping/save', json={'confirmed': True, 'strokes': []})
    assert r.status_code == 409 and r.json()['code'] == 'NO_DRAFT'
    assert client.post('/api/mapping/adopt').status_code == 409
    assert client.post('/api/mapping/stop').status_code == 409
    assert client.get('/api/stores/other/map-review').status_code == 404


def test_pending_draft_blocks_start_until_discarded(client):
    client.post('/api/mapping/start')
    wait_ready(client)
    r = client.post('/api/mapping/start')
    assert r.status_code == 409 and r.json()['code'] == 'DRAFT_PENDING'
    assert client.post('/api/mapping/discard').status_code == 200
    assert client.post('/api/mapping/start').status_code == 200
    wait_ready(client)


def test_map_revision_survives_a_restart_of_the_backend(tmp_path, repo):
    with TestClient(build(tmp_path, repo)) as c:
        c.post('/api/mapping/start')
        info = (wait_ready(c), c.get('/api/stores/demo_store/map-review').json())[1]
        r = c.post('/api/mapping/save', json={'source_hash': info['source_hash'],
                                              'confirmed': True, 'strokes': []})
        assert r.json()['adopted']
        rev = c.get('/api/stores/demo_store/map').json()['map_revision']
        assert rev == 2
    with TestClient(build(tmp_path, repo)) as c2:                     # backend started again
        assert c2.get('/api/stores/demo_store/map').json()['map_revision'] == rev
        assert c2.get('/api/stores/demo_store/semantics').json()['map_revision'] == rev
