import time

import pytest
from fastapi.testclient import TestClient

from app.config import Settings, default_repo_root, parse_robots
from app.main import create_app


def make(**kw):
    base = dict(repo=default_repo_root(), fake_speed=4.0, fake_wait=0.3, stagger_s=0.0)
    base.update(kw)
    return create_app(Settings(**base))


def wait_status(c, tid, want, timeout=40):
    end = time.time() + timeout
    while time.time() < end:
        for t in c.get('/api/tasks').json():
            if t['task_id'] == tid and t['status'] in want:
                return t
        time.sleep(0.3)
    raise AssertionError('timeout waiting for %s' % tid)


@pytest.fixture()
def client():
    with TestClient(make()) as c:
        yield c


def test_validation(client):
    post = lambda b: client.post('/api/tasks', json=b)
    assert post({'type': 'MOVE', 'destination_id': 'table_01'}).status_code == 400
    assert post({'type': 'SERVE', 'destination_id': 'table_01', 'x': 1}).status_code == 400
    assert post({'type': 'SERVE'}).status_code == 400
    assert post({'type': 'SERVE', 'destination_id': 'nope'}).status_code == 422
    assert client.post('/api/tasks', content='not json').status_code == 400


def test_maps(client):
    assert client.get('/api/stores/demo_store/map.png').status_code == 200
    assert client.get('/api/stores/other/map').status_code == 404
    assert client.post('/api/mapping/start').status_code == 503   # mapping is off unless mapping_enabled


def test_order_runs_to_success(client):
    t = client.post('/api/tasks', json={'type': 'SERVE', 'destination_id': 'table_01'}).json()
    done = wait_status(client, t['task_id'], ('SUCCEEDED', 'FAILED'))
    assert done['status'] == 'SUCCEEDED' and done['robot_id'] == 'robot_01'


def test_cancel_and_conflict(client):
    t = client.post('/api/tasks', json={'type': 'SERVE', 'destination_id': 'table_03'}).json()
    r = client.post('/api/tasks/%s/cancel' % t['task_id'])
    assert r.status_code == 200 and r.json()['status'] == 'CANCELED'
    assert client.post('/api/tasks/%s/cancel' % t['task_id']).status_code == 409
    assert client.post('/api/tasks/task-9999/cancel').status_code == 404


def test_ws_snapshot(client):
    with client.websocket_connect('/ws/state') as ws:
        kinds = {ws.receive_json()['type'] for _ in range(2)}
    assert kinds <= {'task_state', 'robot_state', 'alarm'}


def test_two_robots_both_work():
    app = make(robots=parse_robots('robot_01,robot_02'), zones={'table_01': 'robot_01',
               'table_02': 'robot_02'})
    with TestClient(app) as c:
        a = c.post('/api/tasks', json={'type': 'SERVE', 'destination_id': 'table_01'}).json()
        b = c.post('/api/tasks', json={'type': 'SERVE', 'destination_id': 'table_02'}).json()
        da = wait_status(c, a['task_id'], ('SUCCEEDED', 'FAILED'), 90)
        db = wait_status(c, b['task_id'], ('SUCCEEDED', 'FAILED'), 90)
    assert da['robot_id'] == 'robot_01' and db['robot_id'] == 'robot_02'


def test_idempotency_and_snapshot():
    with TestClient(make()) as c:
        body = {'type': 'SERVE', 'destination_id': 'table_01', 'client_request_id': 'abc-1'}
        a = c.post('/api/tasks', json=body)
        b = c.post('/api/tasks', json=body)
        assert a.status_code == 201 and b.status_code == 200
        assert a.json()['task_id'] == b.json()['task_id']
        assert len(c.get('/api/tasks').json()) == 1
        body2 = dict(body, destination_id='table_03')
        assert c.post('/api/tasks', json=body2).status_code == 409
        snap = c.get('/api/snapshot').json()
        assert snap['tasks'][0]['task_id'] == a.json()['task_id'] and 'robots' in snap


def test_history_survives_restart(tmp_path):
    url = 'sqlite:///%s' % (tmp_path / 't.db').as_posix()
    with TestClient(make(db_url=url, stagger_s=0.0)) as c:
        t = c.post('/api/tasks', json={'type': 'SERVE', 'destination_id': 'table_01'}).json()
        wait_status(c, t['task_id'], ('SUCCEEDED',))
    with TestClient(make(db_url=url)) as c:
        tasks = c.get('/api/tasks').json()
        assert tasks[0]['task_id'] == t['task_id'] and tasks[0]['status'] == 'SUCCEEDED'
        n = c.post('/api/tasks', json={'type': 'SERVE', 'destination_id': 'table_02'}).json()
        assert n['task_id'] != t['task_id']


def test_restart_recovers_unfinished(tmp_path):
    from app.repository.sql import SqlRepository
    url = 'sqlite:///%s' % (tmp_path / 'r.db').as_posix()
    repo = SqlRepository(url)
    for tid, st in (('task-0001', 'QUEUED'), ('task-0002', 'RUNNING')):
        repo.save_task({'task_id': tid, 'type': 'SERVE', 'destination_id': 'table_01',
                        'status': st, 'robot_id': None if st == 'QUEUED' else 'robot_01',
                        'created_at': 'x', 'updated_at': 'x'})
    app = make(db_url=url)
    app.state.fleet.recover()
    st = {t['task_id']: t for t in app.state.repo.list_tasks()}
    assert st['task-0002']['status'] == 'FAILED' and st['task-0002']['detail'] == 'SERVER_RESTART'
    assert app.state.fleet.queue == ['task-0001']


def test_error_codes(client):
    r = client.post('/api/tasks', json={'type': 'SERVE', 'destination_id': 'nope'})
    assert r.json()['code'] == 'INVALID_DESTINATION' and r.json()['message']
    assert client.post('/api/tasks/task-9/cancel').json()['code'] == 'TASK_NOT_FOUND'


def test_pause_robot_and_dispatch():
    with TestClient(make()) as c:
        assert c.post('/api/robots/robot_01/pause').json()['accepting_orders'] is False
        assert c.post('/api/robots/nobody/pause').status_code == 404
        t = c.post('/api/tasks', json={'type': 'SERVE', 'destination_id': 'table_01'}).json()
        time.sleep(3)
        assert c.get('/api/tasks').json()[0]['status'] == 'QUEUED'   # nobody may take it
        c.post('/api/robots/robot_01/resume')
        assert wait_status(c, t['task_id'], ('SUCCEEDED',))['robot_id'] == 'robot_01'
        c.post('/api/dispatch/pause')
        t2 = c.post('/api/tasks', json={'type': 'SERVE', 'destination_id': 'table_02'}).json()
        time.sleep(3)
        assert c.get('/api/debug').json()['dispatch_paused'] is True
        assert [x for x in c.get('/api/tasks').json() if x['task_id'] == t2['task_id']][0][
            'status'] == 'QUEUED'


def test_recover_needs_recovered_state():
    with TestClient(make()) as c:
        assert c.post('/api/robots/robot_01/recover', json={'action': 'x'}).status_code == 400
        time.sleep(1)
        r = c.post('/api/robots/robot_01/recover', json={'action': 'resume'})
        assert r.status_code == 409 and r.json()['code'] == 'NOT_RECOVERABLE'


def test_offline_robot_fails_order_and_gets_no_new_one():
    app = make(offline_after_s=1.5, offline_fail_s=2.0)
    with TestClient(app) as c:
        time.sleep(1)
        t = c.post('/api/tasks', json={'type': 'SERVE', 'destination_id': 'table_04'}).json()
        wait_status(c, t['task_id'], ('RUNNING',))
        app.state.fleet.robots['robot_01'].adapter.muted = True      # robot goes silent
        done = wait_status(c, t['task_id'], ('FAILED', 'SUCCEEDED'), 20)
        assert done['status'] == 'FAILED' and done['detail'] == 'ROBOT_OFFLINE'
        rb = [r for r in c.get('/api/robots').json() if r['robot_id'] == 'robot_01'][0]
        assert rb['online'] is False and rb['state'] == 'ERROR'
        t2 = c.post('/api/tasks', json={'type': 'SERVE', 'destination_id': 'table_01'}).json()
        time.sleep(3)
        assert [x for x in c.get('/api/tasks').json() if x['task_id'] == t2['task_id']][0][
            'status'] == 'QUEUED'
        app.state.fleet.robots['robot_01'].adapter.muted = False     # comes back
        assert wait_status(c, t2['task_id'], ('SUCCEEDED',), 60)


def test_unacknowledged_command_fails():
    app = make(ack_timeout_s=2.0)
    with TestClient(app) as c:
        time.sleep(1)
        app.state.fleet.robots['robot_01'].adapter.deaf = True
        t = c.post('/api/tasks', json={'type': 'SERVE', 'destination_id': 'table_01'}).json()
        done = wait_status(c, t['task_id'], ('FAILED', 'SUCCEEDED'), 15)
        assert done['status'] == 'FAILED' and done['detail'] == 'COMMAND_NOT_ACKED'


def test_metrics_and_priority():
    app = make(priorities={'table_04': 9}, stagger_s=0.0)
    with TestClient(app) as c:
        app.state.fleet.set_dispatch_paused(True)
        a = c.post('/api/tasks', json={'type': 'SERVE', 'destination_id': 'table_01'}).json()
        b = c.post('/api/tasks', json={'type': 'SERVE', 'destination_id': 'table_04'}).json()
        assert app.state.fleet._head() == b['task_id']
        app.state.fleet.set_dispatch_paused(False)
        wait_status(c, a['task_id'], ('SUCCEEDED',), 90)
        m = c.get('/api/metrics').json()
        assert m['tasks']['by_status']['SUCCEEDED'] == 2 and m['service_s']['count'] == 2
        assert m['wait_s']['avg'] is not None


def test_modes_and_config_file(tmp_path):
    from app.__main__ import parse_args, settings_from_args
    f = tmp_path / 'f.yaml'
    f.write_text("mode: B\nrobots:\n  - {name: robot_01, domain: 0}\n  - {name: robot_02, domain: 2}\n"
                 "zones:\n  robot_01: [table_01]\npriorities: {table_02: 3}\nstagger_s: 4\n")
    s = settings_from_args(parse_args(['--config', str(f), '--port', '9001']))
    assert s.mode == 'B' and s.route_check is False and s.zones == {} and s.port == 9001
    assert s.stagger_s == 4 and s.priorities == {'table_02': 3} and len(s.robots) == 2
    s = settings_from_args(parse_args(['--mode', 'A', '--robots', 'robot_01,robot_02']))
    assert [n for n, _ in s.robots] == ['robot_01']
    s = settings_from_args(parse_args(['--zones', 'robot_01:table_01']))
    assert s.mode == 'C' and s.zones == {'table_01': 'robot_01'}


def test_booting_or_error_robot_gets_no_order():
    app = make()
    with TestClient(app) as c:
        fleet = app.state.fleet
        time.sleep(1)
        r = fleet.robots['robot_01']
        r.adapter.sink = type('S', (), {'on_status': fleet.on_status, 'on_pose': fleet.on_pose,
                                        'on_alarm': fleet.on_alarm,
                                        'on_health': lambda self, n, h: fleet.on_health(
                                            n, {'robot_state': 'BOOTING'})})()
        time.sleep(1)
        t = c.post('/api/tasks', json={'type': 'SERVE', 'destination_id': 'table_01'}).json()
        time.sleep(3)
        assert [x for x in c.get('/api/tasks').json() if x['task_id'] == t['task_id']][0][
            'status'] == 'QUEUED'
        r.adapter.sink = fleet                                       # healthy again
        assert wait_status(c, t['task_id'], ('SUCCEEDED',), 60)
