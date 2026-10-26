"""Robot card buttons: reset, home, read the map again. Fake robots, repository copy."""
import time

import pytest
from fastapi.testclient import TestClient

from app.config import parse_robots

from .test_api import make, wait_status
from .test_promote import URL, apply, changed, repo  # noqa: F401  (repo is a fixture)


def robot(c, rid='robot_01'):
    return [r for r in c.get('/api/robots').json() if r['robot_id'] == rid][0]


def until(fn, timeout=15):
    end = time.time() + timeout
    while time.time() < end:
        v = fn()
        if v:
            return v
        time.sleep(0.2)
    raise AssertionError('timeout')


@pytest.fixture()
def client(tmp_path, repo):  # noqa: F811
    app = make(repo=str(repo), data_dir=str(tmp_path / 'data'),
               robots=parse_robots('robot_01,robot_02'), mode='B')
    with TestClient(app) as c:
        c.app_, c.repo = app, repo
        until(lambda: robot(c).get('pose'))
        yield c


def adapter(c, rid='robot_01'):
    return c.app_.state.fleet.robots[rid].adapter


def drive_away(c, rid='robot_01'):
    """Leave the robot idle at table_01 (not at home)."""
    adapter(c, rid).send_cmd({'cmd': 'go', 'id': 'away', 'names': ['table_01']})
    until(lambda: robot(c, rid)['mission_state'] == 'DRIVING')
    until(lambda: robot(c, rid)['mission_state'] == 'IDLE', 30)


def test_state_carries_the_button_rules(client):
    r = until(lambda: robot(client) if robot(client)['controls']['reset'] else None)
    assert r['localization'] == 'OK' and r['map_current'] is True
    ctl = r['controls']
    assert ctl['reset_nodes'] == [r['home'], 'kitchen']
    assert ctl['home'] is False, 'already at home'
    assert ctl['reload_map'] is False, 'already on the newest map'


def test_reset_cancels_the_running_order_and_puts_the_robot_on_the_node(client):
    t = client.post('/api/tasks', json={'type': 'SERVE', 'destination_id': 'table_04'}).json()
    rid = wait_status(client, t['task_id'], ('RUNNING',))['robot_id']
    r = client.post('/api/robots/%s/reset' % rid, json={'node': 'kitchen'})
    assert r.status_code == 200, r.text
    assert wait_status(client, t['task_id'], ('CANCELED',))['detail'] == 'RESET'
    until(lambda: robot(client, rid)['mission_state'] == 'IDLE')
    x, y = client.app_.state.site.node_xy('kitchen')
    p = robot(client, rid)['pose']
    assert abs(p['x'] - x) < 0.3 and abs(p['y'] - y) < 0.3


def test_reset_refusals(client):
    assert client.post('/api/robots/nope/reset', json={}).status_code == 404
    r = client.post('/api/robots/robot_01/reset', json={'node': 'no_such_place'})
    assert r.status_code == 400 and r.json()['code'] == 'BAD_NODE'
    assert client.post('/api/robots/robot_01/reset', json={'node': 5}).status_code == 400
    client.app_.state.fleet.robots['robot_01'].adapter.muted = True
    client.app_.state.fleet.robots['robot_01'].offline = True
    r = client.post('/api/robots/robot_01/reset', json={})
    assert r.status_code == 409 and r.json()['code'] == 'ROBOT_OFFLINE'


def test_home_only_when_idle_localized_and_away(client):
    r = client.post('/api/robots/robot_01/home')
    assert r.status_code == 409 and r.json()['code'] == 'ALREADY_HOME'
    drive_away(client)
    r = until(lambda: robot(client) if robot(client)['controls']['home'] else None)
    adapter(client).localization = 'LOST'
    until(lambda: robot(client)['localization'] == 'LOST')
    r = client.post('/api/robots/robot_01/home')
    assert r.status_code == 409 and r.json()['code'] == 'NOT_LOCALIZED'
    assert robot(client)['controls']['home'] is False
    # reset brings the position back (the fake robot is put on the kitchen node)
    assert client.post('/api/robots/robot_01/reset', json={'node': 'kitchen'}).status_code == 200
    until(lambda: robot(client)['localization'] == 'OK' and robot(client)['controls']['home'])
    assert client.post('/api/robots/robot_01/home').status_code == 200
    home = client.app_.state.site.node_xy(robot(client)['home'])

    def arrived():
        p = robot(client)['pose']
        return abs(p['x'] - home[0]) < 0.3 and abs(p['y'] - home[1]) < 0.3
    until(arrived, 30)


def test_home_refused_while_driving(client):
    t = client.post('/api/tasks', json={'type': 'SERVE', 'destination_id': 'table_04'}).json()
    rid = wait_status(client, t['task_id'], ('RUNNING',))['robot_id']
    r = client.post('/api/robots/%s/home' % rid)
    assert r.status_code == 409 and r.json()['code'] == 'ROBOT_BUSY'


def test_reload_map_after_promote(client):
    base = client.get(URL).json()
    apply(client, changed(base))
    assert client.post(URL + '/promote').status_code == 200
    r = until(lambda: robot(client) if robot(client)['map_current'] is False else None)
    assert r['controls']['reload_map'] is True
    assert client.post('/api/robots/robot_01/reload-map').status_code == 200
    until(lambda: robot(client)['map_current'] is True)
    assert robot(client)['controls']['reload_map'] is False
    # the other robot still has the old map: the all-robots button reaches only it
    out = client.post('/api/robots/reload-map').json()['robots']
    assert out['robot_01'] == 'current' and out['robot_02'] == 'sent'
    until(lambda: robot(client, 'robot_02')['map_current'] is True)


def test_reload_map_refused_while_busy_or_unknown(client):
    base = client.get(URL).json()
    apply(client, changed(base))
    assert client.post(URL + '/promote').status_code == 200
    assert client.post('/api/robots/nope/reload-map').status_code == 404
    t = client.post('/api/tasks', json={'type': 'SERVE', 'destination_id': 'table_01'}).json()
    wait_status(client, t['task_id'], ('RUNNING',))
    busy = [r for r in client.get('/api/robots').json() if r['mission_state'] != 'IDLE']
    assert busy
    r = client.post('/api/robots/%s/reload-map' % busy[0]['robot_id'])
    assert r.status_code == 409 and r.json()['code'] == 'ROBOT_BUSY'
    assert busy[0]['controls']['reload_map'] is False


def test_stale_home_trip_does_not_keep_the_fleet_busy(client):
    """A 'going home' mark that never finishes must not block map adoption forever."""
    fleet = client.app_.state.fleet
    r = fleet.robots['robot_01']
    with fleet.lock:
        r.homing, r.homing_at = 'home-9999', time.monotonic() - 10     # robot never took it
        assert fleet.is_busy()
        fleet._watch()
        assert r.homing is None and not fleet.is_busy()
        r.homing, r.homing_at, r.offline = 'home-9998', time.monotonic(), True   # went silent
        fleet._watch()
        assert r.homing is None
        r.offline = False


def test_running_home_trip_is_kept(client):
    fleet = client.app_.state.fleet
    r = fleet.robots['robot_01']
    with fleet.lock:
        r.homing, r.homing_at = 'home-9997', time.monotonic()           # fresh: still waiting
        r.ms = dict(r.ms or {}, mission_id='home-9997')
        fleet._watch()
        assert r.homing == 'home-9997' and fleet.is_busy()
        r.homing = None


def test_no_idle_home_for_a_robot_that_does_not_know_where_it_is(client):
    fleet = client.app_.state.fleet
    r = fleet.robots['robot_01']
    sent = []
    real = r.adapter.send_cmd
    r.adapter.send_cmd = lambda cmd: sent.append(cmd) or real(cmd)
    try:
        with fleet.lock:
            r.ms = dict(r.ms or {}, localization='UNINITIALIZED')
            r.idle_since = time.monotonic() - 100
            fleet._idle_home()
            assert r.homing is None and not sent and not fleet.is_busy()
            r.ms = dict(r.ms, localization='OK')
            r.idle_since = time.monotonic() - 100
            fleet._idle_home()
            assert r.homing and sent and sent[0]['cmd'] == 'go'
            r.homing = None
    finally:
        r.adapter.send_cmd = real
