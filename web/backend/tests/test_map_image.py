"""The robot's map picture vs the store's: map_image_current, dispatch exclusion, restart."""
import time

from .test_mapping_api import build, client, repo, wait_ready  # noqa: F401  (fixtures)

ORDER = {'type': 'SERVE', 'destination_id': 'table_01'}


def robots(c):
    return {r['robot_id']: r for r in c.get('/api/robots').json()}


def wait_for(fn, secs=10):
    end = time.time() + secs
    while time.time() < end:
        v = fn()
        if v:
            return v
        time.sleep(0.2)
    raise AssertionError('timeout')


def adopt_new_map(c):
    c.post('/api/mapping/start')
    wait_ready(c)
    info = c.get('/api/stores/demo_store/map-review').json()
    r = c.post('/api/mapping/save', json={'source_hash': info['source_hash'], 'confirmed': True, 'strokes': []})
    assert r.json()['adopted'] is True


def test_robots_start_with_the_current_picture(client):
    wait_for(lambda: all(r.get('map_image_current') is True for r in robots(client).values()))


def test_adopting_a_new_map_marks_every_robot_for_restart_and_holds_orders(client):
    wait_for(lambda: all(r.get('map_image_current') is True for r in robots(client).values()))
    adopt_new_map(client)
    now = robots(client)
    assert all(r['map_image_current'] is False for r in now.values())
    task = client.post('/api/tasks', json=ORDER).json()
    time.sleep(2.0)
    t = [x for x in client.get('/api/tasks').json() if x['task_id'] == task['task_id']][0]
    assert t['status'] == 'QUEUED' and t['robot_id'] is None
    assert any('restarted' in line for line in client.app.state.fleet.log)


def test_a_restarted_robot_serves_again(client):
    wait_for(lambda: all(r.get('map_image_current') is True for r in robots(client).values()))
    adopt_new_map(client)
    task = client.post('/api/tasks', json=ORDER).json()
    # "restart": the robots now report the picture the store has
    for r in client.app.state.fleet.robots.values():
        r.adapter.map_image = r.adapter._live_map_image()
    wait_for(lambda: all(r['map_image_current'] is True for r in robots(client).values()))
    done = wait_for(lambda: [x for x in client.get('/api/tasks').json()
                             if x['task_id'] == task['task_id'] and x['status'] != 'QUEUED'])
    assert done[0]['robot_id'] in ('robot_01',)


def test_robot_that_does_not_report_a_picture_is_not_excluded(client):
    fleet = client.app.state.fleet
    for r in fleet.robots.values():
        r.adapter.map_image = ''                     # an older robot node
    wait_for(lambda: all(r.get('map_image_current') is None for r in robots(client).values()))
    task = client.post('/api/tasks', json=ORDER).json()
    wait_for(lambda: [x for x in client.get('/api/tasks').json()
                      if x['task_id'] == task['task_id'] and x['status'] != 'QUEUED'])
