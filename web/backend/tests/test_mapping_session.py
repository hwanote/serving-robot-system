import os
import sys
import time

import pytest

from app.mapping.session import MappingConfig, MappingError, MappingSession

RUNNER = os.path.join(os.path.dirname(__file__), 'fake_mapping_runner.py')


def make(tmp_path, mode='finish', **kw):
    os.environ['FAKE_MAP_MODE'] = mode
    cfg = MappingConfig(store_id='demo_store', work_dir=str(tmp_path / 'map'), runner=RUNNER,
                        ros_python=sys.executable, enabled=True, require_ros_env=False,
                        ready_check=kw.pop('ready_check', lambda: None), **kw)
    return MappingSession(cfg)


def wait(s, phase='stopped', secs=15):
    end = time.time() + secs
    while time.time() < end:
        st = s.status()
        if st['phase'] == phase:
            return st
        time.sleep(0.1)
    raise AssertionError('still %s' % s.status())


def running(s, secs=10):
    """Wait until the (hanging) fake runner has written its log, i.e. its Stop handler is in place."""
    end = time.time() + secs
    while time.time() < end:
        if s.status()['progress']['goals_sent'] >= 4:
            return
        time.sleep(0.05)
    raise AssertionError('runner did not come up')


def test_disabled_by_default(tmp_path):
    cfg = MappingConfig(store_id='x', work_dir=str(tmp_path), runner=RUNNER)
    with pytest.raises(MappingError) as e:
        MappingSession(cfg).start()
    assert e.value.status == 503 and e.value.code == 'MAPPING_DISABLED'


def test_stack_not_ready(tmp_path):
    s = make(tmp_path, ready_check=lambda: 'stack down')
    with pytest.raises(MappingError) as e:
        s.start()
    assert (e.value.status, e.value.code) == (503, 'STACK_NOT_READY')
    assert s.status()['phase'] == 'idle'


def test_auto_ingest_without_stop(tmp_path):
    s = make(tmp_path)
    assert s.start()['phase'] == 'mapping'
    st = wait(s)
    assert st['draft_ready'] is True and st['stage'] == 'ready' and st['error'] is None
    assert st['progress']['percent'] == 100 and st['progress']['estimated'] is False
    assert st['progress']['goals_sent'] == 4 and st['progress']['goals_arrived'] == 3
    assert st['progress']['goals_failed'] == 1
    assert (tmp_path / 'map' / 'drafts' / 'current' / 'R16-147_official_raw_map.pgm').is_file()
    assert s.review_service().describe()['width'] == 12


def test_stop_cuts_early_and_makes_draft(tmp_path):
    s = make(tmp_path, mode='hang')
    s.start()
    running(s)
    assert s.status()['stage'] in ('starting', 'exploring')
    assert s.status()['progress']['percent'] < 100
    s.stop()
    st = wait(s)
    assert st['draft_ready'] and st['stop_requested'] and st['completion_reason'] == 'INTERRUPTED'


def test_percent_never_goes_down_and_stays_below_100(tmp_path):
    s = make(tmp_path, mode='hang')
    s.start()
    running(s)
    a = s.status()['progress']['percent']
    b = s.status()['progress']['percent']
    assert 0 < a <= b <= 99
    s.stop()
    wait(s)


def test_runner_crash_reports_failure(tmp_path):
    s = make(tmp_path, mode='crash')
    s.start()
    st = wait(s)
    assert st['draft_ready'] is False and st['stage'] == 'failed' and st['error']
    assert s.start()['phase'] == 'mapping'            # can try again, nothing is stuck
    wait(s)


def test_no_map_is_a_failure(tmp_path):
    s = make(tmp_path, mode='nomap')
    s.start()
    st = wait(s)
    assert not st['draft_ready'] and 'NO_MAP' in st['error']


def test_second_start_and_pending_draft_are_refused(tmp_path):
    s = make(tmp_path, mode='hang')
    s.start()
    running(s)
    with pytest.raises(MappingError) as e:
        s.start()
    assert e.value.code == 'ALREADY_MAPPING'
    s.stop()
    wait(s)
    with pytest.raises(MappingError) as e:
        s.start()
    assert e.value.code == 'DRAFT_PENDING'
    s.discard()
    assert not (tmp_path / 'map' / 'drafts' / 'current').exists()
    assert list((tmp_path / 'map' / 'drafts' / 'archive').iterdir())      # kept, not deleted
    assert s.status()['phase'] == 'idle'


def test_orders_active_blocks_start(tmp_path):
    s = make(tmp_path)
    with pytest.raises(MappingError) as e:
        s.start(busy=lambda: True)
    assert (e.value.status, e.value.code) == (409, 'ORDERS_ACTIVE')


def test_state_survives_backend_restart(tmp_path):
    s = make(tmp_path)
    s.start()
    wait(s)
    s2 = make(tmp_path)               # a new object reads state.json
    st = s2.status()
    assert st['draft_ready'] and st['phase'] == 'stopped'
    assert s2.review_service().describe()['source_hash']


def test_restart_while_runner_alive_still_follows_it(tmp_path):
    s = make(tmp_path, mode='hang')
    s.start()
    running(s)
    s2 = make(tmp_path)
    assert s2.status()['phase'] == 'mapping'
    s2.stop()
    assert wait(s2)['draft_ready']


def test_save_review_and_finish_adoption(tmp_path):
    s = make(tmp_path)
    s.start()
    wait(s)
    desc = s.review_service().describe()
    with pytest.raises(MappingError) as e:
        s.save_review({'source_hash': desc['source_hash'], 'confirmed': False, 'strokes': []})
    assert e.value.status == 422
    with pytest.raises(MappingError) as e:
        s.save_review({'source_hash': 'bad', 'confirmed': True, 'strokes': []})
    assert e.value.status == 409
    r = s.save_review({'source_hash': desc['source_hash'], 'confirmed': True, 'strokes': []})
    assert r['saved'] and os.path.isfile(os.path.join(r['output_dir'], 'navigation_map.pgm'))
    assert s.status()['last_saved_revision'] == 1
    s.finish_adoption()
    assert s.status()['phase'] == 'idle' and not (tmp_path / 'map' / 'drafts' / 'current').exists()
    s.start()                          # a new mapping can begin
    wait(s)
