"""Camera observation -> person state, after splitting person_detector (no ROS).

    python3 tests/robot/test_person_judge.py

The main test replays scenarios through the OLD one-node logic (copied below
from person_detector.py before the split) and through the new pair
make_observation (camera node) -> PersonJudge (judge node), and checks that the
person_detection message is the same, field by field, every tick.
"""
import json
import math
import os
import sys

import numpy as np

sys.path.insert(0, os.path.join(
    os.path.dirname(__file__), '..', '..', 'ros_ws', 'src',
    'serving_robot_navigation'))
from serving_robot_navigation.decision.person_judge import PersonJudge  # noqa: E402
from serving_robot_navigation.decision.person_safety import (  # noqa: E402
    NORMAL, SLOW, STALE, STOP, PersonSafetyCore)
from serving_robot_navigation.monitoring.person_observation import (  # noqa: E402
    box_distance, fx_from_fov, make_observation, measured, parse_observation,
    pixel_angle, true_range)

BOX = [100.0, 40.0, 160.0, 230.0]
BOX2 = [200.0, 60.0, 240.0, 200.0]


def core():
    return PersonSafetyCore(stop_m=0.8, slow_m=1.5, hysteresis_m=0.2,
                            release_frames=3, median_n=3, stale_s=2.0,
                            lost_hold_s=3.0, clear_m=1.0, clear_hold_s=2.0)


class OldDetector:
    """person_detector._tick before the split (YOLO replaced by `found`)."""

    def __init__(self):
        self.core = core()
        self.last = {'confidence': 0.0, 'count': 0, 'bbox': None}

    def tick(self, now, new_frame, found, hold, clear):
        if new_frame:
            valid = [f for f in found if f[0] is not None]
            self.core.update(now, [f[0] for f in valid], hold, clear)
            best = min(valid, key=lambda f: f[0]) if valid else None
            self.last = {'confidence': round(best[1], 2) if best else 0.0,
                         'count': len(found),
                         'bbox': [round(v) for v in best[2]] if best else None}
        else:
            self.core.check_stale(now)
        self.core.check_stale(now)
        c = self.core
        return {'state': c.state, 'reason': c.reason,
                'obstacle_hold': hold, 'source': c.source,
                'front_clear_m': (None if clear in (None, float('inf'))
                                  else round(clear, 2)),
                'distance_m': None if c.stable is None else round(c.stable, 2),
                'nearest_m': None if c.raw is None else round(c.raw, 2),
                'confidence': self.last['confidence'], 'count': self.last['count'],
                'bbox': self.last['bbox'],
                'frame_age_s': None if c.last_frame is None
                else round(now - c.last_frame, 2),
                'camera': 'camera_front', 'time': round(now, 1)}


def scenario():
    """(new_frame, found, obstacle_hold, front_clear_m) for every 0.25 s tick."""
    inf = float('inf')
    s = []
    s += [(False, [], False, None)] * 4                       # no frame yet: STALE
    s += [(True, [], False, inf)] * 4                         # nobody
    for d in (3.0, 2.4, 1.8, 1.4, 1.2, 0.9, 0.7, 0.6):        # a person walks up
        s.append((True, [(d, 0.83, BOX), (None, 0.5, BOX)], False, inf))
    s += [(True, [], False, 0.55)] * 4                        # lost while close, LiDAR sees
    s += [(True, [], True, None)] * 4                         # obstacle hold, scan unknown
    s += [(True, [], True, inf)] * 20                         # way clear, hold alone keeps it
    s += [(True, [], False, 2.5)] * 14                        # hold off, front clear -> released
    s += [(True, [(1.4, 0.55, BOX), (2.6, 0.95, BOX2)], False, inf)] * 3   # two measured
    s += [(True, [(1.3, 0.77, BOX)], False, inf)] * 3         # SLOW
    s += [(True, [(2.6, 0.9, BOX)], False, inf)] * 5          # away again
    s += [(True, [], False, 0.7)] * 3                         # LiDAR alone: STOP
    s += [(False, [], False, 0.7)] * 12                       # camera frames stop: STALE
    s += [(True, [(1.0, 0.6, BOX)], False, inf)] * 3          # back
    return s


def test_split_gives_the_same_message_as_the_old_node_every_tick():
    old, new = OldDetector(), PersonJudge(core())
    seen = set()
    for i, (frame, found, hold, clear) in enumerate(scenario()):
        now = 10.0 + 0.25 * i
        expect = old.tick(now, frame, found, hold, clear)
        obs = parse_observation(json.dumps(          # through the wire as JSON
            make_observation('camera_front', now, frame, found)))
        got = new.on_observation(now, obs, hold, clear)
        assert {k: got[k] for k in expect} == expect, (i, got, expect)
        assert got['side_m'] is None and got['rear_m'] is None   # one camera
        # the camera is listed silent only while its frames are missing (STALE)
        assert got['silent_cameras'] in ([], ['camera_front'])
        if got['silent_cameras']:
            assert got['state'] == STALE and got['frame_age_s'] > 2.0
            seen.add('silent')
        seen.add(got['state'])
    assert seen == {STALE, NORMAL, SLOW, STOP, 'silent'}, seen  # all covered


def test_observation_json_round_trip():
    obs = make_observation('camera_left', 3.14159, True,
                           [(1.234, 0.9, BOX), (None, 0.4, [1, 2, 3, 4])])
    back = parse_observation(json.dumps(obs))
    assert back['camera'] == 'camera_left' and back['time'] == 3.142
    assert back['new_frame'] is True and back['count'] == 2
    assert [p['distance_m'] for p in measured(back)] == [1.234]


def test_bad_observations_are_refused_or_filled():
    assert parse_observation('not json') is None
    assert parse_observation('[1, 2]') is None
    assert parse_observation('{"people": 5}') is None
    obs = parse_observation('{"camera": "c"}')
    assert obs['people'] == [] and obs['new_frame'] is False and obs['count'] == 0
    junk = {'people': [{'distance_m': 'far'}, 'x', {'distance_m': None}]}
    assert measured(junk) == []


def test_heartbeat_without_new_frames_goes_stale():
    j = PersonJudge(core())
    body = j.on_observation(1.0, make_observation('c', 1.0, True, [(1.2, 0.8, BOX)]),
                            False, float('inf'))
    assert body['state'] == SLOW and body['count'] == 1
    t = 1.0
    while t < 3.2:                                     # camera alive, no images
        t += 0.25
        body = j.on_observation(t, make_observation('c', t, False), False, None)
    assert body['state'] == STALE and body['count'] == 1   # last detection kept
    assert body['frame_age_s'] > 2.0


def test_box_distance_uses_the_middle_of_the_box():
    depth = np.full((240, 320), 5.0, dtype=np.float32)
    depth[90:180, 115:146] = 1.5                      # the person, middle of BOX
    depth[40:60, :] = np.nan                          # invalid pixels are skipped
    assert math.isclose(box_distance(depth, BOX, 0.1, 8.0), 1.5)
    assert box_distance(np.full((240, 320), np.nan, np.float32), BOX, 0.1, 8.0) is None
    assert box_distance(np.full((240, 320), 9.0, np.float32), BOX, 0.1, 8.0) is None



# ----- step 4: several cameras -------------------------------------------------
FRONT, LEFT, RIGHT = 'camera_front', 'camera_rear_left', 'camera_rear_right'
CAMS = [FRONT, LEFT, RIGHT]
DEG = math.pi / 180.0


def judge3():
    return PersonJudge(core(), primary_camera=FRONT, cameras=CAMS,
                       front_half_deg=60.0, side_max_deg=120.0, side_slow_m=0.8,
                       side_release_s=1.0, camera_stale_s=2.0)


def feed(j, t, front=(), left=(), right=(), clear=float('inf'), hold=False):
    """One tick: both rear cameras, then the front one (as the camera node does).
    People: (distance, bearing_deg)."""
    for cam, people in ((LEFT, left), (RIGHT, right)):
        if people is not None:
            j.on_observation(t, make_observation(cam, t, True, [
                (d, 0.8, BOX, b * DEG) for d, b in people]), hold, clear)
    return j.on_observation(t, make_observation(FRONT, t, True, [
        (d, 0.8, BOX, b * DEG) for d, b in front]), hold, clear)


def run(j, seconds, t0, **kw):
    t, body = t0, None
    while t < t0 + seconds - 1e-9:
        t += 0.25
        body = feed(j, t, **kw)
    return t, body


def test_pixel_geometry():
    fx = fx_from_fov(320, 120 * DEG)
    assert math.isclose(pixel_angle(160, 160, fx), 0.0)
    assert math.isclose(pixel_angle(0, 160, fx), 60 * DEG, rel_tol=1e-6)     # left edge
    assert math.isclose(pixel_angle(320, 160, fx), -60 * DEG, rel_tol=1e-6)  # right edge
    assert math.isclose(true_range(1.0, 60 * DEG), 2.0, rel_tol=1e-6)        # edge: x2
    assert math.isclose(true_range(1.5, 0.0), 1.5)


def test_only_the_primary_camera_publishes():
    j = judge3()
    assert j.on_observation(1.0, make_observation(LEFT, 1.0, True), False, None) is None
    assert j.on_observation(1.0, make_observation(FRONT, 1.0, True), False, None) is not None


def test_person_beside_raises_slow_never_stop_then_releases():
    j = judge3()
    t, body = run(j, 2.0, 0.0)
    assert body['state'] == NORMAL
    t, body = run(j, 1.0, t, left=[(0.5, 90)])            # very close, beside
    assert body['state'] == SLOW and body['source'] == 'camera'
    assert 'beside' in body['reason'] and body['side_camera'] == LEFT
    assert body['side_m'] == 0.5
    t, body = run(j, 0.5, t, left=[(0.9, 90)])            # inside the hysteresis band
    assert body['state'] == SLOW
    t, body = run(j, 0.75, t, left=[(1.2, 90)])           # clear, but < side_release_s
    assert body['state'] == SLOW
    t, body = run(j, 0.5, t, left=[(1.2, 90)])
    assert body['state'] == NORMAL and body['side_m'] is None


def test_person_behind_does_not_change_the_speed():
    j = judge3()
    t, body = run(j, 3.0, 0.0, right=[(0.4, -170)], left=[(0.6, 150)])
    assert body['state'] == NORMAL and body['rear_m'] == 0.4


def test_side_does_not_weaken_a_front_stop():
    j = judge3()
    t, body = run(j, 2.0, 0.0, front=[(0.6, 5)], left=[(0.5, 100)])
    assert body['state'] == STOP and 'beside' not in body['reason']


def test_front_camera_edge_person_counts_as_ahead():
    j = judge3()
    t, body = run(j, 1.0, 0.0, front=[(1.2, 55)])
    assert body['state'] == SLOW and body['source'] == 'camera'


def test_a_rear_camera_that_stops_is_listed_and_only_alarms():
    j = judge3()
    t, body = run(j, 2.0, 0.0)
    assert body['silent_cameras'] == []
    t, body = run(j, 2.5, t, left=None)                    # left camera: no frames
    assert body['silent_cameras'] == [LEFT] and body['state'] == NORMAL
    t, body = run(j, 0.5, t)                               # back
    assert body['silent_cameras'] == []


def test_a_rear_camera_that_never_spoke_is_listed():
    j = judge3()
    t, body = run(j, 3.0, 0.0, right=None)
    assert body['silent_cameras'] == [RIGHT]


def test_old_side_info_is_forgotten_when_that_camera_goes_silent():
    j = judge3()
    t, body = run(j, 1.0, 0.0, left=[(0.5, 90)])
    assert body['state'] == SLOW
    t, body = run(j, 3.5, t, left=None)                    # camera stops: stale data
    assert body['state'] == NORMAL and LEFT in body['silent_cameras']



def test_a_wide_front_camera_person_beside_is_only_slow():
    """A front camera wider than the ahead sector: a person it sees beside the
    robot follows the beside rule (SLOW), not STOP."""
    j = judge3()
    t, body = run(j, 2.0, 0.0, front=[(0.5, 75)])
    assert body['state'] == SLOW and 'beside' in body['reason']
    assert body['distance_m'] is None          # not an ahead candidate



def test_a_camera_node_restart_does_not_list_every_camera():
    """Camera node killed and started again: only DETECTOR_SILENT/RECOVERED,
    no burst of CAMERA_SILENT; a camera that really stays dark is listed later."""
    j = judge3()
    t, body = run(j, 2.0, 0.0)
    assert body['silent_cameras'] == []
    t += 10.0                                          # node away (model reloads)
    body = j.on_observation(t, make_observation(FRONT, t, False), False, None)
    assert body['silent_cameras'] == [], body['silent_cameras']
    assert body['state'] == STALE                      # no frame: the rule stays
    t, body = run(j, 1.5, t, right=None)               # frames again, right dark
    assert body['silent_cameras'] == [] and body['state'] == NORMAL
    t, body = run(j, 1.0, t, right=None)               # > camera_stale_s since restart
    assert body['silent_cameras'] == [RIGHT]


if __name__ == '__main__':
    for name, fn in sorted(globals().items()):
        if name.startswith('test_') and callable(fn):
            fn()
            print('ok  ', name)
    print('all passed')
