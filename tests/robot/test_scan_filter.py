"""Tests for the known-obstacle laser filter (no ROS needed, numpy only).

    python3 tests/robot/test_scan_filter.py

What must hold: points on the map's fixed objects (walls, counter, table legs)
are removed, points on anything the map does not know (a person, a chair) are
kept, and values that are not real returns are left alone. The last test runs
on the real demo map with the tolerance from the node's yaml, so it also
checks a map swap (e.g. the team's real map) in one command.
"""
import math
import os
import sys

import numpy as np
import yaml

ROOT = os.path.join(os.path.dirname(__file__), '..', '..')
sys.path.insert(0, os.path.join(ROOT, 'ros_ws', 'src', 'serving_robot_navigation'))
from serving_robot_navigation.monitoring.scan_filter import (  # noqa: E402
    StaticMask, dilate, filter_ranges)

RES = 0.05


def grid(w, h, occupied=(), fill=0):
    """OccupancyGrid-like data (row 0 = bottom). `occupied`: (ix, iy) cells."""
    g = np.full((h, w), fill, dtype=np.int16)
    for ix, iy in occupied:
        g[iy, ix] = 100
    return g.ravel().tolist()


def wall_mask(tolerance_m=0.10):
    """10 m x 6 m room, origin (-5, -3). One wall along x = 2.0 m (east)."""
    w, h = 200, 120
    cells = [(int((2.0 + 5.0) / RES), iy) for iy in range(h)]
    return StaticMask(grid(w, h, cells), w, h, RES, -5.0, -3.0, tolerance_m)


def beam(r, angle):
    """One-beam scan: ranges, angle_min, angle_increment."""
    return [r], angle, 0.0


# ----- dilate -------------------------------------------------------------------
def test_dilate_radius_0_is_a_copy():
    a = np.zeros((5, 5), dtype=bool)
    a[2, 2] = True
    out = dilate(a, 0)
    assert (out == a).all() and out is not a


def test_dilate_radius_1_is_a_cross_radius_2_reaches_diagonals():
    a = np.zeros((7, 7), dtype=bool)
    a[3, 3] = True
    one = dilate(a, 1)
    assert one.sum() == 5 and one[2, 3] and one[3, 4] and not one[2, 2]
    two = dilate(a, 2)
    assert two[2, 2] and two[1, 3] and not two[1, 1]


def test_dilate_does_not_wrap_around_the_edges():
    a = np.zeros((5, 5), dtype=bool)
    a[0, 0] = True
    out = dilate(a, 1)
    assert out[0, 0] and out[0, 1] and out[1, 0]
    assert not out[4, 0] and not out[0, 4] and not out[4, 4]


# ----- StaticMask ------------------------------------------------------------------
def test_threshold_and_unknown_cells():
    data = [-1, 64, 65, 100]                       # one row of 4 cells
    m = StaticMask(data, 4, 1, 1.0, 0.0, 0.0, tolerance_m=0.0,
                   occupied_threshold=65)
    xs = [0.5, 1.5, 2.5, 3.5]
    assert m.is_static(xs, [0.5] * 4).tolist() == [False, False, True, True]


def test_tolerance_grows_by_whole_cells():
    m = wall_mask(0.10)                            # 0.10 m = 2 cells of 0.05 m
    # wall cell spans x 2.00..2.05; 2 cells either side are "known"
    assert m.is_static([1.91, 2.02, 2.14], [0.0] * 3).tolist() == [True] * 3
    assert m.is_static([1.89, 2.16], [0.0] * 2).tolist() == [False, False]


def test_negative_origin_and_points_outside_the_map():
    m = wall_mask()
    assert m.is_static([2.02], [-2.99]).tolist() == [True]    # first row
    assert m.is_static([2.02], [-3.01]).tolist() == [False]   # below the map
    assert m.is_static([-5.5, 5.5], [0.0, 0.0]).tolist() == [False, False]


# ----- filter_ranges -----------------------------------------------------------------
def test_wall_point_removed_person_point_kept():
    m = wall_mask()
    # sensor at origin facing east: beam 0 hits the wall at 2.0 m,
    # beam 1 (pointing north) hits a person at 1.0 m in open floor
    out, removed = filter_ranges([2.02, 1.0], 0.0, math.pi / 2, 0.1, 10.0,
                                 (0.0, 0.0, 0.0), m)
    assert math.isinf(out[0]) and out[1] == 1.0 and removed == 1


def test_person_standing_right_in_front_of_the_wall_is_kept():
    m = wall_mask()
    # 0.3 m in front of the wall: beyond the 0.10 m tolerance
    out, removed = filter_ranges(*beam(1.7, 0.0), 0.1, 10.0, (0.0, 0.0, 0.0), m)
    assert out[0] == 1.7 and removed == 0


def test_non_returns_are_left_alone():
    m = wall_mask()
    ranges = [float('nan'), float('inf'), 0.05, 12.0]   # < range_min, > range_max
    out, removed = filter_ranges(ranges, 0.0, 0.0, 0.1, 10.0, (0.0, 0.0, 0.0), m)
    assert removed == 0
    assert math.isnan(out[0]) and math.isinf(out[1])
    assert out[2] == 0.05 and out[3] == 12.0
    # beyond range_max but landing ON the wall: still not touched (not a return)
    out, removed = filter_ranges(*beam(2.02, 0.0), 0.1, 2.0, (0.0, 0.0, 0.0), m)
    assert removed == 0 and out[0] == 2.02
    # closer than range_min but on the wall (sensor 0.05 m from it): not touched
    out, removed = filter_ranges(*beam(0.05, 0.0), 0.1, 10.0, (1.97, 0.0, 0.0), m)
    assert removed == 0 and out[0] == 0.05


def test_sensor_heading_is_used():
    m = wall_mask()
    # sensor at (2.0, -1.0) turned to face north (+90 deg): a beam straight
    # ahead runs along the wall line and is removed; the same beam from a
    # sensor that faces east would land in open floor and is kept
    north = filter_ranges(*beam(1.0, 0.0), 0.1, 10.0, (2.02, -1.0, math.pi / 2), m)
    east = filter_ranges(*beam(1.0, 0.0), 0.1, 10.0, (2.02 - 3.0, -1.0, 0.0), m)
    assert north[1] == 1 and math.isinf(north[0][0])
    assert east[1] == 0


def test_localization_error_beyond_the_tolerance_keeps_wall_points():
    """The limit written in dynamic_scan_filter.yaml: if the estimated pose is
    off by more than tolerance_m, wall points look unknown again (the robot
    then slows down near walls: safe side, but it shows a localization issue)."""
    m = wall_mask(0.10)
    true_pose = filter_ranges(*beam(2.02, 0.0), 0.1, 10.0, (0.0, 0.0, 0.0), m)
    off_pose = filter_ranges(*beam(2.02, 0.0), 0.1, 10.0, (-0.3, 0.0, 0.0), m)
    assert true_pose[1] == 1 and off_pose[1] == 0


# ----- the real demo map ----------------------------------------------------------------
def load_map(map_yaml):
    """Read map.yaml + pgm like the map server (trinary): row 0 = bottom."""
    with open(map_yaml) as f:
        meta = yaml.safe_load(f)
    with open(os.path.join(os.path.dirname(map_yaml), meta['image']), 'rb') as f:
        raw = f.read()
    magic, size, _maxval, pixels = raw.split(b'\n', 3)
    assert magic == b'P5'
    w, h = map(int, size.split())
    img = np.frombuffer(pixels[:w * h], dtype=np.uint8).reshape(h, w)
    occ = (255.0 - img) / 255.0 if not meta.get('negate', 0) else img / 255.0
    g = np.full((h, w), -1, dtype=np.int16)
    g[occ > meta['occupied_thresh']] = 100
    g[occ < meta['free_thresh']] = 0
    g = g[::-1]                                     # image top row = map top
    return g.ravel().tolist(), w, h, meta['resolution'], meta['origin']


def node_params():
    with open(os.path.join(ROOT, 'ros_ws', 'src', 'serving_robot_navigation',
                           'config', 'dynamic_scan_filter.yaml')) as f:
        p = yaml.safe_load(f)['dynamic_scan_filter']['ros__parameters']
    return float(p['tolerance_m']), int(p['occupied_threshold'])


def test_unknown_pixels_of_the_map_load_as_unknown():
    """map.yaml thresholds must keep the three pixel values apart: 254 free,
    0 occupied, 205 unknown (-1). With free_thresh 0.25 the unknown 205
    (0.196) was read as free. Check this again for every new map."""
    path = os.path.join(ROOT, 'maps', 'stores', 'demo_store', 'map.yaml')
    data, w, h, _res, _origin = load_map(path)
    g = np.asarray(data)
    assert (g == -1).sum() > 0, 'no unknown cells: 205 is read as free'
    with open(path) as f:
        meta = yaml.safe_load(f)
    occ = lambda px: (255.0 - px) / 255.0
    assert occ(254) < meta['free_thresh'] < occ(205) < meta['occupied_thresh'] < occ(0)


def test_real_demo_map_walls_removed_aisle_kept():
    data, w, h, res, origin = load_map(
        os.path.join(ROOT, 'maps', 'stores', 'demo_store', 'map.yaml'))
    tol, thr = node_params()
    m = StaticMask(data, w, h, res, origin[0], origin[1], tol, thr)
    sensor = (-3.20, -2.40, math.radians(90.0))     # W_S, facing north
    g = np.asarray(data).reshape(h, w)
    iy, ix = np.nonzero(g >= thr)
    assert ix.size > 100, 'demo map has no walls?'
    # aim one beam at the centre of every occupied cell: all must go
    xs = origin[0] + (ix + 0.5) * res
    ys = origin[1] + (iy + 0.5) * res
    dx, dy = xs - sensor[0], ys - sensor[1]
    for r, a in zip(np.hypot(dx, dy), np.arctan2(dy, dx) - sensor[2]):
        out, removed = filter_ranges([r], a, 0.0, 0.05, 30.0, sensor, m)
        assert removed == 1, 'wall point at r=%.2f a=%.2f kept' % (r, a)
    # a person in the west aisle, 1.0 m ahead (x -3.2, y -1.4): kept
    out, removed = filter_ranges(*beam(1.0, 0.0), 0.05, 30.0, sensor, m)
    assert removed == 0 and out[0] == 1.0


if __name__ == '__main__':
    for name, fn in sorted(globals().items()):
        if name.startswith('test_') and callable(fn):
            fn()
            print('ok  ', name)
    print('all passed')
