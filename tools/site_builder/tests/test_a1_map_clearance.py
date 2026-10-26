#!/usr/bin/env python3
"""A-1: map reading, coordinates, clearance. Run: python3 tools/site_builder/tests/test_a1_map_clearance.py"""
import os
import struct
import sys
import tempfile
import zlib

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, '..'))
from site_builder.clearance import clearance_map, rasterize_rects   # noqa: E402
from site_builder.mapio import MapData, MapError, read_pgm, save_png  # noqa: E402

REPO = os.path.abspath(os.path.join(HERE, '..', '..', '..'))


def write_map(tmp, img, res=0.05, origin=(-1.0, -2.0), free_thresh=0.196, extra=''):
    h, w = img.shape
    with open(os.path.join(tmp, 'm.pgm'), 'wb') as f:
        f.write(b'P5\n# comment\n%d %d\n255\n' % (w, h) + img.astype(np.uint8).tobytes())
    with open(os.path.join(tmp, 'm.yaml'), 'w') as f:
        f.write('image: m.pgm\nmode: trinary\nresolution: %s\norigin: [%s, %s, 0.0]\n'
                'negate: 0\noccupied_thresh: 0.65\nfree_thresh: %s\n%s'
                % (res, origin[0], origin[1], free_thresh, extra))
    return os.path.join(tmp, 'm.yaml')


def corridor_image():
    img = np.full((30, 60), 254, np.uint8)
    img[0:5, :] = 0
    img[25:30, :] = 0
    return img


def test_thresholds():
    with tempfile.TemporaryDirectory() as tmp:
        img = np.array([[254, 205, 0]], np.uint8)
        m = MapData(write_map(tmp, img))
        assert m.free.tolist() == [[True, False, False]]
        assert m.unknown.tolist() == [[False, True, False]]
        assert m.occupied.tolist() == [[False, False, True]]
        # the nav2 default 0.25 would read 205 as free: the loader must follow the yaml
        m2 = MapData(write_map(tmp, img, free_thresh=0.25))
        assert m2.free.tolist() == [[True, True, False]]


def test_coordinates_round_trip_and_row_flip():
    with tempfile.TemporaryDirectory() as tmp:
        m = MapData(write_map(tmp, corridor_image(), origin=(-1.0, -2.0)))
        # top row of the image is the largest y
        x, y = m.xy_of(0, 0)
        assert abs(x - (-1.0 + 0.025)) < 1e-9 and abs(y - (-2.0 + 30 * 0.05 - 0.025)) < 1e-9
        x, y = m.xy_of(29, 59)
        assert abs(x - (-1.0 + 59.5 * 0.05)) < 1e-9 and abs(y - (-2.0 + 0.025)) < 1e-9
        for r, c in [(0, 0), (10, 20), (29, 59)]:
            assert m.cell_of(*m.xy_of(r, c)) == (r, c)
        assert not m.inside(30, 0) and not m.inside(0, -1)


def test_clearance_in_known_corridor():
    with tempfile.TemporaryDirectory() as tmp:
        m = MapData(write_map(tmp, corridor_image()))
        c = clearance_map(m.blocked, m.res, cap_m=1.0)
        # corridor = 20 rows (1.0 m); the two middle rows are 10 cells from a wall:
        # 10 * 0.05 - 0.025 = 0.475 m to the obstacle edge
        assert abs(c[14, 30] - 0.475) < 1e-6 and abs(c[15, 30] - 0.475) < 1e-6
        assert abs(c[5, 30] - 0.025) < 1e-6          # next to a wall: half a cell
        assert c[0, 30] == 0.0                       # blocked cell
        assert c.max() <= 1.0
        # the map border counts as blocked
        img = np.full((20, 20), 254, np.uint8)
        m2 = MapData(write_map(tmp, img))
        c2 = clearance_map(m2.blocked, m2.res, 1.0)
        assert abs(c2[0, 10] - 0.025) < 1e-6


def test_unknown_blocks_like_a_wall():
    with tempfile.TemporaryDirectory() as tmp:
        img = corridor_image()
        img[10:20, 28:32] = 205        # unknown patch in the middle of the corridor
        m = MapData(write_map(tmp, img))
        c = clearance_map(m.blocked, m.res, 1.0)
        assert c[14, 29] == 0.0
        assert c[14, 40] < 0.475


def test_rasterize_rects():
    with tempfile.TemporaryDirectory() as tmp:
        m = MapData(write_map(tmp, corridor_image(), origin=(0.0, 0.0)))
        mask = rasterize_rects((m.h, m.w), m, [(0.5, 1.0, 0.5, 1.0)])
        rows, cols = np.nonzero(mask)
        assert rows.size == 100 and cols.min() == 10 and cols.max() == 19


def test_unsupported_maps_are_refused():
    with tempfile.TemporaryDirectory() as tmp:
        p = write_map(tmp, corridor_image())
        text = open(p).read().replace('[-1.0, -2.0, 0.0]', '[-1.0, -2.0, 0.5]')
        open(p, 'w').write(text)
        try:
            MapData(p)
        except MapError:
            return
        raise AssertionError('origin yaw must be refused')


def test_png_writer():
    with tempfile.TemporaryDirectory() as tmp:
        p = os.path.join(tmp, 'a.png')
        rgb = np.zeros((4, 6, 3), np.uint8)
        rgb[1, 2] = (255, 0, 0)
        save_png(p, rgb)
        data = open(p, 'rb').read()
        assert data[:8] == b'\x89PNG\r\n\x1a\n'
        w, h = struct.unpack('>II', data[16:24])
        assert (w, h) == (6, 4)
        i = data.index(b'IDAT')
        n = struct.unpack('>I', data[i - 4:i])[0]
        raw = zlib.decompress(data[i + 4:i + 4 + n])
        assert len(raw) == 4 * (1 + 6 * 3) and raw[(1 + 18) * 1 + 1 + 6:(1 + 18) * 1 + 1 + 9] == b'\xff\x00\x00'


def test_demo_map_matches_what_the_vm_measured():
    p = os.path.join(REPO, 'maps', 'stores', 'demo_store', 'map.yaml')
    if not os.path.exists(p):
        return
    m = MapData(p)
    # VM /map counts (checkpoint 2-b): free 17804, occupied 1958, unknown 5438
    assert (int(m.free.sum()), int(m.occupied.sum()), int(m.unknown.sum())) == (17804, 1958, 5438)


if __name__ == '__main__':
    for name, fn in sorted(globals().items()):
        if name.startswith('test_') and callable(fn):
            fn()
            print('ok  ', name)
    print('all passed')
