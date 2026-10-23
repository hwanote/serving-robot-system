"""scripts/seal_map_edges.py: sealing leaks in a raw exploration map (no ROS).

    python3 tests/robot/test_seal_map.py
"""
import json
import os
import sys
import tempfile

import numpy as np

ROOT = os.path.join(os.path.dirname(__file__), '..', '..')
sys.path.insert(0, os.path.join(ROOT, 'scripts'))
import seal_map_edges as sm  # noqa: E402

F, O, U = sm.FREE, sm.OCC, sm.UNK


def _clipped_room():
    """6x8 room whose top wall row was cut off (free cells touch the grid edge)."""
    g = np.full((6, 8), F, np.uint8)
    g[:, 0] = g[:, -1] = g[-1, :] = O      # left, right, bottom walls; top wall missing
    g[2, 3:5] = U                          # an unseen pocket inside, touching free cells
    return g


def _write_raw(d, grid, mode='raw'):
    sm.write_pgm(os.path.join(d, 'raw.pgm'), grid)
    with open(os.path.join(d, 'raw.yaml'), 'w') as f:
        f.write('image: raw.pgm\nmode: %s\nresolution: 0.050\norigin: [-1.0, -2.0, 0]\n'
                'negate: 0\noccupied_thresh: 0.650\nfree_thresh: 0.250\n' % mode)
    return os.path.join(d, 'raw.yaml')


def test_seal_closes_every_leak_and_keeps_raw():
    g = _clipped_room()
    sealed, mask = sm.seal(g, pad=1)
    assert sm.leaks(g) > 0
    assert sm.leaks(sealed) == 0
    assert sealed.shape == (8, 10)
    inner = sealed[1:-1, 1:-1]
    assert (inner[g == F] == F).all()            # free cells stay free
    assert (inner[g == O] == O).all()            # walls stay walls
    assert mask.sum() == (sealed == O).sum() - (g == O).sum()   # mask = exactly the added walls
    assert mask[0, 2:-2].all()                   # the missing top wall appears in the padding


def test_unknown_not_next_to_free_is_left_alone():
    g = np.full((7, 7), O, np.uint8)
    g[3, 3] = U                                  # enclosed unknown, no free neighbour
    g[1, 1] = F
    sealed, mask = sm.seal(g, pad=1)
    assert sealed[3 + 1, 3 + 1] == U
    assert not mask[3 + 1, 3 + 1]


def test_cli_seal_then_review_marks_hand_edits():
    with tempfile.TemporaryDirectory() as d:
        y = _write_raw(d, _clipped_room())
        before = open(os.path.join(d, 'raw.pgm'), 'rb').read()
        assert sm.main(['seal', y]) == 0
        assert open(os.path.join(d, 'raw.pgm'), 'rb').read() == before   # raw untouched
        meta = open(os.path.join(d, 'raw_corrected.yaml')).read()
        assert 'raw_corrected.pgm' in meta
        rep = json.load(open(os.path.join(d, 'raw_autofix_report.json')))
        assert rep['leaks_after'] == 0 and rep['cells_added'] > 0
        assert abs(rep['origin_after'][0] - (-1.05)) < 1e-9 and abs(rep['origin_after'][1] - (-2.05)) < 1e-9
        # hand edit: erase one auto wall cell and one raw cell, then redraw the review
        cp = os.path.join(d, 'raw_corrected.pgm')
        c = sm.read_pgm(cp)
        assert set(np.unique(c).tolist()) <= {sm.TRI_OCC, sm.TRI_FREE, sm.TRI_UNK}   # trinary look
        mask = sm.read_pgm(os.path.join(d, 'raw_autofix_mask.pgm')) > 0
        r, col = np.argwhere(mask)[0]
        c[r, col] = 250                      # hand edit with an off-white shade: still "free"
        c[4, 4] = 190                        # a mid grey: unknown
        sm.write_pgm(cp, c)
        assert sm.main(['review', os.path.join(d, 'raw_corrected.yaml')]) == 0
        png = open(os.path.join(d, 'raw_review.png'), 'rb').read()
        assert png[:8] == b'\x89PNG\r\n\x1a\n'
        _, now = sm.load_map(os.path.join(d, 'raw_corrected.yaml'))
        assert now[r, col] == F and now[4, 4] == U
        assert 'mode: trinary' in meta


def test_shift_origin_moves_only_the_origin():
    with tempfile.TemporaryDirectory() as d:
        y = _write_raw(d, _clipped_room())
        assert sm.main(['seal', y]) == 0
        plain = sm.read_pgm(os.path.join(d, 'raw_corrected.pgm')).copy()
        assert sm.main(['seal', y, '--shift-origin', '-3.2', '-2.4']) == 0
        assert (sm.read_pgm(os.path.join(d, 'raw_corrected.pgm')) == plain).all()   # pixels same
        rep = json.load(open(os.path.join(d, 'raw_autofix_report.json')))
        assert abs(rep['origin_after'][0] - (-1.05 - 3.2)) < 1e-9
        assert abs(rep['origin_after'][1] - (-2.05 - 2.4)) < 1e-9
        assert rep['origin_before'][:2] == [-1.0, -2.0] and rep['shift_origin_m'] == [-3.2, -2.4]
        # segment coordinates are reported in the shifted (output) frame
        assert all(seg['x_m'][0] < -4.0 for seg in rep['segments'])


def test_review_colors():
    base = np.array([[F, O], [O, U]], np.uint8)
    mask = np.array([[False, True], [False, False]])
    cur = base.copy()
    cur[1, 0] = F                                 # hand change
    img = sm.review_image(cur, base, mask)
    s = sm.SCALE
    assert tuple(img[0, s]) == sm.COLORS['auto']
    assert tuple(img[s, 0]) == sm.COLORS['manual']
    assert tuple(img[0, 0]) == sm.COLORS['free']
    assert tuple(img[s, s]) == sm.COLORS['unk']


def test_refuses_unsupported_mode():
    with tempfile.TemporaryDirectory() as d:
        y = _write_raw(d, np.zeros((4, 4), np.uint8), mode='scale')
        assert sm.main(['seal', y]) == 1


def test_trinary_roundtrip_and_thresholds():
    g = np.array([[F, O, U]], np.uint8)
    assert (sm.decode_trinary(sm.encode_trinary(g)) == g).all()
    assert sm.encode_trinary(g).tolist() == [[254, 0, 205]]
    # shades from an image editor: dark = wall, bright = free, 205 grey = unknown
    assert sm.decode_trinary(np.array([[10, 60, 120, 205, 240, 255]], np.uint8)).tolist() == \
        [[O, O, U, U, F, F]]


def test_trinary_input_is_accepted():
    with tempfile.TemporaryDirectory() as d:
        sm.write_pgm(os.path.join(d, 'raw.pgm'), sm.encode_trinary(_clipped_room()))
        with open(os.path.join(d, 'raw.yaml'), 'w') as f:
            f.write('image: raw.pgm\nresolution: 0.05\norigin: [0, 0, 0]\nnegate: 0\n'
                    'occupied_thresh: 0.65\nfree_thresh: 0.196\n')
        assert sm.main(['seal', os.path.join(d, 'raw.yaml')]) == 0
        _, g = sm.load_map(os.path.join(d, 'raw_corrected.yaml'))
        assert sm.leaks(g) == 0


def test_real_map_if_present():
    p = os.path.join(ROOT, '_mapping_test_results', 'R16-147_official_raw_map.yaml')
    if not os.path.exists(p):
        return
    meta, g = sm.load_map(p)
    sealed, _ = sm.seal(g)
    assert sm.leaks(sealed) == 0


if __name__ == '__main__':
    for name, fn in sorted(globals().items()):
        if name.startswith('test_') and callable(fn):
            fn()
            print('ok', name)
