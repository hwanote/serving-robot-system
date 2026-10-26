"""R16-176 conservative map wall proposal tests."""

from serving_robot_mapping.map_wall_suggestions import (
    find_wall_gaps,
)


def fixture():
    w = h = 14
    pixels = [205] * (w * h)

    for x in (1, 2, 3, 7, 8, 9):
        pixels[6 * w + x] = 100

    return pixels, w, h


def test_short_unknown_wall_gap_proposed():
    pixels, w, h = fixture()
    found = find_wall_gaps(
        pixels, w, h, 0.05
    )
    assert any(
        p["x1"] == 3 and p["x2"] == 7
        and p["y1"] == 6 and p["y2"] == 6
        for p in found
    )


def test_known_free_space_never_crossed():
    pixels, w, h = fixture()
    pixels[6 * w + 5] = 0
    found = find_wall_gaps(
        pixels, w, h, 0.05
    )
    assert not found


def test_door_like_free_space_rejected():
    pixels, w, h = fixture()
    for x in (4, 5, 6):
        pixels[5 * w + x] = 0
        pixels[7 * w + x] = 0

    assert not find_wall_gaps(
        pixels, w, h, 0.05
    )


def test_unanchored_ends_rejected():
    pixels, w, h = fixture()
    pixels[6 * w + 1] = 205
    pixels[6 * w + 9] = 205

    assert not find_wall_gaps(
        pixels, w, h, 0.05
    )


def test_gap_limit_prevents_large_connection():
    pixels, w, h = fixture()
    assert not find_wall_gaps(
        pixels, w, h, 0.05, max_gap_m=0.10
    )


def test_source_pixels_unchanged():
    pixels, w, h = fixture()
    original = list(pixels)

    find_wall_gaps(pixels, w, h, 0.05)

    assert pixels == original
