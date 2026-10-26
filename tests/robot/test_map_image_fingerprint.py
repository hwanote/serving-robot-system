"""Which map picture a robot loaded: fingerprint of map.yaml + its image, reported in mission_status."""
import os
import shutil
import sys

ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..'))
NAV = os.path.join(ROOT, 'ros_ws', 'src', 'serving_robot_navigation')
sys.path.insert(0, NAV)

from serving_robot_navigation.command.mission import MissionCore  # noqa: E402
from serving_robot_navigation.site_config import load_site, map_image_fingerprint  # noqa: E402

STORE = os.path.join(ROOT, 'maps', 'stores', 'demo_store')


def copy_store(tmp_path, name='demo_store'):
    dst = tmp_path / name
    shutil.copytree(STORE, dst)
    return str(dst / 'map.yaml')


def test_same_files_same_fingerprint_and_it_is_short_hex(tmp_path):
    a = map_image_fingerprint(os.path.join(STORE, 'map.yaml'))
    assert a == map_image_fingerprint(copy_store(tmp_path))
    assert len(a) == 12 and int(a, 16) >= 0


def test_line_endings_of_the_yaml_do_not_matter(tmp_path):
    y = copy_store(tmp_path)
    before = map_image_fingerprint(y)
    with open(y, 'rb') as f:
        raw = f.read().replace(b'\r\n', b'\n')
    with open(y, 'wb') as f:
        f.write(raw.replace(b'\n', b'\r\n'))
    assert map_image_fingerprint(y) == before


def test_changed_picture_or_origin_changes_it(tmp_path):
    y = copy_store(tmp_path)
    before = map_image_fingerprint(y)
    pgm = os.path.join(os.path.dirname(y), 'map.pgm')
    data = bytearray(open(pgm, 'rb').read())
    data[-1] ^= 0x01
    open(pgm, 'wb').write(bytes(data))
    assert map_image_fingerprint(y) != before
    y2 = copy_store(tmp_path, 'other')
    text = open(y2, encoding='utf-8').read().replace('-4.5', '-3.0')
    open(y2, 'w', encoding='utf-8').write(text)
    assert map_image_fingerprint(y2) != before


def test_unreadable_gives_empty_string(tmp_path):
    assert map_image_fingerprint(str(tmp_path / 'nope.yaml')) == ''
    bad = tmp_path / 'map.yaml'
    bad.write_text('image: missing.pgm\n')
    assert map_image_fingerprint(str(bad)) == ''


def test_mission_status_reports_map_image():
    core = MissionCore(load_site())
    assert core.status()['map_image'] == ''
    core.map_image = 'abc123abc123'
    assert core.status()['map_image'] == 'abc123abc123'
