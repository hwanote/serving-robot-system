"""Tests of the interface names (no ROS).

    python3 tests/robot/test_interfaces.py
"""
import os
import shutil
import sys
import tempfile

ROOT = os.path.join(os.path.dirname(__file__), '..', '..')
sys.path.insert(0, os.path.join(ROOT, 'ros_ws', 'src', 'serving_robot_navigation'))
sys.path.insert(0, os.path.join(ROOT, 'scripts'))
from serving_robot_navigation.interfaces import (  # noqa: E402
    InterfaceError, names_for)
import validate_interfaces  # noqa: E402

CONFIG = os.path.join(ROOT, 'config', 'common')


def test_default_robot_names():
    n = names_for('', CONFIG)
    assert n.robot == 'robot_01'
    assert n.topic('person_limit') == '/robot_01/person_limit'
    assert n.topic('cmd_vel') == '/robot_01/cmd_vel'
    assert n.topic('camera_image') == '/robot_01/camera_front/image'
    assert n.service('request_detour') == '/robot_01/request_detour'
    assert n.nav2('nav_action') == 'navigate_to_pose'
    assert n.nav2('collision_monitor_state') == 'collision_monitor_state'


def test_other_robot_changes_every_name():
    n = names_for('robot_02', CONFIG)
    assert n.robot == 'robot_02'
    assert all(t.startswith('/robot_02/') for t in n.all_topics().values())
    assert all(s.startswith('/robot_02/') for s in n.all_services().values())
    assert names_for(None, CONFIG).robot == 'robot_01'


def test_nav2_names_have_no_robot_prefix():
    a, b = names_for('robot_01', CONFIG), names_for('robot_02', CONFIG)
    assert a.nav2('nav_action') == b.nav2('nav_action')


def test_unknown_key_and_bad_robot_name():
    n = names_for('', CONFIG)
    for call in (lambda: n.topic('nope'), lambda: n.service('nope'),
                 lambda: n.nav2('nope'), lambda: names_for('Robot 1', CONFIG),
                 lambda: names_for('', os.path.join(CONFIG, 'missing'))):
        try:
            call()
        except InterfaceError:
            continue
        raise AssertionError('expected InterfaceError')


def test_names_are_unique():
    n = names_for('', CONFIG)
    topics = list(n.all_topics().values())
    assert len(topics) == len(set(topics))
    services = list(n.all_services().values())
    assert len(services) == len(set(services))


def test_static_files_agree_with_interfaces_yaml():
    assert validate_interfaces.check(CONFIG) == []


def _copy_tree(dst):
    for rel in ('ros_ws/src/serving_robot_gazebo/config',
                'ros_ws/src/serving_robot_navigation/config', 'docs'):
        shutil.copytree(os.path.join(ROOT, rel), os.path.join(dst, rel))


def test_validator_finds_a_missing_bridge_entry():
    with tempfile.TemporaryDirectory() as tmp:
        _copy_tree(tmp)
        path = os.path.join(tmp, 'ros_ws/src/serving_robot_gazebo/config/bridge.yaml')
        text = open(path, encoding='utf-8').read().replace(
            '/robot_01/contact', '/robot_01/contact_old')
        open(path, 'w', encoding='utf-8').write(text)
        problems = validate_interfaces.check(CONFIG, tmp)
        assert any('contact_old' in p for p in problems), problems
        assert any('/robot_01/contact' in p and 'not bridged' in p
                   for p in problems), problems


def test_validator_finds_a_wrong_name_in_nav2_and_docs():
    with tempfile.TemporaryDirectory() as tmp:
        _copy_tree(tmp)
        nav = os.path.join(tmp, 'ros_ws/src/serving_robot_navigation/config/nav2.yaml')
        text = open(nav, encoding='utf-8').read().replace(
            '/robot_01/odom', '/robot_01/odometry')
        open(nav, 'w', encoding='utf-8').write(text)
        doc = os.path.join(tmp, 'docs/INTERFACES.md')
        open(doc, 'a', encoding='utf-8').write('\n`/robot_01/typo_topic`\n')
        problems = validate_interfaces.check(CONFIG, tmp)
        assert any('nav2.yaml' in p and 'odometry' in p for p in problems), problems
        assert any('INTERFACES.md' in p and 'typo_topic' in p
                   for p in problems), problems


if __name__ == '__main__':
    for name, fn in sorted(globals().items()):
        if name.startswith('test_'):
            fn()
    print('all passed')
