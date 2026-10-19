"""Tests of the section layout checks (no ROS).

    python3 tests/robot/test_layout.py
"""
import os
import shutil
import sys
import tempfile

ROOT = os.path.join(os.path.dirname(__file__), '..', '..')
sys.path.insert(0, os.path.join(ROOT, 'scripts'))
import validate_layout  # noqa: E402

NAV = 'ros_ws/src/serving_robot_navigation'


def _copy(dst):
    shutil.copytree(os.path.join(ROOT, NAV), os.path.join(dst, NAV),
                    ignore=shutil.ignore_patterns('__pycache__', 'maps'))


def _write(path, text):
    with open(path, 'w', encoding='utf-8') as f:
        f.write(text)


def test_repository_layout_is_plugged_in():
    assert validate_layout.check(os.path.abspath(ROOT)) == []


def test_new_node_that_is_not_plugged_in_is_found():
    with tempfile.TemporaryDirectory() as tmp:
        _copy(tmp)
        _write(os.path.join(tmp, NAV, 'scripts/monitoring/new_sensor.py'), '# node\n')
        problems = validate_layout.check(tmp)
        joined = '\n'.join(problems)
        assert 'node new_sensor.py is not installed in CMakeLists.txt' in joined
        assert 'node new_sensor.py is not named in the README' in joined
        assert 'node new_sensor.py is not started by monitoring.launch.py' in joined


def test_new_module_missing_from_readme_is_found():
    with tempfile.TemporaryDirectory() as tmp:
        _copy(tmp)
        _write(os.path.join(tmp, NAV, 'serving_robot_navigation/decision/new_rule.py'),
               '"""rule"""\n')
        problems = validate_layout.check(tmp)
        assert any('module new_rule.py is not named in the README' in p
                   for p in problems), problems


def test_loose_files_and_missing_files_are_found():
    with tempfile.TemporaryDirectory() as tmp:
        _copy(tmp)
        _write(os.path.join(tmp, NAV, 'scripts/loose.py'), '# node\n')
        _write(os.path.join(tmp, NAV, 'serving_robot_navigation/loose_logic.py'), '')
        os.remove(os.path.join(tmp, NAV, 'scripts/command/serve.py'))
        problems = '\n'.join(validate_layout.check(tmp))
        assert 'scripts/loose.py is outside the sections' in problems
        assert 'serving_robot_navigation/loose_logic.py is outside the sections' in problems
        assert 'CMakeLists.txt installs scripts/command/serve.py' in problems


def test_launch_that_starts_a_foreign_node_is_found():
    with tempfile.TemporaryDirectory() as tmp:
        _copy(tmp)
        path = os.path.join(tmp, NAV, 'launch/command.launch.py')
        text = open(path, encoding='utf-8').read().replace(
            "executable='route_executor.py'", "executable='person_detector.py'")
        _write(path, text)
        problems = '\n'.join(validate_layout.check(tmp))
        assert 'command.launch.py starts person_detector.py' in problems
        assert 'node route_executor.py is not started by command.launch.py' in problems


def test_navigation_launch_must_include_every_section():
    with tempfile.TemporaryDirectory() as tmp:
        _copy(tmp)
        path = os.path.join(tmp, NAV, 'launch/navigation.launch.py')
        text = open(path, encoding='utf-8').read().replace(
            'motion.launch.py', 'x.launch.py').replace("'motion'", "'x'")
        _write(path, text)
        problems = '\n'.join(validate_layout.check(tmp))
        assert 'navigation.launch.py does not include motion.launch.py' in problems


if __name__ == '__main__':
    for name, fn in sorted(globals().items()):
        if name.startswith('test_'):
            fn()
    print('all passed')
