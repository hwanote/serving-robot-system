"""route_executor `reload_map` command, without ROS: rclpy and the message packages are
replaced by stubs, so only the node's own logic (refuse / load / check / swap) runs.

    python3 tests/robot/test_reload_map.py
"""
import importlib.util
import os
import shutil
import sys
import tempfile
import types
from unittest import mock

ROOT = os.path.join(os.path.dirname(__file__), '..', '..')
sys.path.insert(0, os.path.join(ROOT, 'ros_ws', 'src', 'serving_robot_navigation'))
REPO_CONFIG = os.path.join(ROOT, 'config', 'common')

for name in ('rclpy', 'rclpy.action', 'rclpy.duration', 'rclpy.node', 'rclpy.qos',
             'rclpy.time', 'action_msgs', 'action_msgs.msg', 'geometry_msgs',
             'geometry_msgs.msg', 'nav2_msgs', 'nav2_msgs.action', 'nav_msgs',
             'nav_msgs.msg', 'std_msgs', 'std_msgs.msg', 'std_srvs', 'std_srvs.srv',
             'tf2_ros'):
    sys.modules.setdefault(name, mock.MagicMock())
sys.modules['rclpy.node'].Node = object      # RouteExecutor(Node) needs a real class

spec = importlib.util.spec_from_file_location(
    'route_executor', os.path.join(ROOT, 'ros_ws', 'src', 'serving_robot_navigation',
                                   'scripts', 'command', 'route_executor.py'))
route_executor = importlib.util.module_from_spec(spec)
spec.loader.exec_module(route_executor)

from serving_robot_navigation.command.mission import MissionCore, MissionParams  # noqa: E402
from serving_robot_navigation.site_config import load_site  # noqa: E402


def make_node(config_dir):
    n = object.__new__(route_executor.RouteExecutor)
    n.site = load_site(REPO_CONFIG)
    n.core = MissionCore(n.site, MissionParams())
    params = {'config_dir': config_dir, 'home_name': 'standby_01', 'kitchen_name': 'kitchen'}
    n.get_parameter = lambda k: types.SimpleNamespace(value=params[k])
    n.alarms = []
    n._alarm = lambda code, level, text, extra=None: n.alarms.append((code, level, extra))
    n.get_logger = lambda: mock.MagicMock()
    n._publish_status = lambda: n.alarms.append(('STATUS', '', None))
    return n


def copy_config():
    d = tempfile.mkdtemp()
    for f in ('entities.yaml', 'waypoints.yaml', 'zones.yaml'):
        shutil.copy(os.path.join(REPO_CONFIG, f), d)
    return d


def test_reload_takes_the_new_files_and_reports_the_revision():
    d = copy_config()
    n = make_node(d)
    old = n.site.revision
    with open(os.path.join(d, 'zones.yaml'), 'a') as f:
        f.write('\n# changed\n')
    n._reload_map()
    assert n.site.revision != old and n.core.site is n.site
    code, level, extra = n.alarms[0]
    assert code == 'MAP_RELOADED' and level == 'info'
    assert extra['config_revision'] == n.site.revision and extra['old_revision'] == old
    assert n.alarms[-1][0] == 'STATUS'


def test_reload_refused_when_not_idle():
    n = make_node(copy_config())
    old = n.site
    n.core.state = 'DRIVING'
    n._reload_map()
    assert n.alarms[0][0] == 'MAP_RELOAD_REFUSED' and n.site is old


def test_broken_map_keeps_the_old_one():
    d = copy_config()
    n = make_node(d)
    old = n.site
    with open(os.path.join(d, 'waypoints.yaml'), 'w') as f:
        f.write('not: [valid')
    n._reload_map()
    assert n.alarms[0][0] == 'MAP_RELOAD_FAILED' and n.site is old and n.core.site is old


def test_map_without_the_home_pose_is_refused():
    d = copy_config()
    n = make_node(d)
    n.get_parameter = lambda k: types.SimpleNamespace(
        value={'config_dir': d, 'home_name': 'no_such_home',
               'kitchen_name': 'kitchen'}[k])
    old = n.site
    n._reload_map()
    assert n.alarms[0][0] == 'MAP_RELOAD_FAILED' and n.site is old


def test_cmd_dispatch_reaches_reload():
    n = make_node(copy_config())
    called = []
    n._reload_map = lambda: called.append(1)
    n._on_cmd(types.SimpleNamespace(data='{"cmd": "reload_map"}'))
    assert called == [1]


if __name__ == '__main__':
    for name, fn in sorted(globals().items()):
        if name.startswith('test_') and callable(fn):
            fn()
            print('ok  ', name)
    print('all passed')
