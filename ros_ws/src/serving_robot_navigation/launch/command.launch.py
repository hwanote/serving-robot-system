"""Command section: missions and orders (see serving_robot_navigation/command/README.md).

    route_executor   mission commands -> Nav2 goals; mission_status and alarm out

Started by navigation.launch.py; can also run alone (Nav2 must be running).
go_to.py and serve.py are command line tools, not launched.
Add a new command-section node in MODULES below (one Node).
"""
import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import OpaqueFunction
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node

from serving_robot_navigation.launch_params import executor_params, load_limits


def _modules(context, *args, **kwargs):
    share = get_package_share_directory('serving_robot_navigation')
    sim = {'use_sim_time':
           LaunchConfiguration('use_sim_time').perform(context).lower() == 'true'}
    robot = LaunchConfiguration('robot_name').perform(context)   # '' = default
    rn = {'robot_name': robot} if robot else {}   # nodes read their names
    person_limit = (
        LaunchConfiguration('person_limit').perform(context).lower() == 'true')
    # the map picture Nav2's map_server loads (same file as localization.launch.py)
    store = LaunchConfiguration('store').perform(context)
    map_yaml = {'map_yaml': os.path.join(share, 'maps', 'stores', store, 'map.yaml')}

    # test shortcuts (empty = use runtime_limits.yaml)
    test_overrides = {}
    for key in ('delivery_limit_s', 'auto_confirm_s'):
        value = LaunchConfiguration(key).perform(context)
        if value:
            test_overrides[key] = float(value)
    home = LaunchConfiguration('home_name').perform(context)
    if home:
        test_overrides['home_name'] = home   # return pose of this robot

    # --- MODULES: one Node per command-section node -----------------------------
    modules = [
        Node(
            package='serving_robot_navigation',
            executable='route_executor.py',
            name='route_executor',
            output='screen',
            parameters=[sim, rn, executor_params(load_limits(share), person_limit),
                        test_overrides, map_yaml],
        ),
    ]
    # ---------------------------------------------------------------------------
    return modules


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true',
                              description='Use Gazebo simulation clock'),
        DeclareLaunchArgument('robot_name', default_value='',
                              description='Robot name (empty = default_robot of '
                                          'config/common/interfaces.yaml)'),
        DeclareLaunchArgument('store', default_value='demo_store',
                              description='Store map directory under maps/stores'),
        DeclareLaunchArgument('person_limit', default_value='true',
                              description='The person detector runs: the executor '
                                          'watches it and tells the operator when silent'),
        DeclareLaunchArgument(
            'delivery_limit_s', default_value='',
            description='test only: delivery time limit in s (default: yaml)'),
        DeclareLaunchArgument(
            'auto_confirm_s', default_value='',
            description='test only: confirm load/handover after this many s'),
        DeclareLaunchArgument(
            'home_name', default_value='',
            description='Return pose (empty = executor default standby_01)'),
        OpaqueFunction(function=_modules),
    ])
