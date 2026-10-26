"""Decision section: facts -> states (see serving_robot_navigation/decision/README.md).

    person_state   person_observations + scan_dynamic + obstacle_hold
                   -> person_detection (NORMAL / SLOW / STOP / SENSOR_STALE)   (person_limit:=true)
    robot_health   everything above + mission_status + amcl_pose + TF
                   -> health (robot / safety / localization state, team enums)

Started by navigation.launch.py; can also run alone to try a new rule:
    ros2 launch serving_robot_navigation decision.launch.py
Add a new decision node in MODULES below (one Node).
"""
import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import OpaqueFunction
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node

from serving_robot_navigation.launch_params import (
    health_params, load_limits, map_revision, person_params)


def _modules(context, *args, **kwargs):
    share = get_package_share_directory('serving_robot_navigation')
    sim = {'use_sim_time':
           LaunchConfiguration('use_sim_time').perform(context).lower() == 'true'}
    robot = LaunchConfiguration('robot_name').perform(context)   # '' = default
    rn = {'robot_name': robot} if robot else {}   # nodes read their names
    person_limit = (
        LaunchConfiguration('person_limit').perform(context).lower() == 'true')
    nav2_params_file = LaunchConfiguration('nav2_params_file').perform(context)

    store = LaunchConfiguration('store').perform(context)
    limits = load_limits(share)

    # --- MODULES: one Node per decision module -----------------------------------
    modules = [
        Node(
            package='serving_robot_navigation',
            executable='robot_health.py',
            name='robot_health',
            output='screen',
            parameters=[sim, rn, health_params(limits, person_limit,
                                               map_revision(share, store))],
        ),
    ]
    if person_limit:
        judge_params, _lim_params = person_params(limits, nav2_params_file)
        modules.append(Node(
            package='serving_robot_navigation',
            executable='person_state.py',
            name='person_state',
            output='screen',
            parameters=[sim, rn, judge_params],
        ))
    # ---------------------------------------------------------------------------
    return modules


def generate_launch_description():
    share = get_package_share_directory('serving_robot_navigation')
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true',
                              description='Use Gazebo simulation clock'),
        DeclareLaunchArgument('robot_name', default_value='',
                              description='Robot name (empty = default_robot of '
                                          'config/common/interfaces.yaml)'),
        DeclareLaunchArgument('person_limit', default_value='true',
                              description='Start person_state (person safety state '
                                          'from the camera observations + LiDAR)'),
        DeclareLaunchArgument('store', default_value='demo_store',
                              description='Store map directory (map_revision for health)'),
        DeclareLaunchArgument(
            'nav2_params_file',
            default_value=os.path.join(share, 'config', 'nav2.yaml'),
            description='Nav2 parameter file (the body size comes from it)'),
        OpaqueFunction(function=_modules),
    ])
