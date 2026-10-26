"""Monitoring section: sensors -> facts (see serving_robot_navigation/monitoring/README.md).

    dynamic_scan_filter   scan -> scan_dynamic (the scan without the map's fixed obstacles)
    person_detector       camera (YOLO + depth) -> person_observations   (person_limit:=true)
                          (the state is decided by person_state, decision.launch.py)

Started by navigation.launch.py; can also run alone to try a new sensor module:
    ros2 launch serving_robot_navigation monitoring.launch.py
Add a new monitoring module in MODULES below (one Node).
"""
import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import OpaqueFunction
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node

from serving_robot_navigation.launch_params import camera_params, load_limits



def _modules(context, *args, **kwargs):
    share = get_package_share_directory('serving_robot_navigation')
    sim = {'use_sim_time':
           LaunchConfiguration('use_sim_time').perform(context).lower() == 'true'}
    robot = LaunchConfiguration('robot_name').perform(context)   # '' = default
    rn = {'robot_name': robot} if robot else {}   # nodes read their names
    person_limit = (
        LaunchConfiguration('person_limit').perform(context).lower() == 'true')

    # --- MODULES: one Node per monitoring module --------------------------------
    modules = [
        Node(
            package='serving_robot_navigation',
            executable='dynamic_scan_filter.py',
            name='dynamic_scan_filter',
            output='screen',
            parameters=[
                os.path.join(share, 'config', 'dynamic_scan_filter.yaml'), sim, rn],
        ),
    ]
    if person_limit:
        # one person_detector node handles all cameras of person_safety.cameras
        # (one YOLO model; see decision C5)
        modules.append(Node(
            package='serving_robot_navigation',
            executable='person_detector.py',
            name='person_detector',
            output='screen',
            parameters=[sim, rn, camera_params(load_limits(share))],
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
                              description='Start person_detector (camera + YOLO)'),
        DeclareLaunchArgument(
            'nav2_params_file',
            default_value=os.path.join(share, 'config', 'nav2.yaml'),
            description='Nav2 parameter file (the body size comes from it)'),
        OpaqueFunction(function=_modules),
    ])
