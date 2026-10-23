import os
import re
import tempfile

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import OpaqueFunction
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node

import xacro


def _bridge_config(path, robot):
    """bridge.yaml is written for robot_01. For another robot, write a copy with
    the name replaced (topics /robot_02/..., Gazebo /model/robot_02/...)."""
    if robot == 'robot_01':
        return path
    with open(path, 'r', encoding='utf-8') as f:
        text = f.read().replace('robot_01', robot)
    out = os.path.join(tempfile.gettempdir(),
                       'serving_robot_bridge_%s.yaml' % robot)
    with open(out, 'w', encoding='utf-8') as f:
        f.write(text)
    return out


def _build(context, *args, **kwargs):
    robot = LaunchConfiguration('robot_name').perform(context).strip()
    if not re.match(r'^[a-z][a-z0-9_]*$', robot):
        raise RuntimeError("bad robot_name %r (like robot_01)" % robot)

    description_share = get_package_share_directory(
        'serving_robot_description'
    )

    gazebo_share = get_package_share_directory(
        'serving_robot_gazebo'
    )

    xacro_file = os.path.join(
        description_share,
        'urdf',
        'serving_robot.urdf.xacro',
    )

    bridge_config = _bridge_config(os.path.join(
        gazebo_share,
        'config',
        'bridge.yaml',
    ), robot)

    robot_description = xacro.process_file(
        xacro_file,
        mappings={
            'robot_name': robot,
        },
    ).toxml()

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[
            {
                'robot_description': robot_description,
                'use_sim_time': True,
            }
        ],
    )

    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        name='spawn_' + robot,
        output='screen',
        arguments=[
            '-world', 'cafe',
            '-string', robot_description,
            '-name', robot,
            '-allow_renaming', 'false',
            '-x', LaunchConfiguration('spawn_x'),
            '-y', LaunchConfiguration('spawn_y'),
            '-z', '0.02',
            '-Y', LaunchConfiguration('spawn_yaw'),
        ],
    )

    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='serving_robot_bridge',
        output='screen',
        parameters=[
            {
                'config_file': bridge_config,
            }
        ],
    )

    # Spawn pose in the world/map frame (yaw in radians). The defaults match
    # config/common/robot_start.yaml; operation_mode.launch.py passes that file's
    # values so there is a single source of truth.
    return [robot_state_publisher, bridge, spawn_robot]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('spawn_x', default_value='-3.2'),
        DeclareLaunchArgument('spawn_y', default_value='-2.4'),
        DeclareLaunchArgument('spawn_yaw', default_value='0.0'),
        DeclareLaunchArgument(
            'robot_name', default_value='robot_01',
            description='Gazebo model + topic name (robot_02 for a second '
                        'robot: run it in its own ROS_DOMAIN_ID)'),
        OpaqueFunction(function=_build),
    ])
