import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription

from launch_ros.actions import Node

import xacro


def generate_launch_description():
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

    bridge_config = os.path.join(
        gazebo_share,
        'config',
        'bridge.yaml',
    )

    robot_description = xacro.process_file(
        xacro_file,
        mappings={
            'robot_name': 'robot_01',
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
        name='spawn_robot_01',
        output='screen',
        arguments=[
            '-world', 'cafe',
            '-string', robot_description,
            '-name', 'robot_01',
            '-allow_renaming', 'false',
            '-x', '-3.2',
            '-y', '-2.25',
            '-z', '0.02',
            '-Y', '0.0',
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

    return LaunchDescription([
        robot_state_publisher,
        bridge,
        spawn_robot,
    ])
