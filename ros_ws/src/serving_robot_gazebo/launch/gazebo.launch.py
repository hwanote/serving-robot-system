import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PathJoinSubstitution


def generate_launch_description():
    gazebo_share = get_package_share_directory(
        'serving_robot_gazebo'
    )

    ros_gz_sim_share = get_package_share_directory(
        'ros_gz_sim'
    )

    # world:=<file name under worlds/, or an absolute path>.
    # Default = the normal cafe (nothing changes unless world is given).
    world_file = PathJoinSubstitution([
        gazebo_share,
        'worlds',
        LaunchConfiguration('world'),
    ])

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                ros_gz_sim_share,
                'launch',
                'gz_sim.launch.py',
            )
        ),
        launch_arguments={
            'gz_args': ['-r -v4 ', world_file],
            'on_exit_shutdown': 'true',
        }.items(),
    )

    declare_world = DeclareLaunchArgument(
        'world',
        default_value='cafe.world.sdf',
        description='World file under worlds/ (or an absolute path). '
                    'cafe_closed.world.sdf = entrance wall closed (mapping tests)',
    )

    return LaunchDescription([
        declare_world,
        gazebo,
    ])
