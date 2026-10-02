import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    gazebo_share = get_package_share_directory(
        'serving_robot_gazebo'
    )

    ros_gz_sim_share = get_package_share_directory(
        'ros_gz_sim'
    )

    world_file = os.path.join(
        gazebo_share,
        'worlds',
        'cafe.world.sdf',
    )

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                ros_gz_sim_share,
                'launch',
                'gz_sim.launch.py',
            )
        ),
        launch_arguments={
            'gz_args': f'-r -v4 {world_file}',
            'on_exit_shutdown': 'true',
        }.items(),
    )

    return LaunchDescription([
        gazebo,
    ])
