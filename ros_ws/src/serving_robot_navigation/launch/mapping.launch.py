import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():

    use_sim_time = LaunchConfiguration('use_sim_time')
    slam_params_file = LaunchConfiguration('slam_params_file')

    navigation_share = get_package_share_directory(
        'serving_robot_navigation'
    )

    slam_toolbox_share = get_package_share_directory(
        'slam_toolbox'
    )

    default_slam_params_file = os.path.join(
        navigation_share,
        'config',
        'slam.yaml',
    )

    slam_toolbox_launch_file = os.path.join(
        slam_toolbox_share,
        'launch',
        'online_async_launch.py',
    )

    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use Gazebo simulation clock',
    )

    declare_slam_params_file = DeclareLaunchArgument(
        'slam_params_file',
        default_value=default_slam_params_file,
        description='SLAM Toolbox parameter file',
    )

    slam_toolbox = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            slam_toolbox_launch_file
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'slam_params_file': slam_params_file,
        }.items(),
    )

    return LaunchDescription([
        declare_use_sim_time,
        declare_slam_params_file,
        slam_toolbox,
    ])
