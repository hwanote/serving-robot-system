"""Robot-side input/output layer: everything that differs between simulation
and a real robot lives here, so the layers above (localization, navigation,
safety, serving logic) never change when the code moves to hardware.

Robot I/O contract (identical in sim and real, namespace = robot name):

    publishes   /robot_01/scan   sensor_msgs/LaserScan   frame laser_link
                /robot_01/odom   nav_msgs/Odometry        odom -> base_footprint
                /tf              odom -> base_footprint (+ robot_state_publisher)
                /joint_states
    subscribes  /robot_01/cmd_vel geometry_msgs/Twist     [m/s, rad/s]

mode:=sim   spawn the robot in Gazebo and start the ros_gz bridge
            (serving_robot_gazebo/launch/spawn_robot.launch.py).
mode:=real  start robot_state_publisher only (wall clock). The hardware
            drivers are NOT part of this repository yet. Hardware drivers
            must provide the topics above with the same names and frames:
              - LiDAR driver        -> /robot_01/scan   (frame laser_link)
              - motor/base driver   -> /robot_01/odom + odom TF,
                                       consumes /robot_01/cmd_vel
            Start them here (add them to the `real` branch) or separately.
"""
import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.conditions import LaunchConfigurationEquals
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node

import xacro


def generate_launch_description():
    gazebo_share = get_package_share_directory('serving_robot_gazebo')
    description_share = get_package_share_directory(
        'serving_robot_description')

    robot_description = xacro.process_file(
        os.path.join(description_share, 'urdf', 'serving_robot.urdf.xacro'),
        mappings={'robot_name': 'robot_01'},
    ).toxml()

    sim_robot = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_share, 'launch', 'spawn_robot.launch.py')
        ),
        launch_arguments={
            'spawn_x': LaunchConfiguration('spawn_x'),
            'spawn_y': LaunchConfiguration('spawn_y'),
            'spawn_yaw': LaunchConfiguration('spawn_yaw'),
            'robot_name': LaunchConfiguration('robot_name'),
        }.items(),
        condition=LaunchConfigurationEquals('mode', 'sim'),
    )

    real_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': False,
        }],
        condition=LaunchConfigurationEquals('mode', 'real'),
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'mode', default_value='sim',
            description='sim = Gazebo robot, real = physical robot'),
        DeclareLaunchArgument('spawn_x', default_value='-3.2'),
        DeclareLaunchArgument('spawn_y', default_value='-2.4'),
        DeclareLaunchArgument('spawn_yaw', default_value='0.0'),
        DeclareLaunchArgument(
            'robot_name', default_value='robot_01',
            description='sim only: name of the robot to spawn'),
        sim_robot,
        real_robot_state_publisher,
    ])
