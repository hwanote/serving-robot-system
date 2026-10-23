"""Start only the independent people subsystem; the robot stack is untouched."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os
import yaml


def nodes(context, *args, **kwargs):
    default=os.path.join(get_package_share_directory('serving_robot_person_simulator'),'config','policy.yaml')
    config=LaunchConfiguration('config').perform(context)
    if not config:
        config=default
    with open(config,encoding='utf-8') as f:
        cfg=yaml.safe_load(f)
    world=cfg['world']
    prefix=cfg['model_prefix']
    if prefix!='test_person_':
        raise ValueError('Unexpected model prefix')
    # Single bridge process, max 8 person velocity feeds only.
    # Gazebo named poses are read by manager's independent gz CLI stream.
    topics = [f'/model/{prefix}{i:02d}/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist'
               for i in range(1,9)]
    return [
        Node(package='ros_gz_bridge',executable='parameter_bridge',name='people_bridge',
             arguments=topics,output='screen'),
        Node(package='serving_robot_person_simulator',executable='watchdog',name='person_simulator_watchdog',output='screen'),
        Node(package='serving_robot_person_simulator',executable='manager',name='person_simulator_manager',
             parameters=[{'config_path':config,'use_sim_time':True}],output='screen'),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('config',default_value='',description='Absolute path to approved policy.yaml'),
        OpaqueFunction(function=nodes),
    ])
