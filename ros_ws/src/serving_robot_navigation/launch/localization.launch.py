"""Standalone localization from the Part 1 map; mapping is never started here."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory
from pathlib import Path

def generate_launch_description():
    config=str(Path(get_package_share_directory("serving_robot_navigation"))/"config/amcl.yaml")
    return LaunchDescription([
        DeclareLaunchArgument("map_yaml"),DeclareLaunchArgument("robot_id",default_value="robot_01"),
        DeclareLaunchArgument("use_sim_time",default_value="true"),
        Node(package="nav2_map_server",executable="map_server",name="map_server",namespace=LaunchConfiguration("robot_id"),
             parameters=[{"yaml_filename":LaunchConfiguration("map_yaml"),"use_sim_time":ParameterValue(LaunchConfiguration("use_sim_time"),value_type=bool)}]),
        Node(package="nav2_amcl",executable="amcl",name="amcl",namespace=LaunchConfiguration("robot_id"),parameters=[config,{"use_sim_time":ParameterValue(LaunchConfiguration("use_sim_time"),value_type=bool)}]),
        Node(package="nav2_lifecycle_manager",executable="lifecycle_manager",namespace=LaunchConfiguration("robot_id"),
             parameters=[{"autostart":True,"use_sim_time":ParameterValue(LaunchConfiguration("use_sim_time"),value_type=bool),"node_names":["map_server","amcl"]}])])
