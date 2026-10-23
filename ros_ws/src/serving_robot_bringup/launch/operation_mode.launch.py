from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
def generate_launch_description():
    return LaunchDescription([IncludeLaunchDescription(PythonLaunchDescriptionSource(str(Path(get_package_share_directory("serving_robot_navigation"))/"launch/navigation.launch.py")))])
