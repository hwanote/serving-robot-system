import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PathJoinSubstitution

from launch_ros.actions import Node


def _amcl(context, *args, **kwargs):
    """AMCL node. The initial pose in amcl.yaml can be overridden from outside
    (operation_mode.launch.py passes it from config/common/robot_start.yaml)."""
    use_sim_time = (
        LaunchConfiguration('use_sim_time').perform(context).lower() == 'true'
    )
    params_file = LaunchConfiguration('amcl_params_file').perform(context)

    override = {'use_sim_time': use_sim_time}

    x = LaunchConfiguration('initial_x').perform(context).strip()
    y = LaunchConfiguration('initial_y').perform(context).strip()
    yaw = LaunchConfiguration('initial_yaw').perform(context).strip()
    if x and y and yaw:
        override['initial_pose.x'] = float(x)
        override['initial_pose.y'] = float(y)
        override['initial_pose.z'] = 0.0
        override['initial_pose.yaw'] = float(yaw)

    set_pose = LaunchConfiguration('set_initial_pose').perform(context)
    set_pose = set_pose.strip().lower()
    if set_pose in ('true', 'false'):
        override['set_initial_pose'] = set_pose == 'true'

    return [
        Node(
            package='nav2_amcl',
            executable='amcl',
            name='amcl',
            output='screen',
            parameters=[params_file, override],
        )
    ]


def generate_launch_description():
    share = get_package_share_directory('serving_robot_navigation')

    use_sim_time = LaunchConfiguration('use_sim_time')
    store = LaunchConfiguration('store')
    rviz_config = LaunchConfiguration('localization_rviz_config')

    map_yaml = PathJoinSubstitution(
        [share, 'maps', 'stores', store, 'map.yaml']
    )

    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use Gazebo simulation clock',
    )

    declare_store = DeclareLaunchArgument(
        'store',
        default_value='demo_store',
        description='Store map directory name under maps/stores',
    )

    declare_params_file = DeclareLaunchArgument(
        'amcl_params_file',
        default_value=os.path.join(share, 'config', 'amcl.yaml'),
        description='AMCL parameter file',
    )

    declare_initial = [
        DeclareLaunchArgument(
            'initial_x', default_value='',
            description='AMCL initial x [m]; empty = use amcl.yaml'),
        DeclareLaunchArgument(
            'initial_y', default_value='',
            description='AMCL initial y [m]; empty = use amcl.yaml'),
        DeclareLaunchArgument(
            'initial_yaw', default_value='',
            description='AMCL initial yaw [rad]; empty = use amcl.yaml'),
        DeclareLaunchArgument(
            'set_initial_pose', default_value='',
            description='true/false overrides amcl.yaml; empty = use amcl.yaml'),
    ]

    declare_rviz = DeclareLaunchArgument(
        'rviz',
        default_value='true',
        description='Start RViz2 with the localization view',
    )

    declare_rviz_config = DeclareLaunchArgument(
        'localization_rviz_config',
        default_value=os.path.join(share, 'rviz', 'localization.rviz'),
        description='RViz2 configuration file',
    )

    map_server = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[
            {
                'use_sim_time': use_sim_time,
                'yaml_filename': map_yaml,
            }
        ],
    )

    lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_localization',
        output='screen',
        parameters=[
            {
                'use_sim_time': use_sim_time,
                'autostart': True,
                'node_names': ['map_server', 'amcl'],
            }
        ],
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='log',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': use_sim_time}],
        condition=IfCondition(LaunchConfiguration('rviz')),
    )

    return LaunchDescription([
        declare_use_sim_time,
        declare_store,
        declare_params_file,
        *declare_initial,
        declare_rviz,
        declare_rviz_config,
        map_server,
        OpaqueFunction(function=_amcl),
        lifecycle_manager,
        rviz,
    ])
