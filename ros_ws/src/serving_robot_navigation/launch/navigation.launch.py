"""Navigation stack for robot_01: localization (map_server + AMCL) + Nav2 + RViz.

Speed and acceleration limits are read from config/common/runtime_limits.yaml
(single source of truth) and override the placeholder values in nav2.yaml.

This file starts the Nav2 core (controller, planner, behaviors, bt_navigator and
their lifecycle manager) and includes the sections of the autonomy part, each
with its own launch file (what they do: serving_robot_navigation/<section>/README.md):

    monitoring.launch.py   dynamic_scan_filter, person_detector (camera)
    decision.launch.py     person_state (person safety state), robot_health
    motion.launch.py       velocity_smoother, person_speed_limiter, collision_monitor,
                           soft_stop_limiter
    command.launch.py      route_executor

Velocity command chain (see motion.launch.py):
    controller_server -> cmd_vel_nav -> velocity_smoother -> cmd_vel_smoothed
        -> person_speed_limiter -> cmd_vel_semantic
        -> collision_monitor -> cmd_vel_safe -> soft_stop_limiter -> <cmd_vel_out>
(person_limit:=false leaves person_detector, person_state and the person_speed_limiter out:
smoothed goes straight to the collision monitor.) The collision monitor is the
last gate before the robot base: whatever sends velocity commands passes through
it. Its stop / slow zones read scan_dynamic (scan without the known obstacles of
the map, made by dynamic_scan_filter); the hard stop reads the raw scan.
"""
import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.actions import OpaqueFunction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node

from serving_robot_navigation.launch_params import load_limits

LIFECYCLE_NODES = [
    'controller_server',
    'planner_server',
    'behavior_server',
    'bt_navigator',
    'velocity_smoother',
    'collision_monitor',
]


def _nav2_nodes(context, *args, **kwargs):
    """Nav2 core + its lifecycle manager (the sections start their own nodes)."""
    share = get_package_share_directory('serving_robot_navigation')

    use_sim_time = (
        LaunchConfiguration('use_sim_time').perform(context).lower() == 'true'
    )
    params_file = LaunchConfiguration('nav2_params_file').perform(context)

    limits = load_limits(share)
    max_lin = float(limits['max_linear_mps'])
    comfort = limits.get('motion_comfort', {})
    ang_acc = float(comfort.get('max_angular_accel_rps2', 0.8))

    bt_xml = os.path.join(
        share, 'behavior_trees', 'navigate_to_pose_simple.xml'
    )

    sim = {'use_sim_time': use_sim_time}

    controller_overrides = {
        'FollowPath.desired_linear_vel': max_lin,
        'FollowPath.max_angular_accel': ang_acc,
    }
    if LaunchConfiguration('free_goal_yaw').perform(context).lower() == 'true':
        # mapping: the goal direction means nothing, so do not turn to it
        controller_overrides['FollowPath.rotate_to_goal_heading'] = False
        controller_overrides['general_goal_checker.yaw_goal_tolerance'] = 3.14

    controller = Node(
        package='nav2_controller',
        executable='controller_server',
        name='controller_server',
        output='screen',
        parameters=[
            params_file,
            sim,
            controller_overrides,
        ],
        remappings=[('cmd_vel', 'cmd_vel_nav')],
    )

    planner = Node(
        package='nav2_planner',
        executable='planner_server',
        name='planner_server',
        output='screen',
        parameters=[params_file, sim],
    )

    behaviors = Node(
        package='nav2_behaviors',
        executable='behavior_server',
        name='behavior_server',
        output='screen',
        parameters=[params_file, sim],
        remappings=[('cmd_vel', 'cmd_vel_nav')],
    )

    bt_navigator = Node(
        package='nav2_bt_navigator',
        executable='bt_navigator',
        name='bt_navigator',
        output='screen',
        parameters=[
            params_file,
            sim,
            {'default_nav_to_pose_bt_xml': bt_xml},
        ],
    )

    lifecycle = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_navigation',
        output='screen',
        parameters=[
            {
                'use_sim_time': use_sim_time,
                'autostart': True,
                'node_names': LIFECYCLE_NODES,
                # The VM simulates slower than real time: be patient.
                'bond_timeout': 10.0,
            }
        ],
    )

    return [controller, planner, behaviors, bt_navigator, lifecycle]


def generate_launch_description():
    share = get_package_share_directory('serving_robot_navigation')

    use_sim_time = LaunchConfiguration('use_sim_time')

    declare_args = [
        DeclareLaunchArgument(
            'delivery_limit_s', default_value='',
            description='test only: delivery time limit in s (default: yaml)'),
        DeclareLaunchArgument(
            'auto_confirm_s', default_value='',
            description='test only: confirm load/handover after this many s'),
        DeclareLaunchArgument(
            'home_name', default_value='',
            description='where the robot returns to (empty = the executor default, '
                        'standby_01). Another robot: its own standby pose'),
        DeclareLaunchArgument(
            'person_limit', default_value='true',
            description='Start person_detector + person_state + person_speed_limiter (camera + '
                        'YOLO + LiDAR state limits the speed). false = leave out'),
        DeclareLaunchArgument(
            'hold_recovery', default_value='false',
            description='Start hold_recovery: after 5 s in an obstacle hold with no goal '
                        'active the robot turns in place to a clear heading (mapping '
                        'mode). false = leave out'),
        DeclareLaunchArgument(
            'free_goal_yaw', default_value='false',
            description='true = arrive at the goal position without turning to the goal '
                        'direction (mapping: every frontier goal has yaw 0). '
                        'false = turn to the goal direction (delivery)'),
        DeclareLaunchArgument(
            'use_sim_time', default_value='true',
            description='Use Gazebo simulation clock'),
        DeclareLaunchArgument(
            'store', default_value='demo_store',
            description='Store map directory name under maps/stores'),
        DeclareLaunchArgument(
            'nav2_params_file',
            default_value=os.path.join(share, 'config', 'nav2.yaml'),
            description='Nav2 parameter file'),
        DeclareLaunchArgument(
            'collision_monitor_params_file',
            default_value=os.path.join(
                share, 'config', 'collision_monitor.yaml'),
            description='Collision monitor parameter file'),
        DeclareLaunchArgument(
            'robot_name', default_value='',
            description='Robot name for all topic names (empty = default_robot of '
                        'config/common/interfaces.yaml)'),
        DeclareLaunchArgument(
            'cmd_vel_out', default_value='',
            description='Final velocity command topic (empty = cmd_vel of '
                        'interfaces.yaml; soft stop limiter output)'),
        DeclareLaunchArgument(
            'localization', default_value='true',
            description='Also start map_server + AMCL'),
        DeclareLaunchArgument(
            'rviz', default_value='true',
            description='Start RViz2 with the navigation view'),
        DeclareLaunchArgument(
            'rviz_config',
            default_value=os.path.join(share, 'rviz', 'navigation.rviz'),
            description='RViz2 configuration file'),
    ]

    localization = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(share, 'launch', 'localization.launch.py')
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'store': LaunchConfiguration('store'),
            'rviz': 'false',
        }.items(),
        condition=IfCondition(LaunchConfiguration('localization')),
    )

    def section(name, extra):
        args = {
            'use_sim_time': use_sim_time,
            'robot_name': LaunchConfiguration('robot_name'),
            'person_limit': LaunchConfiguration('person_limit'),
        }
        args.update(extra)
        return IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(share, 'launch', '%s.launch.py' % name)),
            launch_arguments=args.items(),
        )

    nav2_params = LaunchConfiguration('nav2_params_file')
    monitoring = section('monitoring', {'nav2_params_file': nav2_params})
    decision = section('decision', {'nav2_params_file': nav2_params,
                                    'store': LaunchConfiguration('store')})
    motion = section('motion', {
        'nav2_params_file': nav2_params,
        'collision_monitor_params_file':
            LaunchConfiguration('collision_monitor_params_file'),
        'cmd_vel_out': LaunchConfiguration('cmd_vel_out'),
        'hold_recovery': LaunchConfiguration('hold_recovery'),
    })
    command = section('command', {
        'store': LaunchConfiguration('store'),
        'delivery_limit_s': LaunchConfiguration('delivery_limit_s'),
        'auto_confirm_s': LaunchConfiguration('auto_confirm_s'),
        'home_name': LaunchConfiguration('home_name'),
    })

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='log',
        arguments=['-d', LaunchConfiguration('rviz_config')],
        parameters=[{'use_sim_time': use_sim_time}],
        condition=IfCondition(LaunchConfiguration('rviz')),
    )

    return LaunchDescription(
        declare_args + [localization, monitoring, decision, motion, command,
                        OpaqueFunction(function=_nav2_nodes), rviz]
    )
