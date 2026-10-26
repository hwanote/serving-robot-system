"""Motion section: decisions -> speed commands (see serving_robot_navigation/motion/README.md).

Velocity command chain:
    controller_server -> cmd_vel_nav -> velocity_smoother -> cmd_vel_smoothed
        -> person_speed_limiter -> cmd_vel_semantic        (person_limit:=true)
        -> collision_monitor -> cmd_vel_safe -> soft_stop_limiter -> <cmd_vel_out>
(person_limit:=false leaves the person_speed_limiter out: smoothed goes straight
to the collision monitor.) soft_stop_limiter is the only owner of the final
cmd_vel. velocity_smoother and collision_monitor are Nav2 nodes: the lifecycle
manager of navigation.launch.py brings them up.

Started by navigation.launch.py; can also run alone.
Add a new speed-limit stage in MODULES below and keep the topic names of the
chain connected (cmd_vel_in_topic of the collision monitor).
"""
import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import OpaqueFunction
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node

from serving_robot_navigation.interfaces import names_for
from serving_robot_navigation.launch_params import (
    limiter_params, load_limits, monitor_zones, person_params)


def _modules(context, *args, **kwargs):
    share = get_package_share_directory('serving_robot_navigation')
    sim = {'use_sim_time':
           LaunchConfiguration('use_sim_time').perform(context).lower() == 'true'}
    params_file = LaunchConfiguration('nav2_params_file').perform(context)
    monitor_params_file = LaunchConfiguration(
        'collision_monitor_params_file').perform(context)
    robot = LaunchConfiguration('robot_name').perform(context)   # '' = default
    names = names_for(robot)
    cmd_vel_out = (LaunchConfiguration('cmd_vel_out').perform(context)
                   or names.topic('cmd_vel'))
    rn = {'robot_name': robot} if robot else {}   # nodes read their names
    person_limit = (
        LaunchConfiguration('person_limit').perform(context).lower() == 'true')
    hold_recovery = (
        LaunchConfiguration('hold_recovery').perform(context).lower() == 'true')

    limits = load_limits(share)
    max_lin = float(limits['max_linear_mps'])
    max_ang = float(limits['max_angular_rps'])
    comfort = limits.get('motion_comfort', {})
    lin_acc = float(comfort.get('max_linear_accel_mps2', 0.4))
    lin_dec = float(comfort.get('max_linear_decel_mps2', 0.4))
    ang_acc = float(comfort.get('max_angular_accel_rps2', 0.8))

    # --- MODULES: the chain, in order -------------------------------------------
    smoother = Node(
        package='nav2_velocity_smoother',
        executable='velocity_smoother',
        name='velocity_smoother',
        output='screen',
        parameters=[
            params_file,
            sim,
            {
                'max_velocity': [max_lin, 0.0, max_ang],
                'max_accel': [lin_acc, 0.0, ang_acc],
                'max_decel': [-lin_dec, 0.0, -ang_acc],
            },
        ],
        # Output stays on the default topic `cmd_vel_smoothed`;
        # the next stage reads it.
        remappings=[('cmd_vel', 'cmd_vel_nav')],
    )

    monitor = Node(
        package='nav2_collision_monitor',
        executable='collision_monitor',
        name='collision_monitor',
        output='screen',
        parameters=[
            monitor_params_file,
            sim,
            {
                'cmd_vel_in_topic': (
                    'cmd_vel_semantic' if person_limit else 'cmd_vel_smoothed'),
                'cmd_vel_out_topic': 'cmd_vel_safe',
            },
            monitor_zones(limits, params_file),
        ],
    )

    limiter = Node(
        package='serving_robot_navigation',
        executable='soft_stop_limiter.py',
        name='soft_stop_limiter',
        output='screen',
        parameters=[
            sim,
            {'in_topic': 'cmd_vel_safe', 'out_topic': cmd_vel_out},
            rn,
            limiter_params(limits, params_file),
        ],
    )

    modules = [smoother, monitor, limiter]
    if hold_recovery:
        # Mapping mode: turn in place out of an obstacle hold (never reverses).
        lp = limiter_params(limits, params_file)
        modules.append(Node(
            package='serving_robot_navigation',
            executable='hold_recovery.py',
            name='hold_recovery',
            output='screen',
            parameters=[sim, rn, {
                'resume_clear_distance_m': lp['resume_clear_distance_m'],
                'front_m': lp['front_m'],
                'half_width_m': lp['half_width_m'],
            }],
        ))
    if person_limit:
        _det_params, lim_params = person_params(limits, params_file)
        modules.append(Node(
            package='serving_robot_navigation',
            executable='person_speed_limiter.py',
            name='person_speed_limiter',
            output='screen',
            parameters=[sim, rn, lim_params],
        ))
    # ---------------------------------------------------------------------------
    return modules


def generate_launch_description():
    share = get_package_share_directory('serving_robot_navigation')
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true',
                              description='Use Gazebo simulation clock'),
        DeclareLaunchArgument('robot_name', default_value='',
                              description='Robot name (empty = default_robot of '
                                          'config/common/interfaces.yaml)'),
        DeclareLaunchArgument('person_limit', default_value='true',
                              description='Start person_speed_limiter (person state '
                                          'limits the speed)'),
        DeclareLaunchArgument('hold_recovery', default_value='false',
                              description='Start hold_recovery (turns in place out of '
                                          'an obstacle hold; mapping mode)'),
        DeclareLaunchArgument(
            'nav2_params_file',
            default_value=os.path.join(share, 'config', 'nav2.yaml'),
            description='Nav2 parameter file'),
        DeclareLaunchArgument(
            'collision_monitor_params_file',
            default_value=os.path.join(share, 'config', 'collision_monitor.yaml'),
            description='Collision monitor parameter file'),
        DeclareLaunchArgument(
            'cmd_vel_out', default_value='',
            description='Final velocity command topic (empty = cmd_vel of '
                        'interfaces.yaml; soft stop limiter output)'),
        OpaqueFunction(function=_modules),
    ])
