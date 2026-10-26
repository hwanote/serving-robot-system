"""Mapping mode: start the whole MAPPING stack with ONE command (simulation).

Each stage starts only after the previous one is really ready (conditions are
checked by scripts/wait_for_ready.py, no fixed sleeps), same idea as
operation_mode.launch.py:

    1 Gazebo world (closed cafe) -> 2 robot + bridge -> 3 SLAM (slam_toolbox)
    -> 4 Nav2 (no localization: the SLAM map is the map)
    -> 5 frontier_visualizer (produces the exploration candidates; without it
       the explorer never gets a goal)
    -> 6 RViz (last, so Nav2 starts while the CPU is not busy rendering)

This launch does NOT start the exploration itself. Start/stop it in a second
terminal, so a later "Start Mapping" button only has to control the session:

    ros2 launch serving_robot_bringup mapping_mode.launch.py
    python3 <repo>/ros_ws/src/serving_robot_mapping/tools/run_lidar_mapping_session.py \\
        --run --max-seconds 1200 --log-dir ~/mapping-run-logs

Started here on purpose: free_goal_yaw (the explorer sends every goal with
yaw 0, so the robot would turn to 0 degrees at each arrival for nothing) and
hold_recovery (after 5 s in an obstacle hold with no goal
active, turns in place to a clear heading; the explorer would wait forever in a
corner otherwise).

Not started on purpose: person_detector / person_state / person_speed_limiter
(the session tool refuses them), exploration_manager (the session tool starts
it), AMCL / map_server (SLAM publishes map->odom and /map).

Arguments:
    world:=false        the Gazebo world is already running
    world_file:=NAME    world under serving_robot_gazebo/worlds
                        (default cafe_closed.world.sdf = entrance wall closed,
                        so the robot cannot map the outside)
    rviz:=false         no RViz (navigation.rviz is used: it shows the live /map)

The robot starts where config/common/robot_start.yaml says; the SLAM map frame
starts at that pose (so a saved map is shifted from the Gazebo world by it; see
scripts/seal_map_edges.py --shift-origin).
Only mode sim is supported so far (the real robot is not tested).
"""
import math
import os

import yaml

from ament_index_python.packages import get_package_prefix
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import EmitEvent
from launch.actions import ExecuteProcess
from launch.actions import IncludeLaunchDescription
from launch.actions import LogInfo
from launch.actions import OpaqueFunction
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node

from serving_robot_navigation.interfaces import names_for
from serving_robot_navigation.site_config import load_site

TAG = '[mapping_mode]'


def _waiter(name, args, timeout):
    script = os.path.join(
        get_package_prefix('serving_robot_bringup'),
        'lib', 'serving_robot_bringup', 'wait_for_ready.py')
    return ExecuteProcess(
        cmd=['python3', script, '--name', name,
             '--timeout', str(timeout)] + args,
        name='wait_' + name,
        output='screen',
    )


def _chain(steps, idx=0):
    """Actions for steps[idx:]. A step with a `wait` starts the next step only
    after its waiter exits with code 0; otherwise the launch shuts down."""
    step = steps[idx]
    out = [LogInfo(msg=step['title'])] + list(step['actions'])
    rest = _chain(steps, idx + 1) if idx + 1 < len(steps) else []
    wait = step.get('wait')
    if wait is None:
        return out + rest

    waiter = _waiter(wait['name'], wait['args'], wait['timeout'])

    def on_exit(event, context, _rest=rest, _w=wait):
        if event.returncode == 0:
            return [LogInfo(msg=_w['ok'])] + _rest
        return [
            LogInfo(msg=TAG + ' FAILED: ' + _w['fail']),
            EmitEvent(event=Shutdown(reason='startup condition not met')),
        ]

    out.append(RegisterEventHandler(
        OnProcessExit(target_action=waiter, on_exit=on_exit)))
    out.append(waiter)
    return out


def _build(context, *args, **kwargs):
    mode = LaunchConfiguration('mode').perform(context).strip().lower()
    if mode != 'sim':
        raise RuntimeError("mapping_mode supports mode:=sim only for now, got '%s'" % mode)
    nm = names_for('')
    timeout = float(LaunchConfiguration('wait_timeout').perform(context))
    store = LaunchConfiguration('store').perform(context)
    world_file = LaunchConfiguration('world_file').perform(context)
    want_rviz = LaunchConfiguration('rviz').perform(context).lower() == 'true'
    with_world = LaunchConfiguration('world').perform(context).lower() == 'true'

    # --- start pose (single source of truth, same file as operation_mode) ----
    start_cfg = LaunchConfiguration('start_config').perform(context)
    with open(start_cfg, 'r', encoding='utf-8') as f:
        pose = yaml.safe_load(f)[nm.robot]['start_pose']
    if 'pose' in pose:
        named = load_site().resolve(pose['pose'])
        x, y, yaw = named.x, named.y, named.yaw_rad
    else:
        x, y = float(pose['x']), float(pose['y'])
        yaw = math.radians(float(pose['yaw_deg']))

    bringup = get_package_share_directory('serving_robot_bringup')
    gazebo = get_package_share_directory('serving_robot_gazebo')
    nav = get_package_share_directory('serving_robot_navigation')

    def include(path, arguments):
        return IncludeLaunchDescription(
            PythonLaunchDescriptionSource(path),
            launch_arguments=arguments.items())

    steps = []

    if with_world:
        steps.append({
            'title': '%s Gazebo world (%s)' % (TAG, world_file),
            'actions': [include(
                os.path.join(gazebo, 'launch', 'gazebo.launch.py'),
                {'world': world_file})],
            'wait': {
                'name': 'gazebo', 'args': ['--gz-topic', '/clock'],
                'timeout': timeout,
                'ok': TAG + ' Gazebo is up',
                'fail': 'Gazebo did not start (check the Gazebo window/log '
                        'and leftover gz processes: ps aux | grep gz)',
            },
        })

    steps.append({
        'title': TAG + ' robot + bridge',
        'actions': [include(
            os.path.join(bringup, 'launch', 'robot_base.launch.py'),
            {'mode': 'sim', 'spawn_x': str(x), 'spawn_y': str(y),
             'spawn_yaw': str(yaw), 'robot_name': nm.robot})],
        'wait': {
            'name': 'robot',
            'args': ['--clock', '--tf', 'odom', 'base_footprint',
                     '--topic', nm.topic('scan')],
            'timeout': timeout,
            'ok': TAG + ' robot is publishing scan/odom/TF',
            'fail': 'robot did not publish /clock, %s and '
                    'odom->base_footprint (spawn failed? bridge running?)'
                    % nm.topic('scan'),
        },
    })

    steps.append({
        'title': TAG + ' SLAM (slam_toolbox, mode mapping)',
        'actions': [include(
            os.path.join(nav, 'launch', 'mapping.launch.py'),
            {'use_sim_time': 'true'})],
        'wait': {
            'name': 'slam', 'args': ['--tf', 'map', 'odom', '--topic', '/map'],
            'timeout': timeout,
            'ok': TAG + ' SLAM is running (map->odom, /map)',
            'fail': 'slam_toolbox did not publish map->odom and /map '
                    '(is %s publishing? see the SLAM log above)' % nm.topic('scan'),
        },
    })

    steps.append({
        'title': TAG + ' Nav2 (localization off, person limit off, hold recovery on, free goal yaw)',
        'actions': [include(
            os.path.join(nav, 'launch', 'navigation.launch.py'),
            {'use_sim_time': 'true', 'store': store,
             'localization': 'false', 'rviz': 'false',
             'person_limit': 'false', 'hold_recovery': 'true',
             'free_goal_yaw': 'true', 'home_name': '',
             'delivery_limit_s': '', 'auto_confirm_s': ''})],
        'wait': {
            'name': 'nav2',
            'args': ['--lifecycle', 'controller_server',
                     '--lifecycle', 'planner_server',
                     '--lifecycle', 'behavior_server',
                     '--lifecycle', 'bt_navigator',
                     '--lifecycle', 'velocity_smoother',
                     '--lifecycle', 'collision_monitor',
                     '--topic', nm.topic('scan_dynamic'),
                     '--topic', nm.topic('cmd_vel')],
            'timeout': timeout,
            'ok': TAG + ' Nav2 is active',
            'fail': 'Nav2 nodes did not become active (see the Nav2 log above)',
        },
    })

    steps.append({
        'title': TAG + ' frontier_visualizer (exploration candidates)',
        'actions': [Node(
            package='serving_robot_mapping', executable='frontier_visualizer',
            name='frontier_visualizer', output='screen',
            parameters=[{'use_sim_time': True}])],
    })

    if want_rviz:
        steps.append({
            'title': TAG + ' RViz (started last)',
            'actions': [Node(
                package='rviz2', executable='rviz2', name='rviz2',
                output='log',
                arguments=['-d', os.path.join(nav, 'rviz', 'navigation.rviz')],
                parameters=[{'use_sim_time': True}])],
        })

    steps.append({
        'title': TAG + ' READY. Start the exploration in another terminal: '
                 'python3 <repo>/ros_ws/src/serving_robot_mapping/tools/'
                 'run_lidar_mapping_session.py --run --max-seconds 1200 '
                 '--log-dir ~/mapping-run-logs',
        'actions': [],
    })

    return _chain(steps)


def generate_launch_description():
    bringup = get_package_share_directory('serving_robot_bringup')
    return LaunchDescription([
        DeclareLaunchArgument(
            'mode', default_value='sim',
            description='sim only for now (the real robot is not tested)'),
        DeclareLaunchArgument(
            'store', default_value='demo_store',
            description='Store name for the Nav2 launch (map revision only; '
                        'the map itself comes from SLAM)'),
        DeclareLaunchArgument(
            'world', default_value='true',
            description='start the Gazebo world; false = already running'),
        DeclareLaunchArgument(
            'world_file', default_value='cafe_closed.world.sdf',
            description='world file under serving_robot_gazebo/worlds'),
        DeclareLaunchArgument(
            'rviz', default_value='true',
            description='Start RViz2 (true/false)'),
        DeclareLaunchArgument(
            'start_config',
            default_value=os.path.join(
                bringup, 'config', 'common', 'robot_start.yaml'),
            description='Robot start pose file'),
        DeclareLaunchArgument(
            'wait_timeout', default_value='180',
            description='Seconds each stage may take before giving up'),
        OpaqueFunction(function=_build),
    ])
