"""Operation mode: start the whole robot stack in the right order with ONE
command. Each stage starts only after the previous one is really ready
(conditions are checked by scripts/wait_for_ready.py, no fixed sleeps).

    mode:=sim   1 Gazebo world -> 2 robot + bridge -> 3 localization
                -> 4 Nav2 -> 5 RViz (last, so Nav2 starts while the CPU is
                not busy rendering; a slow machine otherwise makes the
                lifecycle manager give up)
    mode:=real  1 robot drivers -> 2 localization + RViz -> 3 Nav2
                (RViz stays early: the person sets the start pose in it)

Everything above the robot I/O contract (see robot_base.launch.py) is the
same in both modes.

Usage:
    ros2 launch serving_robot_bringup operation_mode.launch.py
    ros2 launch serving_robot_bringup operation_mode.launch.py rviz:=false
    ros2 launch serving_robot_bringup operation_mode.launch.py mode:=real

The start pose of the robot comes from config/common/robot_start.yaml.
"""
import math
import os
import tempfile

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


def _renamed(path, robot):
    """Nav2/AMCL/collision YAMLs name /robot_01/... topics. For another robot,
    return a copy with the name replaced (robot_01 = the file itself)."""
    if robot == 'robot_01':
        return path
    with open(path, 'r', encoding='utf-8') as f:
        text = f.read().replace('/robot_01/', '/%s/' % robot)
    out = os.path.join(tempfile.gettempdir(),
                       '%s_%s' % (robot, os.path.basename(path)))
    with open(out, 'w', encoding='utf-8') as f:
        f.write(text)
    return out


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
    """Return the actions to emit for steps[idx:]. A step with a `wait` starts
    the next step only after its waiter exits with code 0; otherwise the whole
    launch shuts down with a clear message."""
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
            LogInfo(msg='[operation_mode] FAILED: ' + _w['fail']),
            EmitEvent(event=Shutdown(reason='startup condition not met')),
        ]

    out.append(RegisterEventHandler(
        OnProcessExit(target_action=waiter, on_exit=on_exit)))
    out.append(waiter)
    return out


def _build(context, *args, **kwargs):
    mode = LaunchConfiguration('mode').perform(context).strip().lower()
    if mode not in ('sim', 'real'):
        raise RuntimeError("mode must be 'sim' or 'real', got '%s'" % mode)
    sim = mode == 'sim'
    use_sim_time = 'true' if sim else 'false'
    store = LaunchConfiguration('store').perform(context)
    robot_arg = LaunchConfiguration('robot_name').perform(context)   # '' = default
    nm = names_for(robot_arg)
    rviz = LaunchConfiguration('rviz').perform(context)
    timeout = float(LaunchConfiguration('wait_timeout').perform(context))

    # --- start pose (single source of truth) --------------------------------
    start_cfg = LaunchConfiguration('start_config').perform(context)
    with open(start_cfg, 'r', encoding='utf-8') as f:
        pose = yaml.safe_load(f)[nm.robot]['start_pose']
    home_name = ''
    if 'pose' in pose:
        # named destination from waypoints.yaml (e.g. standby_01)
        home_name = str(pose['pose'])    # the robot returns to where it starts
        named = load_site().resolve(pose['pose'])
        x, y, yaw = named.x, named.y, named.yaw_rad
    else:
        x, y = float(pose['x']), float(pose['y'])
        yaw = math.radians(float(pose['yaw_deg']))

    use_start = LaunchConfiguration('use_start_pose').perform(context)
    use_start = use_start.strip().lower()
    if use_start == 'auto':
        use_start = 'true' if sim else 'false'

    # --- package shares ------------------------------------------------------
    bringup = get_package_share_directory('serving_robot_bringup')
    gazebo = get_package_share_directory('serving_robot_gazebo')
    nav = get_package_share_directory('serving_robot_navigation')

    def include(path, arguments):
        return IncludeLaunchDescription(
            PythonLaunchDescriptionSource(path),
            launch_arguments=arguments.items())

    # --- stages --------------------------------------------------------------
    steps = []

    # world:=false = join a Gazebo world that is already running (2nd robot)
    with_world = LaunchConfiguration('world').perform(context).lower() == 'true'
    if sim and with_world:
        steps.append({
            'title': '[operation_mode] Gazebo world',
            # 'world' here is true/false (start the world?). gazebo.launch.py has its own
            # 'world' = file name, which would inherit that 'true': pass the file explicitly.
            'actions': [include(
                os.path.join(gazebo, 'launch', 'gazebo.launch.py'),
                {'world': 'cafe.world.sdf'})],
            'wait': {
                'name': 'gazebo', 'args': ['--gz-topic', '/clock'],
                'timeout': timeout,
                'ok': '[operation_mode] Gazebo is up',
                'fail': 'Gazebo did not start (check the Gazebo window/log '
                        'and leftover gz processes: ps aux | grep gz)',
            },
        })

    robot_wait = ['--tf', 'odom', 'base_footprint',
                  '--topic', nm.topic('scan')]
    if sim:
        robot_wait = ['--clock'] + robot_wait
    steps.append({
        'title': '[operation_mode] robot + bridge' if sim
        else '[operation_mode] robot base',
        'actions': [include(
            os.path.join(bringup, 'launch', 'robot_base.launch.py'),
            {'mode': mode, 'spawn_x': str(x), 'spawn_y': str(y),
             'spawn_yaw': str(yaw), 'robot_name': nm.robot})],
        'wait': {
            'name': 'robot', 'args': robot_wait, 'timeout': timeout,
            'ok': '[operation_mode] robot is publishing scan/odom/TF',
            'fail': 'robot did not publish /clock, %s and '
                    'odom->base_footprint (spawn failed? bridge running?)'
                    % nm.topic('scan'),
        },
    })

    # Other robots than robot_01: same YAMLs with the topic names replaced.
    renamed_loc, renamed_nav = {}, {}
    if nm.robot != 'robot_01':
        renamed_loc['amcl_params_file'] = _renamed(
            os.path.join(nav, 'config', 'amcl.yaml'), nm.robot)
        renamed_nav['nav2_params_file'] = _renamed(
            os.path.join(nav, 'config', 'nav2.yaml'), nm.robot)
        renamed_nav['collision_monitor_params_file'] = _renamed(
            os.path.join(nav, 'config', 'collision_monitor.yaml'), nm.robot)

    localization_actions = [include(
        os.path.join(nav, 'launch', 'localization.launch.py'),
        {
            'use_sim_time': use_sim_time,
            'store': store,
            'rviz': 'false',
            'initial_x': str(x),
            'initial_y': str(y),
            'initial_yaw': str(yaw),
            'set_initial_pose': use_start,
            **renamed_loc,
        })]
    rviz_node = Node(
        package='rviz2', executable='rviz2', name='rviz2', output='log',
        arguments=['-d', os.path.join(nav, 'rviz', 'navigation.rviz')],
        parameters=[{'use_sim_time': sim}],
    )
    want_rviz = rviz.lower() == 'true'
    # With an automatic start pose nobody needs RViz before Nav2 is up, so it
    # starts last. Without one (real robot) it must be there for 2D Pose Estimate.
    rviz_last = want_rviz and use_start == 'true'
    if want_rviz and not rviz_last:
        localization_actions.append(rviz_node)

    # Without a start pose (real robot) the person sets it in RViz
    # ("2D Pose Estimate"), so allow a long wait.
    map_timeout = timeout if use_start == 'true' else max(timeout, 600.0)
    map_fail = 'map->odom never appeared (AMCL has no pose)'
    if use_start != 'true':
        map_fail += '; set the pose in RViz with "2D Pose Estimate"'
    steps.append({
        'title': '[operation_mode] localization (map_server + AMCL)'
        + ('' if rviz_last or not want_rviz else ' + RViz')
        + ('' if use_start == 'true' else
           ' - set the pose in RViz: "2D Pose Estimate"'),
        'actions': localization_actions,
        'wait': {
            'name': 'localization', 'args': ['--tf', 'map', 'odom'],
            'timeout': map_timeout,
            'ok': '[operation_mode] localization is running (map->odom)',
            'fail': map_fail,
        },
    })

    steps.append({
        'title': '[operation_mode] Nav2',
        'actions': [include(
            os.path.join(nav, 'launch', 'navigation.launch.py'),
            {'use_sim_time': use_sim_time, 'store': store,
             'localization': 'false', 'rviz': 'false',
             'delivery_limit_s': LaunchConfiguration('delivery_limit_s').perform(context),
             'auto_confirm_s': LaunchConfiguration('auto_confirm_s').perform(context),
             'person_limit': LaunchConfiguration('person_limit').perform(context),
             'robot_name': robot_arg, 'home_name': home_name,
             **renamed_nav})],
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
            'ok': '[operation_mode] READY - send a goal, e.g. '
                  'ros2 run serving_robot_navigation go_to.py table_01  '
                  '(or RViz "2D Goal Pose")',
            'fail': 'Nav2 nodes did not become active (see the Nav2 log above)',
        },
    })

    if rviz_last:
        steps.append({
            'title': '[operation_mode] RViz (started last)',
            'actions': [rviz_node],
        })

    return _chain(steps)


def generate_launch_description():
    bringup = get_package_share_directory('serving_robot_bringup')
    return LaunchDescription([
        DeclareLaunchArgument(
            'mode', default_value='sim',
            description='sim = Gazebo, real = physical robot'),
        DeclareLaunchArgument(
            'store', default_value='demo_store',
            description='Store map directory name under maps/stores'),
        DeclareLaunchArgument(
            'rviz', default_value='true',
            description='Start RViz2 (true/false)'),
        DeclareLaunchArgument(
            'start_config',
            default_value=os.path.join(
                bringup, 'config', 'common', 'robot_start.yaml'),
            description='Robot start pose file'),
        DeclareLaunchArgument(
            'use_start_pose', default_value='auto',
            description='true/false/auto (auto: true in sim, false on real)'),
        DeclareLaunchArgument(
            'wait_timeout', default_value='180',
            description='Seconds each stage may take before giving up'),
        DeclareLaunchArgument(
            'delivery_limit_s', default_value='',
            description='test only: delivery time limit in s (empty = yaml)'),
        DeclareLaunchArgument(
            'robot_name', default_value='',
            description='robot name for the topic names (empty = default_robot of '
                        'config/common/interfaces.yaml)'),
        DeclareLaunchArgument(
            'world', default_value='true',
            description='sim: start the Gazebo world. false = the world is '
                        'already running (second robot, own ROS_DOMAIN_ID)'),
        DeclareLaunchArgument(
            'person_limit', default_value='true',
            description='person state limits the speed (detector + limiter); false = off'),
        DeclareLaunchArgument(
            'auto_confirm_s', default_value='',
            description='test only: confirm load/handover after n s (empty = off)'),
        OpaqueFunction(function=_build),
    ])
