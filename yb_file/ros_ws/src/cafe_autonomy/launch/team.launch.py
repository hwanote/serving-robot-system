"""TEAM assets, independent TF and a single authoritative safety chain per base."""
from pathlib import Path
import math
import xml.etree.ElementTree as ET
import subprocess
import xacro
import yaml
from ament_index_python.packages import get_package_share_directory as share
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, IncludeLaunchDescription, RegisterEventHandler, LogInfo, Shutdown
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def setup(context):
    cfg=Path(LaunchConfiguration('config').perform(context))
    c=yaml.safe_load(cfg.read_text())
    mode=LaunchConfiguration('mode').perform(context)
    if mode not in ('mapping','operation'):raise ValueError('mode must be mapping or operation')
    p=Path(share('cafe_autonomy'))
    world=Path(share('serving_robot_gazebo'))/'worlds/cafe.world.sdf'
    args='-r -v3 '+str(world)
    if LaunchConfiguration('gui').perform(context)=='false':args='-s --headless-rendering '+args
    actions=[IncludeLaunchDescription(PythonLaunchDescriptionSource(str(Path(share('ros_gz_sim'))/'launch/gz_sim.launch.py')),
         launch_arguments={'gz_args':args,'on_exit_shutdown':'true'}.items()),
         Node(package='serving_robot_gazebo',executable='team_sim_clock',name='team_sim_clock')]
    loader=Node(package='cafe_autonomy',executable='contact_loader',name='contact_system_loader',output='screen')
    spawns=[]
    for r,pose in c['robots'].items():
        if mode=='mapping' and r!='robot_01':continue
        tree=ET.fromstring(xacro.process_file(str(Path(share('serving_robot_description'))/'urdf/serving_robot.urdf.xacro'),mappings={'robot_name':r}).toxml())
        for plugin in tree.findall('.//plugin'):
            if 'DiffDrive' in plugin.attrib.get('name',''):
                for tag,text in [('topic',f'/model/{r}/cmd_vel'),('odom_topic',f'/model/{r}/odometry'),('tf_topic',f'/model/{r}/tf'),('frame_id',r+'/odom'),('child_frame_id',r+'/base_footprint')]:
                    elem=plugin.find(tag)
                    if elem is None:elem=ET.SubElement(plugin,tag)
                    elem.text=text
        for el in tree.findall('.//gz_frame_id'):el.text=r+'/laser_link'
        # Add registered RGB-D cameras without replacing the TEAM chassis or LiDAR.
        for view,a in [('front',0.0),('rear_left',2.09439510239),('rear_right',-2.09439510239)]:
            link=view+'_camera_link'
            ET.SubElement(tree,'link',{'name':link})
            j=ET.SubElement(tree,'joint',{'name':view+'_camera_joint','type':'fixed'})
            ET.SubElement(j,'parent',{'link':'base_link'});ET.SubElement(j,'child',{'link':link})
            ET.SubElement(j,'origin',{'xyz':f"0.16 0 {c['perception']['mount_z']}",'rpy':f'0 0 {a}'})
            g=ET.SubElement(tree,'gazebo',{'reference':link})
            sensor=ET.SubElement(g,'sensor',{'name':view+'_rgbd','type':'rgbd_camera'})
            ET.SubElement(sensor,'always_on').text='true'
            ET.SubElement(sensor,'update_rate').text=str(c['perception']['front_hz'] if view=='front' else c['perception']['rear_hz'])
            ET.SubElement(sensor,'topic').text=f'/model/{r}/camera/{view}'
            ET.SubElement(sensor,'gz_frame_id').text=r+'/'+link
            camera=ET.SubElement(sensor,'camera')
            ET.SubElement(camera,'horizontal_fov').text=str(c['perception']['horizontal_fov'])
            image=ET.SubElement(camera,'image')
            for tag,value in [('width','320'),('height','240'),('format','R8G8B8')]:ET.SubElement(image,tag).text=value
            clip=ET.SubElement(camera,'clip');ET.SubElement(clip,'near').text='0.1';ET.SubElement(clip,'far').text='8.0'
        description=ET.tostring(tree,encoding='unicode')
        generated=cfg.parents[2]/'_local_only/generated_models';generated.mkdir(parents=True,exist_ok=True)
        urdf=generated/(r+'.urdf');urdf.write_text(description)
        converted=subprocess.run(['gz','sdf','-p',str(urdf)],capture_output=True,text=True,check=True)
        sdf=ET.fromstring(converted.stdout);model=sdf.find('model')
        body=model.find("link[@name='base_footprint']")
        collisions=[collision.attrib['name'] for collision in body.findall('collision') if any(part in collision.attrib['name'] for part in ('central_chassis_collision','front_bumper_collision','rear_bumper_collision'))]
        if len(collisions)!=3:raise ValueError('Expected the three original chassis/bumper collisions')
        spawn_description=ET.tostring(sdf,encoding='unicode');(generated/(r+'.sdf')).write_text(spawn_description)
        actions += [Node(package='robot_state_publisher',executable='robot_state_publisher',namespace=r,
                         parameters=[{'robot_description':description,'frame_prefix':r+'/','use_sim_time':True}])]
        spawns.append(Node(package='ros_gz_sim',executable='create',name='spawn_'+r,arguments=['-world','cafe','-name',r,'-allow_renaming','false','-string',spawn_description,
                         '-x',str(pose['x']),'-y',str(pose['y']),'-z','0.02','-Y',str(pose['yaw'])]))
        bridge_args=[f'/model/{r}/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
                     f'/model/{r}/odometry@nav_msgs/msg/Odometry[gz.msgs.Odometry',
                     f'/model/{r}/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist',
                     f'/world/cafe/model/{r}/joint_state@sensor_msgs/msg/JointState[gz.msgs.Model']
        remaps=[(f'/model/{r}/scan',f'/{r}/scan_raw'),(f'/model/{r}/odometry',f'/{r}/odom'),
                (f'/model/{r}/cmd_vel',f'/{r}/cmd_vel'),(f'/world/cafe/model/{r}/joint_state',f'/{r}/joint_states')]
        for index in range(3):
            src=f'/model/{r}/contacts/sensor_{index}'
            bridge_args.append(f'{src}@ros_gz_interfaces/msg/Contacts[gz.msgs.Contacts')
            remaps.append((src,f'/{r}/contacts/sensor_{index}'))
        for view in ('front','rear_left','rear_right'):
            for topic,typ,gztyp in [('image','Image','Image'),('depth_image','Image','Image'),('camera_info','CameraInfo','CameraInfo')]:
                src=f'/model/{r}/camera/{view}/{topic}'
                bridge_args.append(f'{src}@sensor_msgs/msg/{typ}[gz.msgs.{gztyp}')
                remaps.append((src,f'/{r}/camera/{view}/{topic}'))
        actions.append(Node(package='ros_gz_bridge',executable='parameter_bridge',name='bridge_'+r,arguments=bridge_args,remappings=remaps))
        def n(exe,package='cafe_autonomy',params=None,name=None,remappings=None):
            return Node(package=package,executable=exe,namespace=r,name=name or exe,parameters=params or [{'use_sim_time':True}],remappings=remappings or [],output='screen',respawn=exe in ('mission',),respawn_delay=2.)
        actions += [n('scan_reframe',params=[{'use_sim_time':True,'robot_id':r}]),n('odom_tf',params=[{'use_sim_time':True,'robot_id':r}])]
        if mode=='mapping':
            slam=yaml.safe_load((Path(share('serving_robot_navigation'))/'config/slam.yaml').read_text())['slam_toolbox']['ros__parameters']
            slam.update({'odom_frame':r+'/odom','base_frame':r+'/base_footprint','scan_topic':f'/{r}/scan','use_sim_time':True})
            actions += [n('async_slam_toolbox_node','slam_toolbox',[slam],'slam_toolbox'),
                        n('lifecycle_manager','nav2_lifecycle_manager',[{'use_sim_time':True,'autostart':True,'node_names':['slam_toolbox']}],'lifecycle_manager_mapping')]
            continue
        params=c['nav2_files'][r]
        actions.append(n('zone_masks',params=[{'use_sim_time':True,'map_yaml':c['map_yaml'],'zones_file':c['zones_file']}]))
        actions.append(n('zone_speed',params=[{'use_sim_time':True,'robot_id':r,'zones_file':c['zones_file']}]))
        actions.append(n('dynamic_obstacles',params=[{'use_sim_time':True,'robot_id':r,'runtime_limits_file':c['limits_file'],'log_directory':str(cfg.parents[2]/'_local_only/dynamic'),'static_world_file':str(world),'alignment_x':c['world_to_map']['x'],'alignment_y':c['world_to_map']['y'],'alignment_yaw':c['world_to_map']['yaw']}]))
        actions += [n('map_server','nav2_map_server',[params,{'yaml_filename':c['map_yaml']}]),
                    n('amcl','nav2_amcl',[params,{'set_initial_pose':True,'initial_pose.x':pose['map_x'],'initial_pose.y':pose['map_y'],'initial_pose.yaw':pose['map_yaw']}])]
        for exe,package in [('planner_server','nav2_planner'),('controller_server','nav2_controller'),('behavior_server','nav2_behaviors'),('bt_navigator','nav2_bt_navigator'),('velocity_smoother','nav2_velocity_smoother'),('collision_monitor','nav2_collision_monitor')]:
            ps=[params]
            if exe=='bt_navigator':ps.append({'default_nav_to_pose_bt_xml':str(p/'config/navigate_no_reverse.xml')})
            rm=[('cmd_vel','cmd_vel_nav')] if exe in ('controller_server','behavior_server') else []
            if exe=='velocity_smoother':rm=[('cmd_vel','cmd_vel_nav'),('cmd_vel_smoothed','cmd_vel_nav_smooth')]
            actions.append(n(exe,package,ps,remappings=rm))
        actions += [n('lifecycle_guard'),
                    n('semantic_safety',params=[{'use_sim_time':True,'robot_id':r,'runtime_limits_file':c['limits_file']}]),
                    n('mission',params=[{'use_sim_time':True,'robot_id':r,'waypoints_file':c['waypoints_file'],'state_file':str(cfg.parents[2]/'_local_only/missions'/(r+'.json')),'runtime_limits_file':c['limits_file']}]),
                    n('rgbd_people',params=[{'use_sim_time':True,'robot_id':r,'weights':c['perception']['weights'],'timeout_s':c['perception']['timeout_s']}])]
    if mode=='operation':
        actions += [Node(package='serving_robot_gazebo',executable='gazebo_truth_evidence',name='gazebo_truth_evidence'),
                    Node(package='cafe_autonomy',executable='fleet',name='cafe_fleet',respawn=True,respawn_delay=2.,parameters=[{'use_sim_time':True,'http_port':c['http_port'],'database_path':c['database_path'],'runtime_limits_file':c['limits_file'],'zones_file':c['zones_file']}],output='screen'),
                    Node(package='cafe_autonomy',executable='metrics',name='cafe_metrics',parameters=[{'use_sim_time':True,'log_directory':str(cfg.parents[2]/'_local_only/metrics'),'alignment_x':c['world_to_map']['x'],'alignment_y':c['world_to_map']['y'],'alignment_yaw':c['world_to_map']['yaw']}],output='screen')]
    def after_contact(event,context):
        return spawns if event.returncode==0 else [LogInfo(msg='Contact system failed; stopping bringup'),Shutdown(reason='Contact system unavailable')]
    actions += [RegisterEventHandler(OnProcessExit(target_action=loader,on_exit=after_contact)),loader]
    return actions


def generate_launch_description():
    return LaunchDescription([DeclareLaunchArgument('config'),DeclareLaunchArgument('mode',default_value='operation'),DeclareLaunchArgument('gui',default_value='true'),OpaqueFunction(function=setup)])
