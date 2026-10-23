"""One operation stack per robot/domain; never share unprefixed TF in one domain."""
from pathlib import Path
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument,ExecuteProcess,OpaqueFunction
from launch.substitutions import LaunchConfiguration

def setup(context):
    get=lambda name:LaunchConfiguration(name).perform(context)
    if get("robot_01_domain")==get("robot_02_domain"):raise ValueError("Robots require distinct ROS domains")
    nodes=[]
    for robot in ("robot_01","robot_02"):
        env=os.environ.copy();env["ROS_DOMAIN_ID"]=get(robot+"_domain")
        cmd=["ros2","launch","serving_robot_bringup","operation_mode.launch.py","robot_id:="+robot]
        for key in ("project_root","map_yaml","semantic_dir","weights","use_sim_time"):cmd.append(key+":="+get(key))
        nodes.append(ExecuteProcess(cmd=cmd,env=env,output="screen"))
    return nodes

def generate_launch_description():
    args=[DeclareLaunchArgument(k,default_value=v) for k,v in
          [("project_root",str(Path.cwd())),("map_yaml",""),("semantic_dir",""),("weights",""),("use_sim_time","true"),("robot_01_domain","0"),("robot_02_domain","2")]]
    return LaunchDescription(args+[OpaqueFunction(function=setup)])
