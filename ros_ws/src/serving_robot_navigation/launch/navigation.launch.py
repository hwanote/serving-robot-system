"""Operation-only Part 2. Base drivers and Part 1 inputs remain external."""
from pathlib import Path
import tempfile
import yaml
from ament_index_python.packages import get_package_share_directory as share
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument,OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from serving_robot_navigation.semantic_inputs import validate_inputs
from serving_robot_navigation.runtime_config import load_limits

def setup(context):
    get=lambda name:LaunchConfiguration(name).perform(context)
    root=Path(get("project_root")).resolve();robot=get("robot_id")
    if robot not in ("robot_01","robot_02"):raise ValueError("Invalid robot_id")
    config=Path(get("semantic_dir")).resolve() if get("semantic_dir") else root/"config/common"
    map_yaml=Path(get("map_yaml")).resolve() if get("map_yaml") else root/"maps/stores/demo_store/map.yaml"
    validate_inputs(map_yaml,config/"waypoints.yaml",config/"zones.yaml",config/"entities.yaml")
    local=root/"_local_only";local.mkdir(exist_ok=True)
    weights=Path(get("weights")).resolve()
    if not weights.is_file():raise ValueError("YOLO weights missing; pass weights:=absolute_path")
    nav_share=Path(share("serving_robot_navigation"))
    source=nav_share/"config/nav2.yaml"
    data=yaml.safe_load(source.read_text().replace("/robot_01/","/"+robot+"/"))
    runtime=load_limits(root/"config/common/runtime_limits.yaml")
    controller=data["/"+robot+"/controller_server"]["ros__parameters"]
    controller["progress_checker"]["movement_time_allowance"]=runtime["navigation"]["progress_allowance_s"]
    controller["general_goal_checker"]["xy_goal_tolerance"]=min(runtime["navigation"]["goal_xy_tolerance_m"],runtime["arrival"]["control_xy_target_m"])
    controller["general_goal_checker"]["yaw_goal_tolerance"]=runtime["navigation"]["goal_yaw_tolerance_rad"]
    data["/"+robot+"/map_server"]["ros__parameters"]["yaml_filename"]=str(map_yaml)
    data["/"+robot+"/bt_navigator"]["ros__parameters"]["default_nav_to_pose_bt_xml"]=str(nav_share/"behavior_trees/navigate_no_reverse.xml")
    # Machine-local expansion only; source map and semantic coordinates are immutable.
    generated=local/("nav2_"+robot+".yaml");generated.write_text(yaml.safe_dump(data,sort_keys=False))
    limits=str(root/"config/common/runtime_limits.yaml");sim=get("use_sim_time")=="true"
    state_dir=Path(get("state_dir")).resolve() if get("state_dir") else local/"missions"
    def n(package,exe,params=None,remappings=None,name=None):
        return Node(package=package,executable=exe,name=name or exe.replace(".py",""),namespace=robot,
                    parameters=[str(generated),{"use_sim_time":sim}]+(params or []),remappings=remappings or [],output="screen")
    nodes=[]
    for exe,package in [("map_server","nav2_map_server"),("amcl","nav2_amcl"),("controller_server","nav2_controller"),
        ("planner_server","nav2_planner"),("behavior_server","nav2_behaviors"),("bt_navigator","nav2_bt_navigator"),
        ("velocity_smoother","nav2_velocity_smoother"),("collision_monitor","nav2_collision_monitor")]:
        remap=[("cmd_vel","cmd_vel_nav")] if exe in ("controller_server","behavior_server") else []
        if exe=="velocity_smoother":remap=[("cmd_vel","cmd_vel_nav"),("cmd_vel_smoothed","cmd_vel_nav_smooth")]
        nodes.append(n(package,exe,remappings=remap))
    common={"robot_id":robot,"runtime_limits_file":limits}
    nodes.extend([
        n("serving_robot_perception","person_detector",[dict(robot_id=robot,weights=str(weights),timeout_s=3.)]),
        n("serving_robot_perception","semantic_safety",[common]),
        n("serving_robot_navigation","dynamic_obstacles.py",[dict(common,static_world_file=str(root/"ros_ws/src/serving_robot_gazebo/worlds/cafe.world.sdf"),log_directory=str(local/"dynamic"))]),
        n("serving_robot_navigation","zone_masks.py",[dict(map_yaml=str(map_yaml),zones_file=str(config/"zones.yaml"))]),
        n("serving_robot_navigation","navigation_executor.py",[dict(common,waypoints_file=str(config/"waypoints.yaml"),zones_file=str(config/"zones.yaml"),state_file=str(state_dir/(robot+".json")),forward_bt=str(nav_share/"behavior_trees/navigate_forward_curves.xml"))]),
        n("nav2_lifecycle_manager","lifecycle_manager",[{"autostart":True,"node_names":["map_server","amcl","controller_server","planner_server","behavior_server","bt_navigator","velocity_smoother","collision_monitor"]}])
    ])
    return nodes

def generate_launch_description():
    # One ROS domain per robot preserves the team's unprefixed TF contract.
    return LaunchDescription([
        DeclareLaunchArgument("project_root",default_value=str(Path.cwd())),
        DeclareLaunchArgument("robot_id",default_value="robot_01"),
        DeclareLaunchArgument("map_yaml",default_value=""),
        DeclareLaunchArgument("semantic_dir",default_value=""),
        DeclareLaunchArgument("weights",default_value=""),
        DeclareLaunchArgument("state_dir",default_value=""),
        DeclareLaunchArgument("use_sim_time",default_value="true"),
        OpaqueFunction(function=setup)])
