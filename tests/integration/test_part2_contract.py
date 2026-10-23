from pathlib import Path
import sys,json,math,hashlib,subprocess
import pytest,yaml
ROOT=Path(__file__).resolve().parents[2]
sys.path.insert(0,str(ROOT/"ros_ws/src/serving_robot_navigation"))
from serving_robot_navigation.semantic_inputs import destinations,zone_ratio,validate_inputs
from serving_robot_navigation.request_ledger import RequestLedger
from serving_robot_navigation.rotation_guard import RotationGuard

def test_semantic_graph_coordinates_are_read_only(tmp_path):
    p=tmp_path/"waypoints.yaml";p.write_text(yaml.safe_dump({"nodes":{"a":{"x":1.2,"y":-.7}},"poses":{"desk":{"node":"a","yaw_deg":90,"aliases":["pickup"]}}}))
    before=p.read_bytes();result=destinations(p)
    assert result["desk"]==(1.2,-.7,math.pi/2)
    assert result["pickup"]==result["desk"] and p.read_bytes()==before

def test_empty_part1_inputs_refuse_navigation(tmp_path):
    p=tmp_path/"map.yaml";p.write_text("")
    with pytest.raises(ValueError,match="Part 1"):validate_inputs(p,p,p,p)

def test_exclusive_requires_current_grant():
    z=[dict(id="door",type="exclusive",bounds=[0,1,0,1])]
    assert zone_ratio(z,.5,.5)==0
    assert zone_ratio(z,.5,.5,["door"])==1
    assert zone_ratio([dict(id="counter",type="no_go",bounds=[0,1,0,1])],.5,.5)==0

def test_request_identity_survives_restart_and_conflicts(tmp_path):
    p=tmp_path/"requests.json";a=RequestLedger(p)
    rec,new=a.accept("one",{"destination_id":"desk"});assert new
    rec["goal_uuid"]=[1]*16;rec["goal_publish_count"]=1;a.save()
    b=RequestLedger(p);rec,new=b.accept("one",{"destination_id":"desk"})
    assert not new and rec["goal_publish_count"]==1 and rec["goal_uuid"]==[1]*16
    with pytest.raises(ValueError,match="conflict"):b.accept("one",{"destination_id":"other"})
    with pytest.raises(ValueError,match="busy"):b.accept("two",{"destination_id":"desk"})

def test_rotation_reset_rules():
    cfg=yaml.safe_load((ROOT/"config/common/runtime_limits.yaml").read_text())["navigation"]
    g=RotationGuard(cfg)
    for i in range(171):g.observe(0,0,math.radians(i),.5,i*.1)
    assert g.reason=="CUMULATIVE_ROTATION_LIMIT"
    for i in range(10):g.observe(0,0,math.radians(170-i),-.5,18+i*.1)
    assert g.reason=="CUMULATIVE_ROTATION_LIMIT"
    g.observe(.081,0,math.radians(161),0.,20)
    assert g.total==0 and g.reason is None

def test_part1_and_other_parts_identical_to_develop():
    paths=["maps","web","deploy","config/common/entities.yaml","config/common/waypoints.yaml","config/common/zones.yaml","config/common/store.yaml","ros_ws/src/serving_robot_mapping","ros_ws/src/serving_robot_description","ros_ws/src/serving_robot_gazebo"]
    assert subprocess.run(["git","diff","--quiet","origin/develop","--",*paths],cwd=ROOT).returncode==0
    assert not (ROOT/"ros_ws/src/cafe_autonomy").exists()

def test_sensor_payload_numpy_types_are_not_a_node_crash():
    import numpy as np
    from serving_robot_navigation.serialization import dumps
    result=json.loads(dumps({"risk":np.bool_(True),"speed":np.float64(.2),"point":np.array([1.,2.])}))
    assert result=={"risk":True,"speed":.2,"point":[1.,2.]}

def test_forward_recovery_keeps_the_normal_mppi_and_forbids_reverse():
    data=yaml.safe_load((ROOT/"ros_ws/src/serving_robot_navigation/config/nav2.yaml").read_text())
    c=data["/robot_01/controller_server"]["ros__parameters"]
    assert c["FollowPath"]["plugin"]=="nav2_mppi_controller::MPPIController"
    assert c["FollowPath"]["vx_min"]==0.0
    assert c["ForwardController"]["allow_reversing"] is False
    assert c["ForwardController"]["use_rotate_to_heading"] is False
