"""Regression checks for the existing mission adapter's blockage retry contract."""
import json
import sys
from pathlib import Path
from types import SimpleNamespace

root=Path(__file__).resolve().parents[2]
sys.path.insert(0,str(root/'ros_ws/src/cafe_autonomy'))
from cafe_autonomy.dynamic_mission import DynamicMissionMixin,candidate_evidence


def mission():
    node=DynamicMissionMixin()
    node.robot='robot_01';node.state='GOING_DIRECT';node.request_id='test-request'
    node.target=(1.,2.,0.);node.dynamic_cfg={'max_detour_attempts':3}
    node.dynamic_hold=False;node.paused_nav=False;node.detour_planning=False
    node.detour_attempt_count=3;node.detour_plan_count=3;node.plan_generation=0
    node.goal_handle=None;node.goal_pending=False;node.events=[];node.published=[]
    node.emit=lambda name,**kw:node.events.append((name,kw))
    node.publish_detour=lambda state,ids,**kw:node.published.append((state,ids,kw))
    node._apply_hold=lambda:None
    node.get_clock=lambda:SimpleNamespace(now=lambda:SimpleNamespace(nanoseconds=50_000_000_000,to_msg=lambda:None))
    node.planning_client=SimpleNamespace(server_is_ready=lambda:True,
        send_goal_async=lambda goal:SimpleNamespace(add_done_callback=lambda callback:None))
    node.fail=lambda *args: (_ for _ in ()).throw(AssertionError('mission must remain active'))
    return node


def test_fourth_mission_wide_request_is_allowed_for_new_blockage():
    node=mission()
    node.request_dynamic_plan('EXECUTE',{'track_ids':['object_B'],'time_s':50.})
    assert node.detour_attempt_count==4 and node.detour_planning
    assert node.events[-1][0]=='DYNAMIC_DETOUR_PLAN_SENT'
    assert node.state=='GOING_DIRECT' and node.target==(1.,2.,0.)


def test_exhaustion_keeps_mission_and_target_in_safe_wait():
    node=mission()
    event={'robot_id':'robot_01','event':'DETOUR_EXHAUSTED',
           'track_ids':['object_B'],'episode_id':'blockage_2','attempts':3}
    node.on_dynamic_event(SimpleNamespace(data=json.dumps(event)))
    assert node.state=='GOING_DIRECT' and node.target==(1.,2.,0.)
    assert node.dynamic_hold and node.paused_nav
    assert node.events==[('DYNAMIC_DETOUR_SAFE_WAIT',
        {'track_ids':['object_B'],'episode_id':'blockage_2','failed_plans':3})]


def test_planner_unavailable_does_not_count_as_path_failure():
    node=mission();node.planning_client.server_is_ready=lambda:False
    node.request_dynamic_plan('EXECUTE',{'track_ids':['object_B']})
    assert node.detour_attempt_count==3
    assert node.published==[('CANCELED',['object_B'],{'reason':'PLANNER_NOT_READY'})]


def test_candidate_evidence_keeps_raw_path_and_both_existing_clearances():
    cfg={'robot_radius_m':.33,'detour_path_margin_m':.05,'path_margin_m':.10}
    obstacle={'x':0.,'y':0.,'radius':.20}
    first=candidate_evidence([(-1.,.62672),(1.,.62672)],[obstacle],cfg,'GridBased')
    nearby=candidate_evidence([(-1.,.62673),(1.,.62673)],[obstacle],cfg,'GridBased')
    assert first['candidate_path']==[[-1.,.62672],[1.,.62672]]
    assert first['candidate_hash']==nearby['candidate_hash']
    assert first['raw_path_sha256']!=nearby['raw_path_sha256']
    clearance=first['obstacle_clearances'][0]
    assert clearance['approval_required_m']==.5800000000000001
    assert clearance['policy_required_m']==.63
    assert clearance['approval_required_m']<clearance['minimum_clearance_m']<clearance['policy_required_m']


def test_candidate_inside_policy_margin_never_starts_follow_path():
    from action_msgs.msg import GoalStatus
    node=mission()
    node.dynamic_cfg.update(robot_radius_m=.33,detour_path_margin_m=.05,path_margin_m=.10)
    node.dynamic_report={'blocker_ids':['obstacle']}
    node.failed_detour_paths=set()
    started=[]
    node.start_follow_path=lambda *args,**kwargs:started.append((args,kwargs))
    poses=[SimpleNamespace(pose=SimpleNamespace(position=SimpleNamespace(x=x,y=.62672)))
           for x in (-1.,1.)]
    path=SimpleNamespace(poses=poses)
    result=SimpleNamespace(status=GoalStatus.STATUS_SUCCEEDED,
                           result=SimpleNamespace(path=path,error_code=0))
    node.dynamic_plan_result(SimpleNamespace(result=lambda:result),0,'EXECUTE',
                             {'track_ids':['obstacle'],
                              'obstacles':[{'x':0.,'y':0.,'radius':.20}]},'GridBased')
    assert not started
    state,ids,details=node.published[-1]
    assert state=='NO_PATH' and details['reason']=='PATH_STILL_INTERSECTS_BLOCKER'
    assert details['minimum_clearance_m']<details['required_m']
