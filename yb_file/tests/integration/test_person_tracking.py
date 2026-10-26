from pathlib import Path
from types import SimpleNamespace
import sys
root=Path(__file__).resolve().parents[2]
sys.path.insert(0,str(root/'ros_ws/src/cafe_autonomy'))
from cafe_autonomy.safety import Safety
from geometry_msgs.msg import PoseArray,Pose
import cafe_autonomy.safety as safety

def gate():
    return SimpleNamespace(people_last={},track_anchors={},track_velocity={},track_time={},next_track=0,
        people_stationary={},replan_fired=set(),safe_cfg={'track_association_m':.75,'stationary_displacement_m':.08},
        perception_fresh=True,people=[],people_stamp=0.)

def frame(points):
    message=PoseArray();message.header.frame_id='map'
    for x,y in points:
        p=Pose();p.position.x=float(x);p.position.y=float(y);message.poses.append(p)
    return message

def test_slow_walk_is_not_a_twenty_second_stationary_episode(monkeypatch):
    clock=[0.];monkeypatch.setattr(safety.time,'monotonic',lambda:clock[0]);g=gate()
    for t in range(31):
        clock[0]=float(t);Safety.on_people(g,frame([(t*.02,0.)]))
    assert clock[0]-g.people_stationary[0]<6.
    assert g.track_velocity[0]>.019

def test_detection_order_does_not_swap_stationary_timers(monkeypatch):
    clock=[0.];monkeypatch.setattr(safety.time,'monotonic',lambda:clock[0]);g=gate()
    Safety.on_people(g,frame([(0.,0.),(2.,0.)]));clock[0]=25.
    Safety.on_people(g,frame([(2.,0.),(0.,0.)]))
    assert g.people_last[0]==(0.,0.) and g.people_last[1]==(2.,0.)
    assert all(t==0. for t in g.people_stationary.values())

def test_invalid_person_coordinates_fail_closed():
    g=gate();Safety.on_people(g,frame([(float('nan'),0.)]));assert not g.perception_fresh
