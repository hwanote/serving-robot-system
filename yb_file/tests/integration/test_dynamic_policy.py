from pathlib import Path
import yaml,sys,math
root=Path(__file__).resolve().parents[2];sys.path.insert(0,str(root/'ros_ws/src/cafe_autonomy'))
from cafe_autonomy.dynamic_policy import DynamicPolicy,Track,Tracks

def config():return yaml.safe_load((root/'config/integration/runtime_limits.yaml').read_text())['dynamic_obstacles']
def fixed(x=2.,y=0.,radius=.2):
    policy=DynamicPolicy(config());t=Track('human','person',x,y,radius,0.,0.,stopped_since=0.)
    policy.tracks.items[t.id]=t;return policy,t
def step(p,t,now,front=2.,path=None):
    t.last_seen=now
    return p.update(now,(0.,0.,0.),0.,path or [(0.,0.),(4.,0.)],front,True)

def test_five_meters_tracks_but_does_not_stop():
    p,t=fixed(4.8);result=step(p,t,0.)
    assert result['stage']=='TRACKING_5M' and not result['stop'] and result['speed_ratio']==1.
    t.x=5.3;assert not step(p,t,1.)['tracks']

def test_three_meters_prepares_and_slows_without_becoming_a_stop_distance():
    p,t=fixed(2.8,1.5);r=step(p,t,0.)
    assert r['stage']=='RESPONSE_3M' and r['speed_ratio']<1. and not r['stop'] and not r['hold_navigation']

def test_approaching_object_stops_before_contact():
    p,t=fixed(.5,1.,.2);t.moving=True;t.stopped_since=None;t.vy=-1.
    r=p.update(1.,(0.,0.,0.),.25,[(0.,0.),(4.,0.)],2.,True)
    assert r['stop'] and r['stage']=='COLLISION_PREDICTED_STOP'
    assert r['tracks'][0]['distance_m']>.8

def test_observe_three_seconds_does_not_resume_a_blocked_path():
    p,t=fixed();assert step(p,t,0.)['stage']=='OBSERVE_3S'
    assert step(p,t,2.99)['stage']=='OBSERVE_3S'
    r=step(p,t,3.);assert r['stage']=='WAIT_BLOCKED' and r['hold_navigation']

def test_check_at_twenty_cannot_execute_detour_before_thirty():
    p,t=fixed();step(p,t,0.)
    events=step(p,t,15.)['events'];assert any(e['event']=='BLOCKED_ALERT' for e in events)
    events=step(p,t,20.)['events'];assert [e['event'] for e in events]==['REPLAN_CHECK']
    assert not any(e['event']=='DYNAMIC_DETOUR_REQUEST' for e in step(p,t,29.99)['events'])
    events=step(p,t,30.)['events'];assert any(e['event']=='DYNAMIC_DETOUR_REQUEST' for e in events)

def test_twenty_second_preview_cannot_clear_original_blocking_timer():
    p,t=fixed();step(p,t,0.)
    preview=[(0.,0.),(0.,1.),(4.,1.),(4.,0.)]
    r=step(p,t,20.,path=preview);assert r['hold_navigation'] and t.blocked_since==0.
    assert any(e['event']=='DYNAMIC_DETOUR_REQUEST' for e in step(p,t,30.,path=preview)['events'])

def test_no_path_retains_stop_and_retries_at_configured_interval():
    p,t=fixed();step(p,t,0.);step(p,t,30.);p.detour_result(['human'],'NO_PATH',30.)
    r=step(p,t,31.);assert r['hold_navigation'] and r['stage']=='NO_PATH_SAFE_WAIT'
    assert not r['events'];assert any(e['event']=='DYNAMIC_DETOUR_REQUEST' for e in step(p,t,35.)['events'])

def test_contact_lock_requires_both_three_seconds_and_two_seconds_clearance():
    p,t=fixed(4.,2.);p.contact(True,0.);assert step(p,t,0.,front=.4)['contact_latched']
    p.contact(False,1.)
    assert step(p,t,3.,front=.4)['contact_latched']
    assert step(p,t,4.,front=1.2)['contact_latched']
    assert step(p,t,5.99,front=1.2)['contact_latched']
    assert not step(p,t,6.,front=1.2)['contact_latched']

def test_noncontact_recovery_cannot_time_out_and_skip_clearance():
    p,t=fixed(.5,0.,.2);assert step(p,t,0.,front=.4)['stop']
    t.x=4.;t.y=2.
    assert step(p,t,300.,front=.5)['stop']
    assert step(p,t,301.,front=1.2)['stop']
    assert not step(p,t,303.,front=1.2)['stop']

def test_slow_resumed_motion_resets_stationary_and_blocking_times():
    store=Tracks(config())
    for i in range(30):store.observe([(2.,0.,.2,'person')],i*.1,'RGBD')
    track=next(iter(store.items.values()));assert track.stopped_since is not None
    for i in range(1,81):store.observe([(2.+i*.005,0.,.2,'person')],3.+i*.1,'RGBD')
    assert track.ever_moved and track.stopped_since is None and track.blocked_since is None


def test_same_contact_does_not_restart_three_second_observation():
    p,t=fixed(4.,2.)
    p.contact(True,0.,['body/person'])
    for now in (1.,2.,3.,4.):p.contact(True,now,['body/person'])
    r=step(p,t,4.,front=.4)
    assert r['contact_since_s']==0. and r['contact_observed_s']==4. and r['contact_active']
    assert r['stop'] and any(e['event']=='CONTACT_PATH_CHECK' for e in r['events'])
    p.contact(False,4.1);assert step(p,t,4.1,front=1.2)['stop']
    assert not step(p,t,6.2,front=1.2)['stop']


def test_side_and_rear_are_tracked_without_unnecessary_slowdown():
    for point in ((0.,1.3),(-1.5,0.)):
        p,t=fixed(*point);r=step(p,t,1.)
        assert not r['stop'] and r['speed_ratio']==1.
    p,t=fixed(0.,.6,.2)
    r=p.update(1.,(0.,0.,0.),0.,[(0.,0.),(4.,0.)],2.,False,angular_speed=.5)
    assert r['stop'] and r['tracks'][0]['rotation_risk']


def test_detour_requests_stop_after_three_attempts():
    p,t=fixed();step(p,t,0.)
    for now in (30.,35.,40.):
        assert any(e['event']=='DYNAMIC_DETOUR_REQUEST' for e in step(p,t,now)['events'])
        p.detour_result(['human'],'NO_PATH',now)
    r=step(p,t,45.)
    assert not any(e['event']=='DYNAMIC_DETOUR_REQUEST' for e in r['events'])
    assert any(e['event']=='DETOUR_EXHAUSTED' for e in r['events'])


def test_person_extent_survives_lidar_classification_dropout():
    tracks=Tracks(config());tracks.observe([(2.,0.,.25,'person')],0.,'RGBD_YOLO')
    tracks.observe([(2.,0.,.08,'unclassified_object')],.2,'LiDAR_FOREGROUND')
    track=next(iter(tracks.items.values()));assert track.label=='person' and track.radius>=.25


def test_localization_correction_does_not_make_stationary_object_move():
    tracks=Tracks(config())
    for i in range(30):
        # Map->odom is corrected by AMCL while the object stays at odom x=2.
        tx=.12*math.sin(i*.8);ty=.08*math.cos(i*.7)
        tracks.observe([(2.+tx,ty,.2,'unclassified_object')],i*.1,'LiDAR_FOREGROUND',(tx,ty,0.))
    tr=next(iter(tracks.items.values()))
    assert not tr.moving and not tr.ever_moved and tr.stopped_since is not None


def test_uncertain_registration_never_mixes_motion_coordinate_frames():
    tracks=Tracks(config())
    for i in range(30):
        valid=i%4!=0
        tracks.observe([(4.,3.,.2,'unclassified_object')],i*.1,'LiDAR_FOREGROUND',(0.,0.,0.),[(.8,.75)] if valid else None,0.,valid)
    track=next(iter(tracks.items.values()))
    assert not track.moving and not track.ever_moved
