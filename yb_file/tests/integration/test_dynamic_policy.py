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


def test_only_failed_plan_results_count_and_third_plan_is_sent():
    p,t=fixed();step(p,t,0.)
    first=step(p,t,30.)
    assert first['events'][-1]['attempt']==1 and p.attempts[t.id]==0
    assert not any(e['event']=='DYNAMIC_DETOUR_REQUEST' for e in step(p,t,35.)['events'])
    p.detour_result([t.id],'CANCELED',35.)
    assert p.attempts[t.id]==0
    for now,attempt in ((40.,1),(45.,2),(50.,3)):
        event=next(e for e in step(p,t,now)['events'] if e['event']=='DYNAMIC_DETOUR_REQUEST')
        assert event['attempt']==attempt
        p.detour_result([t.id],'NO_PATH',now)
        assert p.attempts[t.id]==attempt
    result=step(p,t,55.)
    assert result['stage']=='DETOUR_EXHAUSTED' and result['hold_navigation']
    assert not any(e['event']=='DYNAMIC_DETOUR_REQUEST' for e in result['events'])
    assert [e['event'] for e in result['events']]==['DETOUR_EXHAUSTED']
    assert not step(p,t,60.)['events']


def test_planned_path_does_not_reset_failure_count_before_physical_clearance():
    p,t=fixed();step(p,t,0.);step(p,t,30.)
    p.detour_result([t.id],'EXECUTING',30.)
    assert p.attempts[t.id]==0 and p.episodes[p.track_episode[t.id]]['active']
    p.detour_result([t.id],'NO_PATH',31.)
    assert p.attempts[t.id]==1
    assert next(e for e in step(p,t,36.)['events'] if e['event']=='DYNAMIC_DETOUR_REQUEST')['attempt']==2


def test_lidar_id_change_keeps_same_blockage_failure_history():
    p,t=fixed();step(p,t,0.)
    for now in (30.,35.):
        step(p,t,now);p.detour_result([t.id],'NO_PATH',now)
    old_episode=p.track_episode[t.id]
    del p.tracks.items[t.id]
    replacement=Track('new_lidar_id','person',2.,0.,.2,36.,36.,stopped_since=0.)
    p.tracks.items[replacement.id]=replacement
    result=step(p,replacement,40.)
    request=next(e for e in result['events'] if e['event']=='DYNAMIC_DETOUR_REQUEST')
    assert request['attempt']==3 and request['episode_id']==old_episode
    assert p.attempts[replacement.id]==2
    p.detour_result([replacement.id],'NO_PATH',40.)
    assert step(p,replacement,45.)['stage']=='DETOUR_EXHAUSTED'


def test_separate_blockage_does_not_inherit_failure_count():
    p,a=fixed();step(p,a,0.)
    for now in (30.,35.):
        step(p,a,now);p.detour_result([a.id],'NO_PATH',now)
    b=Track('second','person',3.2,0.,.2,36.,36.,stopped_since=0.)
    p.tracks.items[b.id]=b
    step(p,b,40.)
    result=step(p,b,70.)
    request=next(e for e in result['events'] if e['event']=='DYNAMIC_DETOUR_REQUEST' and b.id in e['track_ids'])
    assert request['attempt']==1 and request['episode_id']!=p.track_episode[a.id]
    assert p.attempts[b.id]==0


def test_blockage_clearance_releases_exhausted_wait():
    p,t=fixed();step(p,t,0.)
    for now in (30.,35.,40.):
        step(p,t,now);p.detour_result([t.id],'NO_PATH',now)
    assert step(p,t,45.)['stage']=='DETOUR_EXHAUSTED'
    t.moving=True;t.stopped_since=None;t.y=1.5
    step(p,t,46.)
    result=step(p,t,48.)
    assert not result['hold_navigation'] and p.track_episode.get(t.id) is None


def test_verified_track_removal_clears_failure_history_after_clearance():
    p,t=fixed();step(p,t,0.)
    for now in (30.,35.,40.):
        step(p,t,now);p.detour_result([t.id],'NO_PATH',now)
    episode_id=p.track_episode[t.id]
    del p.tracks.items[t.id]  # Adapter does this only after verified_clear().
    p.update(41.,(0.,0.,0.),0.,[(0.,0.),(4.,0.)],2.,True)
    assert not p.episodes[episode_id]['active'] and t.id not in p.attempts


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


def test_stationary_confirmation_keeps_first_blocked_path_after_nav_replan():
    p=DynamicPolicy(config())
    t=Track('object_2','unclassified_object',1.,0.,.2,0.,0.)
    p.tracks.items[t.id]=t
    straight=[(0.,0.),(2.,0.)]
    bypass=[(0.,0.),(0.,1.),(2.,1.),(2.,0.)]
    assert p.update(0.,(0.,0.,0.),0.,straight,2.,True)['tracks'][0]['blocks_path']
    assert t.blocked_path==straight
    t.stopped_since=0.
    assert p.update(1.,(0.,0.,0.),0.,bypass,2.,True)['tracks'][0]['blocks_path']
    assert t.blocked_path==straight
    result=p.update(31.,(0.,0.,0.),0.,bypass,2.,True)
    assert result['tracks'][0]['blocks_path']
    assert any(e['event']=='DYNAMIC_DETOUR_REQUEST' for e in result['events'])


def test_closest_approach_numpy_velocity_keeps_safety_report_json_serializable():
    import json
    import numpy as np
    from cafe_autonomy.dynamic_policy import closest_approach

    point=(.45,0.)
    velocity=(np.float64(-.1),np.float64(0.))
    ttc,clearance=closest_approach(point,velocity,(0.,0.,0.),.25,3.)
    assert type(ttc) is float
    assert type(clearance) is float

    policy=DynamicPolicy(config())
    track=Track('object_1','unclassified_object',*point,.2,0.,0.,stopped_since=0.)
    track.vx,track.vy=velocity
    policy.tracks.items[track.id]=track
    report=policy.update(31.,(0.,0.,0.),.25,[(0.,0.),(3.,0.)],2.,True)
    assert report['stop']
    assert type(report['stop']) is bool
    json.dumps(report)


def test_same_path_blocking_stop_distance_for_person_and_moving_object():
    for label in ('person', 'unclassified_object'):
        policy = DynamicPolicy(config())
        track = Track('target', label, 1.05, 0., .2, 0., 0., moving=True)
        policy.tracks.items[track.id] = track
        outside = policy.update(0., (0., 0., 0.), 0., [(0., 0.), (3., 0.)], 2., True)
        assert not outside['stop']
        track.x = .95
        inside = policy.update(2., (0., 0., 0.), 0., [(0., 0.), (3., 0.)], 2., True)
        assert inside['stop']


def test_planned_route_clear_releases_old_risk_below_fixed_front_clearance():
    p,t=fixed(.5,0.,.2)
    path=[(0.,0.),(4.,0.)]
    assert p.update(0.,(0.,0.,0.),0.,path,.6,True,planned_path=True)['stop']
    t.x=2.;t.y=1.5;t.moving=True;t.stopped_since=None
    assert p.update(2.,(0.,0.,0.),0.,path,.9,True,planned_path=True)['stop']
    result=p.update(4.,(0.,0.,0.),0.,path,.9,True,planned_path=True)
    assert not result['stop'] and result['clearance_confirmed']
    assert result['front_clear_m']==.9 and result['blocker_ids']==[]


def test_no_plan_keeps_old_clearance_latch_below_one_meter():
    p,t=fixed(.5,0.,.2)
    path=[(0.,0.),(4.,0.)]
    assert p.update(0.,(0.,0.,0.),0.,path,.6,True)['stop']
    t.x=2.;t.y=1.5;t.moving=True;t.stopped_since=None
    assert p.update(4.,(0.,0.,0.),0.,path,.9,True)['stop']


def test_same_blockage_track_id_change_does_not_repeat_stop_events_within_thirty_seconds():
    p,t=fixed();step(p,t,0.)
    assert [e['event'] for e in step(p,t,20.)['events']]==['BLOCKED_ALERT','REPLAN_CHECK']
    episode_id=p.track_episode[t.id]
    del p.tracks.items[t.id]
    replacement=Track('replacement','person',2.,0.,.2,21.,21.,stopped_since=0.)
    p.tracks.items[replacement.id]=replacement
    report=step(p,replacement,22.)
    assert p.track_episode[replacement.id]==episode_id
    assert report['hold_navigation']
    assert not [e for e in report['events'] if e['event'] in ('BLOCKED_ALERT','REPLAN_CHECK')]
