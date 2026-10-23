"""Common moving-object policy. Geometry is always in the shared map frame.

Sensor freshness uses wall time in the ROS adapter; all policy times use the ROS
clock (simulation seconds in Gazebo). No simulator truth enters this module.
"""
from collections import deque
from dataclasses import dataclass,field
import math


def distance(a,b):return math.hypot(a[0]-b[0],a[1]-b[1])


def segment_distance(p,a,b):
    dx=b[0]-a[0];dy=b[1]-a[1];den=dx*dx+dy*dy
    t=max(0.,min(1.,((p[0]-a[0])*dx+(p[1]-a[1])*dy)/den)) if den>1e-9 else 0.
    return math.hypot(p[0]-a[0]-t*dx,p[1]-a[1]-t*dy)


def path_blocks(point,radius,robot,path,cfg):
    if len(path)<2:
        dx=point[0]-robot[0];dy=point[1]-robot[1]
        forward=dx*math.cos(robot[2])+dy*math.sin(robot[2]);side=-dx*math.sin(robot[2])+dy*math.cos(robot[2])
        return 0.<forward<cfg['path_lookahead_m'] and abs(side)<cfg['robot_radius_m']+radius+cfg['path_margin_m']
    nearest=min(range(len(path)),key=lambda i:distance(robot,path[i]))
    travelled=0.
    for a,b in zip(path[nearest:],path[nearest+1:]):
        if segment_distance(point,a,b)<cfg['robot_radius_m']+radius+cfg['path_margin_m']:return True
        travelled+=distance(a,b)
        if travelled>cfg['path_lookahead_m']:break
    return False


def closest_approach(point,velocity,robot,robot_speed,horizon):
    rx,ry=point[0]-robot[0],point[1]-robot[1]
    vx=velocity[0]-robot_speed*math.cos(robot[2]);vy=velocity[1]-robot_speed*math.sin(robot[2])
    den=vx*vx+vy*vy
    t=max(0.,min(horizon,-(rx*vx+ry*vy)/den)) if den>1e-8 else 0.
    return t,math.hypot(rx+t*vx,ry+t*vy)


@dataclass
class Track:
    id:str
    label:str
    x:float
    y:float
    radius:float
    first_seen:float
    last_seen:float
    vx:float=0.
    vy:float=0.
    moving:bool=False
    ever_moved:bool=False
    stopped_since:float|None=None
    blocked_since:float|None=None
    low_motion_since:float|None=None
    history:deque=field(default_factory=deque)
    emitted:set=field(default_factory=set)
    sources:set=field(default_factory=set)
    uncertain:bool=False
    blocked_path:list=field(default_factory=list)
    stationary_anchor:tuple|None=None

    def snapshot(self,now):
        return {'id':self.id,'class':self.label,'x':self.x,'y':self.y,'vx':self.vx,'vy':self.vy,
                'speed_mps':math.hypot(self.vx,self.vy),'heading_rad':math.atan2(self.vy,self.vx),
                'radius_m':self.radius,'first_seen_s':self.first_seen,'last_seen_s':self.last_seen,
                'moving':self.moving,'motion_state':('MOVING' if self.moving else 'STATIONARY' if self.stopped_since is not None else 'UNKNOWN'),'ever_moved':self.ever_moved,'stopped_since_s':self.stopped_since,
                'stationary_for_s':max(0.,now-self.stopped_since) if self.stopped_since is not None else 0.,
                'blocked_since_s':self.blocked_since,'sources':sorted(self.sources),'uncertain':self.uncertain}


class Tracks:
    def __init__(self,cfg):self.cfg=cfg;self.items={};self.serial=0
    def observe(self,observations,now,source,motion_frame=None,motion_positions=None,motion_yaw_override=None,motion_valid=True):
        """Observations: x,y,radius,label. Batch association survives detection order changes."""
        unused=set(self.items)
        for observation_index,(x,y,radius,label) in enumerate(observations):
            if not all(math.isfinite(v) for v in (x,y,radius)) or radius<0:continue
            choices=[key for key in unused if (label=='robot')==(self.items[key].label=='robot') and distance((x,y),(self.items[key].x,self.items[key].y))<self.cfg['association_m']]
            key=min(choices,key=lambda k:distance((x,y),(self.items[k].x,self.items[k].y))) if choices else None
            if key is None:
                self.serial+=1;key='object_'+str(self.serial)
                tr=Track(key,label,x,y,radius,now,now);self.items[key]=tr
            else:tr=self.items[key];unused.remove(key)
            if motion_frame is None:mx,my=x,y;motion_yaw=0.
            else:
                tx,ty,motion_yaw=motion_frame;dx=x-tx;dy=y-ty
                mx=dx*math.cos(motion_yaw)+dy*math.sin(motion_yaw);my=-dx*math.sin(motion_yaw)+dy*math.cos(motion_yaw)
            if motion_positions is not None:
                mx,my=motion_positions[observation_index];motion_yaw=motion_yaw_override or 0.
            previous_time=tr.last_seen
            tr.sources.add(source);tr.uncertain=False
            if label=='person' or tr.label!='person':tr.label=label
            if now-previous_time>self.cfg['lost_timeout_s']:
                tr.history.clear();tr.moving=False;tr.stopped_since=None;tr.blocked_since=None;tr.emitted.clear()
            tr.x=x;tr.y=y;tr.radius=max(radius,tr.radius*.8,self.cfg['person_radius_m'] if tr.label=='person' else 0.);tr.last_seen=now
            if not motion_valid:tr.uncertain=True;continue
            # RGB and LiDAR can update the same track at the same stamp.
            if not tr.history or now-tr.history[-1][0]>=self.cfg['history_sample_s']:
                tr.history.append((now,mx,my))
            while len(tr.history)>2 and now-tr.history[0][0]>self.cfg['velocity_window_s']:tr.history.popleft()
            span=now-tr.history[0][0] if tr.history else 0.
            if span>=self.cfg['motion_confirm_s']:
                dx=(mx-tr.history[0][1])/max(span,1e-6);dy=(my-tr.history[0][2])/max(span,1e-6)
                vx=dx*math.cos(motion_yaw)-dy*math.sin(motion_yaw);vy=dx*math.sin(motion_yaw)+dy*math.cos(motion_yaw)
                tr.vx=.5*tr.vx+.5*vx;tr.vy=.5*tr.vy+.5*vy
                speed=math.hypot(tr.vx,tr.vy)
                if speed>=self.cfg['moving_speed_mps']:
                    tr.moving=True;tr.ever_moved=True;tr.stopped_since=None;tr.blocked_since=None;tr.blocked_path=[];tr.low_motion_since=None;tr.emitted.clear()
                elif speed<=self.cfg['stationary_speed_mps']:
                    if tr.low_motion_since is None:tr.low_motion_since=now
                    if now-tr.low_motion_since>=self.cfg['motion_confirm_s']:
                        if tr.stopped_since is None:tr.stopped_since=tr.low_motion_since;tr.stationary_anchor=(mx,my)
                        tr.moving=False
                else:tr.low_motion_since=None
            if tr.stopped_since is not None and tr.stationary_anchor is not None and distance((mx,my),tr.stationary_anchor)>self.cfg['stationary_displacement_m']:
                tr.moving=True;tr.ever_moved=True;tr.stopped_since=None;tr.blocked_since=None;tr.blocked_path=[];tr.low_motion_since=None;tr.emitted.clear()
    def age(self,now,verified_clear):
        for key,tr in list(self.items.items()):
            age=now-tr.last_seen
            if age>self.cfg['lost_timeout_s']:
                if verified_clear(tr):del self.items[key]
                else:tr.uncertain=True


class DynamicPolicy:
    def __init__(self,cfg):
        self.cfg=cfg;self.tracks=Tracks(cfg);self.contact_latched=False;self.contact_active=False;self.contact_since=None;self.contact_ids=set();self.contact_check_sent=False;self.attempts={};self.exhausted=set()
        self.last_risk=-1e9;self.clear_since=None;self.detours={};self.no_path=set();self.last_request={};self.recovery_latched=False
    def contact(self,active,now,identities=()):
        if active:
            if not self.contact_latched:
                self.contact_since=now;self.contact_check_sent=False
            self.contact_latched=True;self.contact_ids.update(identities);self.clear_since=None
        self.contact_active=bool(active)
    def detour_result(self,ids,state,now):
        for key in ids:
            if state=='EXECUTING':self.detours[key]=now;self.no_path.discard(key)
            elif state=='NO_PATH':self.detours.pop(key,None);self.no_path.add(key);self.last_request[key]=now
            elif state in ('FAILED','CANCELED','COMPLETED'):self.detours.pop(key,None)
    def update(self,now,robot,speed,path,front_clear,active=True,angular_speed=0.):
        cfg=self.cfg;events=[];states=[];snapshots=[];hold=False;risk=False;ratio=1.;blockers=[];nearest=None;front_person=None
        if front_clear>=cfg['front_clear_m']:
            if self.clear_since is None:self.clear_since=now
        else:self.clear_since=None
        clearance_ok=self.clear_since is not None and now-self.clear_since>=cfg['front_clear_hold_s']
        contact_elapsed=max(0.,now-self.contact_since) if self.contact_since is not None else 0.
        if self.contact_latched and contact_elapsed>=cfg['contact_observe_s']:
            if not self.contact_active and clearance_ok:
                self.contact_latched=False;self.contact_since=None;self.contact_ids.clear()
                events.append({'event':'CONTACT_RELEASED','time_s':now})
            elif self.contact_active and not self.contact_check_sent:
                self.contact_check_sent=True
                events.append({'event':'CONTACT_PATH_CHECK','time_s':now,'track_ids':[],'contact_ids':sorted(self.contact_ids)})
        for key,tr in self.tracks.items.items():
            centre=distance(robot,(tr.x,tr.y));d=max(0.,centre-tr.radius)
            if d>cfg['recognition_m']:continue
            nearest=d if nearest is None else min(nearest,d)
            bearing=math.atan2(math.sin(math.atan2(tr.y-robot[1],tr.x-robot[0])-robot[2]),math.cos(math.atan2(tr.y-robot[1],tr.x-robot[0])-robot[2]))
            in_front=abs(bearing)<=cfg['front_response_half_angle_rad']
            if in_front and tr.label=='person':front_person=d if front_person is None else min(front_person,d)
            corridor=tr.blocked_path if tr.blocked_path and key not in self.detours else path
            blocked=active and path_blocks((tr.x,tr.y),tr.radius,robot,corridor,cfg) and d<=cfg['response_m']
            if blocked and not tr.blocked_path:tr.blocked_path=list(path)
            ttc,cpa=closest_approach((tr.x,tr.y),(tr.vx,tr.vy),robot,speed,cfg['prediction_horizon_s'])
            collision=(cpa<cfg['robot_radius_m']+tr.radius+cfg['prediction_margin_m'] and ttc<=cfg['ttc_stop_s'] and (ttc>0. or centre<cfg['robot_radius_m']+tr.radius+cfg['hard_stop_clearance_m']))
            # Existing person STOP is kept separate from the 3m response range.
            stop_distance=cfg['person_stop_m'] if tr.label=='person' else cfg['object_stop_m']
            rotation_risk=abs(angular_speed)>.02 and (centre<cfg['robot_radius_m']+tr.radius+cfg['prediction_margin_m'] or cpa<cfg['robot_radius_m']+tr.radius+cfg['prediction_margin_m'] and ttc<=cfg['rotation_prediction_s'])
            collision=collision or rotation_risk or (in_front and blocked and d<=stop_distance)
            risk=risk or collision
            state='TRACKING_5M'
            if in_front and d<=cfg['response_m']:
                state='RESPONSE_3M';ratio=min(ratio,cfg['response_speed_ratio'])
            if tr.moving:
                tr.blocked_since=None;tr.emitted.clear();self.detours.pop(key,None);self.no_path.discard(key);self.attempts.pop(key,None);self.exhausted.discard(key)
                if collision:state='YIELD_MOVING';hold=True
            elif tr.stopped_since is not None and blocked:
                if tr.blocked_since is None:
                    tr.blocked_since=max(tr.stopped_since,now);tr.blocked_path=list(path)
                elapsed=now-tr.stopped_since;blocked_for=now-tr.blocked_since
                blockers.append(key)
                if elapsed<cfg['observe_stopped_s']:state='OBSERVE_3S';hold=True
                elif key in self.detours:state='DETOUR_EXECUTING'
                else:
                    state='NO_PATH_SAFE_WAIT' if key in self.no_path else 'WAIT_BLOCKED';hold=True
                    for name,threshold in [('BLOCKED_ALERT',cfg['blocked_alert_s']),('REPLAN_CHECK',cfg['replan_check_s'])]:
                        if elapsed>=threshold and name not in tr.emitted:
                            tr.emitted.add(name);events.append({'event':name,'track_ids':[key],'time_s':now,'stationary_s':elapsed,'blocked_s':blocked_for})
                    if not tr.uncertain and elapsed>=cfg['detour_after_s'] and blocked_for>=cfg['detour_after_s']:
                        if self.attempts.get(key,0)>=cfg['max_detour_attempts']:
                            state='DETOUR_EXHAUSTED'
                            if key not in self.exhausted:
                                self.exhausted.add(key);events.append({'event':'DETOUR_EXHAUSTED','track_ids':[key],'time_s':now,'attempts':self.attempts[key]})
                        elif key not in tr.emitted or now-self.last_request.get(key,-1e9)>=cfg['detour_retry_s']:
                            self.attempts[key]=self.attempts.get(key,0)+1
                            self.last_request[key]=now;tr.emitted.add(key)
                            state='DETOUR_REQUESTED'
                            events.append({'event':'DYNAMIC_DETOUR_REQUEST','track_ids':[key],'time_s':now,'stationary_s':elapsed,'blocked_s':blocked_for,
                                           'attempt':self.attempts[key],'obstacles':[{'x':tr.x,'y':tr.y,'radius':tr.radius}]})
            elif not blocked:
                tr.blocked_since=None
                for flag in ('BLOCKED_ALERT','REPLAN_CHECK',key):tr.emitted.discard(flag)
            if tr.uncertain and blocked:state='TRACK_UNCERTAIN';hold=True
            item=tr.snapshot(now);item.update({'distance_m':d,'blocks_path':blocked,'in_front':in_front,'rotation_risk':rotation_risk,'ttc_s':ttc,'closest_approach_m':cpa,'stage':state});snapshots.append(item);states.append(state)
        if risk:self.last_risk=now;self.recovery_latched=True
        risk_lock=(now-self.last_risk<cfg['noncontact_pause_s'])
        if not risk and not risk_lock and clearance_ok:self.recovery_latched=False
        # A stopped blocking episode cannot resume just because its 3s timer expired.
        clear_hold=self.recovery_latched and not clearance_ok
        turn_only=bool(self.detours) and clear_hold and not (risk or risk_lock or self.contact_latched)
        stop=risk or risk_lock or self.contact_latched or (clear_hold and not turn_only)
        hold=hold or stop
        if self.contact_latched:stage='CONTACT_OBSERVE' if contact_elapsed<cfg['contact_observe_s'] else 'CONTACT_WAIT'
        elif risk:stage='COLLISION_PREDICTED_STOP'
        elif risk_lock or clear_hold:stage='CLEARANCE_CONFIRM'
        elif states:
            order=['DETOUR_EXHAUSTED','TRACK_UNCERTAIN','NO_PATH_SAFE_WAIT','DETOUR_REQUESTED','OBSERVE_3S','WAIT_BLOCKED','YIELD_MOVING','DETOUR_EXECUTING','RESPONSE_3M','TRACKING_5M']
            stage=min(states,key=lambda s:order.index(s))
        else:stage='CLEAR'
        return {'time_s':now,'stage':stage,'stop':stop,'linear_stop':bool(turn_only),'hold_navigation':bool(hold and active),'speed_ratio':ratio,
                'nearest_object_m':nearest,'front_person_m':front_person,'contact_latched':self.contact_latched,'contact_active':self.contact_active,'contact_since_s':self.contact_since,'contact_observed_s':contact_elapsed,'detour_attempts':dict(self.attempts),'front_clear_m':front_clear,
                'clearance_confirmed':clearance_ok,'blocker_ids':blockers,'tracks':snapshots,'events':events}
