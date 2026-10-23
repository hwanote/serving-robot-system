"""Pure domain policies. No ROS dependencies; tested independently."""
import math
from dataclasses import dataclass, field

ROBOT_IDS = ('robot_01', 'robot_02')
STATES = ('IDLE','GOING_PICKUP','WAIT_LOAD','GOING_TABLE','GOING_DIRECT',
          'WAIT_HANDOVER','RETURNING_HOME','GOING_KITCHEN','RECOVERY_WAIT','COMPLETED','CANCELED','ERROR')
ACTIVE = {'GOING_PICKUP','WAIT_LOAD','GOING_TABLE','GOING_DIRECT','WAIT_HANDOVER','RETURNING_HOME','GOING_KITCHEN','RECOVERY_WAIT'}
MAX_SPEED_MPS=0.25
MAX_YAW_RPS=0.80
GOAL_NAV_XY_M=0.10
ARRIVAL_DETECT_M=0.10
FINAL_PHYSICAL_M=0.10
PERSON_SLOW_M=1.5
PERSON_STOP_M=0.8
STATIONARY_PERSON_REPLAN_S=20.0


def norm_angle(a):
    return math.atan2(math.sin(a), math.cos(a))


def distance(a,b):
    return math.hypot(a[0]-b[0], a[1]-b[1])


def velocity_limit(v, w, max_speed=MAX_SPEED_MPS, max_yaw=MAX_YAW_RPS):
    if not all(math.isfinite(float(x)) for x in (v,w,max_speed,max_yaw)):return 0.0,0.0
    return max(0.0,min(max_speed,float(v))), max(-max_yaw,min(max_yaw,float(w)))


def person_factor(distance_m, stop_m=PERSON_STOP_M, slow_m=PERSON_SLOW_M):
    if distance_m is None: return 1.0
    if distance_m <= stop_m: return 0.0
    if distance_m >= slow_m: return 1.0
    return (distance_m-stop_m)/(slow_m-stop_m)


def may_confirm(command, state, mode):
    return ((command=='LOAD' and state=='WAIT_LOAD' and mode=='KITCHEN_PICKUP') or
            (command=='COMPLETE' and state=='WAIT_HANDOVER' and mode in ('KITCHEN_PICKUP','DELIVER_DIRECT')) or
            (command=='COLLECTED' and state=='WAIT_HANDOVER' and mode=='TABLE_COLLECT'))


def next_after_nav(state,mode):
    if state=='GOING_PICKUP' and mode=='KITCHEN_PICKUP':return 'WAIT_LOAD'
    if state in ('GOING_TABLE','GOING_DIRECT'):return 'WAIT_HANDOVER'
    if state in ('RETURNING_HOME','GOING_KITCHEN'):return 'COMPLETED'
    raise ValueError('Unexpected successful goal in state '+state)


@dataclass
class Task:
    request_id:str
    table:str
    robot:str
    mode:str='KITCHEN_PICKUP'
    state:str='QUEUED'
    side:str='auto'


@dataclass
class FleetCore:
    tasks:dict=field(default_factory=dict)
    queue:list=field(default_factory=list)
    active:dict=field(default_factory=dict)

    def submit(self,rid,table,robot,mode='KITCHEN_PICKUP',side='auto'):
        if not rid or not isinstance(rid,str): raise ValueError('Request ID required')
        if rid in self.tasks:
            old=self.tasks[rid]
            if (old.table,old.robot,old.mode,old.side)!=(table,robot,mode,side):
                raise ValueError('request_id already assigned to different mission')
            return old,False
        if robot not in ROBOT_IDS:raise ValueError('Unknown robot: '+str(robot))
        if mode not in ('KITCHEN_PICKUP','DELIVER_DIRECT','TABLE_COLLECT','GO_KITCHEN','GO_HOME'):
            raise ValueError('Unknown mission mode')
        task=Task(rid,table,robot,mode,side=side)
        self.tasks[rid]=task
        self.queue.append(rid)
        return task,True

    def dispatch_next(self,available, kitchen_free=True):
        for rid in list(self.queue):
            t=self.tasks[rid]
            if t.robot not in available or t.robot in self.active:continue
            if t.mode in ('KITCHEN_PICKUP','GO_KITCHEN') and not kitchen_free:continue
            self.queue.remove(rid); self.active[t.robot]=rid; t.state='ASSIGNED'
            return t
        return None

    def update(self,robot,state,request_id=None):
        rid=self.active.get(robot)
        if not rid or rid!=request_id:return
        t=self.tasks[rid]
        t.state=({'COMPLETED':'SUCCEEDED','CANCELED':'CANCELED','ERROR':'FAILED'}
                 .get(state,'RUNNING'))
        if state in ('COMPLETED','CANCELED','ERROR'):
            del self.active[robot]

    def snapshot(self):
        return {'active':dict(self.active),'queued':list(self.queue),
                'requests':{k:vars(v).copy() for k,v in self.tasks.items()}}


# Conservative fleet-owned zone grant. A robot already inside a zone keeps its lease
# until exiting. Robots approaching within 0.45 m queue behind its current owner.
ZONES={'kitchen_access':(-3.75,-0.90,0.72,2.35),
       'central_lane':(-0.60,0.60,-2.45,2.20)}


def in_bounds(xy,b,margin=0.0):
    return xy is not None and b[0]-margin<=xy[0]<=b[1]+margin and b[2]-margin<=xy[1]<=b[3]+margin


@dataclass
class ZoneArbiter:
    owners:dict=field(default_factory=dict)
    zones:dict=field(default_factory=lambda:dict(ZONES))
    def update(self,poses,margin=0.45):
        hold=set()
        for zone,b in self.zones.items():
            inside=[r for r in ROBOT_IDS if in_bounds(poses.get(r),b)]
            candidates=[r for r in ROBOT_IDS if in_bounds(poses.get(r),b,margin)]
            owner=self.owners.get(zone)
            # Keep current occupant if still inside, otherwise grant to oldest approach.
            if owner not in inside:
                owner=(inside or candidates or [None])[0]
            if owner is None:self.owners.pop(zone,None)
            else:self.owners[zone]=owner
            hold.update(r for r in candidates if owner and r!=owner)
        return hold
