"""Pure domain policies. No ROS dependencies; tested independently."""
import math
from dataclasses import dataclass, field

ROBOT_IDS = ('robot_01', 'robot_02')
STATES = ('IDLE','NAVIGATING','SERVING','RETURNING','CHARGING','PAUSED','ERROR')
ACTIVE = {'NAVIGATING','RETURNING'}
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


