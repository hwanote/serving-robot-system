"""Selected guard-budget ideas from the supplied Claude proposal.
The current executor retains Nav2 XY approach and yaw-only fine control.
No reverse command, guard reset, or alternative safety publisher exists here.
"""
import math

def wrap(angle):return math.atan2(math.sin(angle),math.cos(angle))

def rotation_allowance(status,nav_cfg,requested=0.,margin=.10):
    if not status or status.get("reason"):return 0.
    try:
        cumulative=float(status["cumulative_rad"]);continuous=float(status["continuous_rad"])
        if not all(math.isfinite(v) and v>=0 for v in (cumulative,continuous)):return 0.
        same=not requested or not status.get("direction") or (requested>0)==(status["direction"]>0)
        # RotationGuard resets its continuous counter when direction changes.
        used=continuous if same and status.get("direction") else 0.
        return max(0.,min(nav_cfg["cumulative_rotation_rad"]-cumulative,nav_cfg["continuous_rotation_rad"]-used)-margin)
    except (KeyError,TypeError,ValueError):return 0.

def needs_curve_arrival(bearing_to_target,target_yaw,threshold_rad):
    return abs(wrap(target_yaw-bearing_to_target))>=threshold_rad

def yaw_step(error,status,nav_cfg,arrival,settling=False):
    """Existing .18 rad control band enters settling; .25 acceptance band exits it."""
    angle=wrap(error);enter=arrival["control_yaw_target_rad"];exit=arrival["yaw_tolerance_rad"]
    if abs(angle)<=enter or settling and abs(angle)<=exit:
        return 0.,True,"ALIGNED"
    if status.get("reason")=="CONTINUOUS_ROTATION_LIMIT":return 0.,False,"WAIT_GUARD"
    allowance=rotation_allowance(status,nav_cfg,angle,arrival["rotation_budget_margin_rad"])
    if status.get("reason") or max(0.,abs(angle)-enter)>allowance:return 0.,False,"RECOVER"
    return max(-.25,min(.25,angle)),False,"ALIGN_YAW"


def fine_handoff_allowed(distance_m,yaw_error,status,nav_cfg,arrival,forward_mode=False,guard_fresh=True):
    if distance_m>arrival["control_xy_target_m"]:return False
    if not forward_mode:return True
    if not guard_fresh:return False
    needed=max(0.,abs(wrap(yaw_error))-arrival["control_yaw_target_rad"])
    return needed<=rotation_allowance(status,nav_cfg,yaw_error,arrival["rotation_budget_margin_rad"])
