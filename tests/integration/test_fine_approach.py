from pathlib import Path
import math,sys,yaml
ROOT=Path(__file__).resolve().parents[2]
sys.path.insert(0,str(ROOT/"ros_ws/src/serving_robot_navigation"))
from serving_robot_navigation.fine_approach import rotation_allowance,yaw_step,needs_curve_arrival,fine_handoff_allowed
C=yaml.safe_load((ROOT/"config/common/runtime_limits.yaml").read_text());N=C["navigation"];A=C["arrival"]
def status(**kw):return dict(cumulative_rad=kw.get("cumulative_rad",0.),continuous_rad=kw.get("continuous_rad",0.),direction=kw.get("direction",0),reason=kw.get("reason"))
def test_current_control_targets_are_preserved():
 assert A["control_xy_target_m"]==.04 and A["xy_tolerance_m"]==.1
 assert A["control_yaw_target_rad"]==.18 and A["yaw_tolerance_rad"]==.25
def test_no_budget_turn_request_when_remaining_allowance_is_insufficient():
 w,_,v=yaw_step(1.8,status(cumulative_rad=1.6),N,A)
 assert w==0 and v=="RECOVER"
def test_guard_reason_never_generates_a_turn():
 assert yaw_step(1.,status(reason="CUMULATIVE_ROTATION_LIMIT"),N,A)[::2]==(0.,"RECOVER")
 assert yaw_step(1.,status(reason="CONTINUOUS_ROTATION_LIMIT"),N,A)[::2]==(0.,"WAIT_GUARD")
def test_unknown_or_nonfinite_budget_is_zero():
 assert rotation_allowance({},N)==0
 assert rotation_allowance(status(cumulative_rad=float("nan")),N)==0
def test_yaw_hysteresis_prevents_stop_turn_chatter_in_acceptance_band():
 assert yaw_step(.17,status(),N,A)[1:] == (True,"ALIGNED")
 assert yaw_step(.22,status(),N,A,settling=True)==(0.,True,"ALIGNED")
 assert yaw_step(.22,status(),N,A,settling=False)[2]=="ALIGN_YAW"
 assert yaw_step(.26,status(),N,A,settling=True)[2]=="ALIGN_YAW"
def test_direction_change_only_resets_continuous_budget_not_cumulative():
 s=status(cumulative_rad=.1,continuous_rad=3.,direction=1)
 assert rotation_allowance(s,N,requested=-1)>2.
 assert rotation_allowance(s,N,requested=1)==0.
 s["cumulative_rad"]=N["cumulative_rotation_rad"];assert rotation_allowance(s,N,requested=-1)==0.
def test_near_pi_final_alignment_requests_a_curve():
 threshold=N["cumulative_rotation_rad"]-A["rotation_budget_margin_rad"]+A["control_yaw_target_rad"]
 assert needs_curve_arrival(0.,math.pi,threshold)
 assert not needs_curve_arrival(3.,-3.,threshold)


def test_forward_curve_handoff_requires_four_cm_and_measured_turn_budget():
    measured=status(cumulative_rad=.145,continuous_rad=.04,direction=1)
    assert fine_handoff_allowed(.019,-1.756,measured,N,A,True,True)
    assert not fine_handoff_allowed(.041,-1.756,measured,N,A,True,True)
    assert not fine_handoff_allowed(.019,-1.756,status(cumulative_rad=2.5),N,A,True,True)
    assert not fine_handoff_allowed(.019,-1.756,measured,N,A,True,False)
    assert fine_handoff_allowed(.019,-1.756,measured,N,A,False,True)
