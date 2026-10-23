from types import SimpleNamespace as NS
from pathlib import Path
import sys,json,time
import pytest,yaml
ROOT=Path(__file__).resolve().parents[2]
sys.path.insert(0,str(ROOT/"ros_ws/src/serving_robot_navigation"))
from serving_robot_navigation.navigation_executor import NavigationExecutor as E
def fixture():
 h=NS(limits=yaml.safe_load((ROOT/"config/common/runtime_limits.yaml").read_text()),record={"status":"RUNNING"},state="NAVIGATING",motion_recovery_pending=False,motion_recovery_ready=None,rotation_escape_active=False,pending=False,forward_bt="forward.xml",fine=False,precision_pending=False,request_started=0.,progress_anchor=None,progress_at=0.,odom=None,canceling=False,rotation_status={"reason":"CUMULATIVE_ROTATION_LIMIT"},dynamic={"sensor_fresh":True,"hold_navigation":False},target=(1.,2.,0.))
 h.rotation_status_wall=time.monotonic();h.t=1.;h.now=lambda:h.t;h.events=[];h.emit=lambda name,**kw:h.events.append((name,kw))
 h.sent=[];h.send_navigation=lambda **kw:h.sent.append(kw);h.ledger=NS(save=lambda:None,data={})
 h.out=[];h.fine_pub=NS(publish=h.out.append);h.fine_active=NS(publish=h.out.append)
 h.nav=NS(server_is_ready=lambda:True);h.cancels=[];h.goal_handle=NS(cancel_goal_async=lambda:h.cancels.append("cancel"))
 h.failures=[];h.finish=lambda *a:h.failures.append(a)
 h.begin_rotation_recovery=lambda reason:E.begin_rotation_recovery(h,reason)
 return h
def test_rotation_recovery_cancels_once_and_never_sends_over_active_goal():
 h=fixture();E.motion_watchdog(h,(0,0,0));E.motion_watchdog(h,(0,0,0))
 assert h.cancels==["cancel"] and not h.sent and h.record["rotation_recovery_attempts"]==1
 # Even after the wait, an unconfirmed old Goal prevents a replacement.
 h.t=8.;E.motion_watchdog(h,(0,0,0));assert not h.sent
 h.goal_handle=None;h.motion_recovery_ready=10.
 h.t=9.9;E.motion_watchdog(h,(0,0,0));assert not h.sent
 h.t=10.;E.motion_watchdog(h,(0,0,0));assert h.sent==[{"force_forward":True}]
 assert h.state=="NAVIGATING" and h.rotation_escape_active
def test_latched_guard_does_not_cancel_the_escape_goal_in_a_loop():
 h=fixture();h.rotation_escape_active=True;h.record["feedback_time_s"]=1.;h.record["status_time_s"]=1.
 E.motion_watchdog(h,(0,0,0));assert not h.cancels and not h.sent
def test_retry_budget_and_goal_deadline_are_real():
 h=fixture();h.record["rotation_recovery_attempts"]=h.limits["navigation"]["goal_retry_max"]
 E.motion_watchdog(h,(0,0,0));assert h.failures and not h.cancels
 h=fixture();h.t=h.limits["navigation"]["goal_timeout_s"]+.1
 E.motion_watchdog(h,(0,0,0));assert "goal_timeout_s" in h.failures[-1][1]
def test_unknown_acceptance_is_not_republished():
 import time
 h=fixture();h.pending=True;h.accept_sent_wall=time.monotonic()-4.
 E.motion_watchdog(h,(0,0,0));assert h.failures and not h.sent
def test_error_reset_requires_inactive_action_and_does_not_send_a_goal():
 h=fixture();h.state="ERROR";h.recovering=False;h.pose=lambda:(0,0,0);h.task="old";h.record["status"]="FAILED"
 E.command(h,NS(data=json.dumps({"cmd":"reset"})));assert h.state=="ERROR"
 h.goal_handle=None;E.command(h,NS(data=json.dumps({"cmd":"reset"})))
 assert h.state=="IDLE" and not h.sent
 assert h.rotation_status["reason"]=="CUMULATIVE_ROTATION_LIMIT"
