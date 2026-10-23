"""Part 2 destination-only executor. No orders, serving workflow, Fleet or HTTP."""
import hashlib,json,math,time
from pathlib import Path
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.time import Time
from rclpy.qos import QoSProfile,DurabilityPolicy,ReliabilityPolicy
from action_msgs.msg import GoalStatusArray
from geometry_msgs.msg import PoseStamped,Twist
from nav_msgs.msg import Odometry
from nav2_msgs.action import NavigateToPose,FollowPath,ComputePathToPose
from std_msgs.msg import String,Bool,Float64
from unique_identifier_msgs.msg import UUID
from tf2_ros import Buffer,TransformListener
from .request_ledger import RequestLedger
from .semantic_inputs import destinations,zones,zone_ratio
from .runtime_config import load_limits
from .dynamic_policy import segment_distance
from .fine_approach import yaw_step,needs_curve_arrival,fine_handoff_allowed

class NavigationExecutor(Node):
    def __init__(self):
        super().__init__("navigation_executor")
        d=self.declare_parameter
        self.robot=d("robot_id","robot_01").value
        self.limits=load_limits(d("runtime_limits_file","").value)
        self.waypoints=destinations(d("waypoints_file","").value)
        self.zones=zones(d("zones_file","").value)
        self.ledger=RequestLedger(d("state_file","").value)
        self.forward_bt=d("forward_bt","").value;self.rotation_status={};self.motion_recovery_pending=False;self.motion_recovery_ready=None;self.rotation_escape_active=False;self.request_started=None;self.progress_anchor=None;self.progress_at=None;self.forward_mode=False;self.rotation_status_wall=0.;self.yaw_settling=False;self.guard_wait_started=None
        self.nav=ActionClient(self,NavigateToPose,"navigate_to_pose")
        self.follow=ActionClient(self,FollowPath,"follow_path")
        self.planner=ActionClient(self,ComputePathToPose,"compute_path_to_pose")
        self.tf=Buffer();self.listener=TransformListener(self.tf,self)
        self.status_pub=self.create_publisher(String,"navigation_status",10)
        self.public_pub=self.create_publisher(String,"mission_status",10)
        self.hold_pub=self.create_publisher(Bool,"traffic_hold",10)
        self.zone_pub=self.create_publisher(Float64,"zone_speed_ratio",10)
        self.detour_pub=self.create_publisher(String,"detour_state",10)
        self.events=self.create_publisher(String,"navigation_event",20)
        self.fine_pub=self.create_publisher(Twist,"cmd_vel_fine",10)
        self.fine_active=self.create_publisher(Bool,"fine_active",10)
        self.create_subscription(String,"mission_cmd",self.command,10)
        self.create_subscription(String,"dynamic_event",self.dynamic_event,20)
        self.create_subscription(String,"dynamic_safety",self.dynamic_status,10)
        self.create_subscription(String,"safety_status",self.safety_status,10)
        self.create_subscription(String,"zone_permission",self.permissions,10)
        self.create_subscription(Odometry,"odom",self.on_odom,10)
        qos=QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL,reliability=ReliabilityPolicy.RELIABLE)
        for kind,action in (("navigate_to_pose",NavigateToPose),("follow_path",FollowPath)):
            self.create_subscription(GoalStatusArray,kind+"/_action/status",lambda m,k=kind:self.action_status(m,k),qos)
            self.create_subscription(action.Impl.FeedbackMessage,kind+"/_action/feedback",lambda m,k=kind:self.feedback(m,k),10)
        self.state="IDLE";self.goal_handle=None;self.pending=False;self.canceling=False
        self.replacement=None;self.planning=False;self.plan_at=0.;self.plan_generation=0;self.terminal_pending=None
        self.fine=False;self.precision_pending=False;self.fine_since=0.;self.settled=None;self.odom=None
        self.dynamic={};self.grants=[];self.grant_time=0.;self.record=None;self.task=None
        self.target=None;self.last_pose=None;self.last_pose_time=0.;self.last_odom_time=0.
        self.attempt_ids=[];self.detour_origin=None;self.motion_logged=False
        active=self.ledger.data["active"]
        if active:
            self.task=active;self.record=self.ledger.data["requests"][active]
            self.target=self.waypoints[self.record["command"]["destination_id"]]
            self.state="PAUSED";self.recover_started=time.monotonic();self.recovering=True
        else:
            self.recovering=False
            if self.ledger.data.get("error_latched"):self.state="ERROR"
        self.create_timer(.05,self.tick)
    def emit(self,event,**data):
        self.events.publish(String(data=json.dumps(dict(event=event,task_id=self.task,robot_id=self.robot,time_s=self.now(),wall_time=time.time(),**data))))
    def now(self):return self.get_clock().now().nanoseconds*1e-9
    def pose(self):
        try:
            t=self.tf.lookup_transform("map","base_footprint",Time());q=t.transform.rotation
            stamp=t.header.stamp.sec+t.header.stamp.nanosec*1e-9
            if self.now()-stamp>self.limits["safety"]["scan_timeout_s"]:return None
            return (t.transform.translation.x,t.transform.translation.y,math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z)))
        except Exception:return None
    def safety_status(self,m):
        try:self.rotation_status=json.loads(m.data).get("rotation",{});self.rotation_status_wall=time.monotonic()
        except (ValueError,AttributeError):self.rotation_status={}
    def on_odom(self,m):self.odom=m;self.last_odom_time=time.monotonic()
    def permissions(self,m):
        try:self.grants=json.loads(m.data)["granted"];self.grant_time=time.monotonic()
        except (ValueError,KeyError,TypeError):self.grants=[]
    def dynamic_status(self,m):
        try:
            value=json.loads(m.data);self.dynamic=value if isinstance(value,dict) else {}
        except ValueError:self.dynamic={}
    def feedback(self,m,kind):
        if self.record and self.record.get("goal_kind")==kind and list(m.goal_id.uuid)==self.record.get("goal_uuid"):
            self.record["feedback_time_s"]=self.now()
    def command(self,m):
        cmd={}
        try:
            cmd=json.loads(m.data)
            if not isinstance(cmd,dict):raise ValueError("Command must be an object")
            if cmd.get("cmd")=="reset":
                if self.state!="ERROR" or self.goal_handle or self.pending or self.recovering:raise ValueError("Reset requires verified inactive failed action")
                if not self.pose() or self.dynamic.get("sensor_fresh") is not True or not self.nav.server_is_ready():raise ValueError("Reset requires localization, safety sensors and Nav2")
                self.state="IDLE";self.canceling=False;self.motion_recovery_pending=False;self.motion_recovery_ready=None
                self.ledger.data["error_latched"]=False;self.ledger.save()
                self.emit("ERROR_RESET_ACCEPTED",automatic_goal_sent=False);return
            if cmd.get("cmd")=="cancel":
                if str(cmd.get("task_id") or cmd.get("id"))!=self.task:raise ValueError("Cancel ID mismatch")
                if self.record and self.record.get("status")!="RUNNING":
                    self.emit("CANCEL_DUPLICATE_IGNORED");return
                self.canceling=True;self.motion_recovery_pending=False;self.motion_recovery_ready=None;self.replacement=None;self.plan_generation+=1;self.planning=False
                self.fine=False;self.fine_pub.publish(Twist())
                if self.goal_handle:self.goal_handle.cancel_goal_async()
                elif not self.pending:self.finish("CANCELED")
                return
            if cmd.get("cmd","go") not in ("go","navigate"):raise ValueError("Only destination navigation/cancel belongs to Part 2")
            name=cmd.get("destination_id")
            if not isinstance(name,str) or name not in self.waypoints:raise ValueError("Unknown Part 1 destination_id")
            if any(k in cmd for k in ("x","y","yaw","pose")):raise ValueError("Coordinates are not accepted")
            request_id=cmd.get("task_id") or cmd.get("id")
            if not isinstance(request_id,str) or not request_id:raise ValueError("Stable task_id is required")
            normalized={"destination_id":name}
            _,old=self.ledger.check(request_id,normalized)
            if old:self.emit("DUPLICATE_REQUEST",request_id=request_id,status=old["status"]);return
            if self.state!="IDLE" or self.goal_handle or self.pending or not self.pose() or not self.nav.server_is_ready():raise ValueError("Navigation not ready")
            if self.dynamic.get("sensor_fresh") is not True:raise ValueError("Safety sensors are not ready")
            self.record,_=self.ledger.accept(request_id,normalized);self.task=request_id;self.target=self.waypoints[name]
            self.state="NAVIGATING";self.canceling=False;self.yaw_settling=False;self.guard_wait_started=None;self.rotation_escape_active=False;self.request_started=self.now();self.progress_anchor=None;self.progress_at=self.now();self.send_navigation()
        except (ValueError,TypeError,KeyError) as e:self.emit("REQUEST_REJECTED",request_id=cmd.get("task_id"),reason=str(e))
    def pose_msg(self):
        msg=PoseStamped();msg.header.frame_id="map";msg.header.stamp=self.get_clock().now().to_msg()
        msg.pose.position.x=self.target[0];msg.pose.position.y=self.target[1]
        msg.pose.orientation.z=math.sin(self.target[2]/2);msg.pose.orientation.w=math.cos(self.target[2]/2)
        return msg
    def send_navigation(self,force_forward=False):
        goal=NavigateToPose.Goal();goal.pose=self.pose_msg();self.forward_mode=force_forward
        if force_forward:goal.behavior_tree=self.forward_bt
        here=self.pose()
        remaining=self.limits["navigation"]["cumulative_rotation_rad"]-self.rotation_status.get("cumulative_rad",0.)
        if here:
            heading=math.atan2(self.target[1]-here[1],self.target[0]-here[0])-here[2]
            turn=abs(math.atan2(math.sin(heading),math.cos(heading)))
            final_curve=needs_curve_arrival(math.atan2(self.target[1]-here[1],self.target[0]-here[0]),self.target[2],self.limits["navigation"]["cumulative_rotation_rad"]-self.limits["arrival"]["rotation_budget_margin_rad"]+self.limits["arrival"]["control_yaw_target_rad"])
            if self.forward_bt and (self.rotation_status.get("reason") or turn+.15>=remaining or final_curve):
                self.forward_mode=True;goal.behavior_tree=self.forward_bt;self.emit("FORWARD_CURVE_PLANNER_SELECTED",remaining_rotation_rad=remaining)

        # No implicit HOME or serving stages. All coordinates come from Part 1.
        self.send(self.nav,goal,"navigate_to_pose")
    def send(self,client,goal,kind):
        if self.pending or self.goal_handle:raise RuntimeError("Previous action not verified inactive")
        import uuid
        gid=UUID(uuid=list(uuid.uuid4().bytes))
        self.record.update(goal_uuid=[int(v) for v in gid.uuid],goal_kind=kind)
        self.record["goal_publish_count"]+=1;self.ledger.save()
        self.pending=True;self.accept_sent_wall=time.monotonic();self.record["sent_time_s"]=self.now();self.record["feedback_time_s"]=self.now();self.record["status_time_s"]=self.now();self.emit("GOAL_SENT",goal_uuid=bytes(gid.uuid).hex(),kind=kind,count=self.record["goal_publish_count"])
        def accepted(f):
            self.pending=False
            try:
                h=f.result()
                if not h or not h.accepted:self.finish("FAILED","Goal rejected");return
                self.goal_handle=h;self.emit("GOAL_ACCEPTED")
                if self.canceling:h.cancel_goal_async()
                h.get_result_async().add_done_callback(self.result)
            except Exception as e:self.finish("FAILED","Acceptance unknown; no resend: "+str(e))
        client.send_goal_async(goal,goal_uuid=gid).add_done_callback(accepted)
    def action_status(self,m,kind):
        if not self.record or kind!=self.record.get("goal_kind"):return
        for status in m.status_list:
            if list(status.goal_info.goal_id.uuid)!=self.record.get("goal_uuid"):continue
            self.record["status_time_s"]=self.now()
            if not self.recovering:return
            if status.status in (1,2):
                self.recovering=False;self.state="NAVIGATING";self.emit("EXISTING_GOAL_REATTACHED")
                service_type=(NavigateToPose if kind=="navigate_to_pose" else FollowPath).Impl.GetResultService
                self.recovery_client=self.create_client(service_type,kind+"/_action/get_result")
                req=service_type.Request();req.goal_id=UUID(uuid=self.record["goal_uuid"])
                self.recovery_client.call_async(req).add_done_callback(self.result)
            elif status.status in (4,5,6):self.finish("FAILED","Previous goal terminal; operator must submit a new explicit request")
            return
    def result(self,f):
        self.goal_handle=None
        try:status=f.result().status
        except Exception as e:self.finish("FAILED",str(e));return
        if self.terminal_pending:
            status,reason=self.terminal_pending;self.terminal_pending=None
            self.finish(status,reason);return
        if self.canceling:
            self.finish("CANCELED");return
        if self.motion_recovery_pending:
            if status not in (4,5,6):self.motion_recovery_pending=False;self.finish("FAILED","Original action terminal state unresolved");return
            self.motion_recovery_ready=self.now()+self.limits["navigation"]["goal_recovery_wait_s"]
            self.emit("ROTATION_RECOVERY_CANCEL_CONFIRMED",previous_status=status);return
        if self.replacement:
            path,ids=self.replacement;self.replacement=None
            self.execute_path(path,ids);return
        if self.precision_pending:
            self.precision_pending=False
            if status in (4,5):
                self.fine=True;self.fine_since=self.now();self.settled=None;self.yaw_settling=False;self.guard_wait_started=None;self.emit("PRECISION_HANDOFF_CONFIRMED");return
            self.finish("FAILED","Precision handoff action failed");return
        if status!=4:
            if self.record and self.record.get("path_hash"):
                self.record["failed_paths"].append(self.record["path_hash"]);self.ledger.save()
            self.detour("NO_PATH",self.attempt_ids,reason="Navigation action did not succeed")
            self.finish("FAILED","Nav2 status "+str(status));return
        self.fine=True;self.fine_since=self.now();self.settled=None;self.yaw_settling=False;self.guard_wait_started=None
        self.detour("COMPLETED",self.attempt_ids)
    def begin_rotation_recovery(self,reason):
        if self.motion_recovery_pending or self.pending:return
        attempts=self.record.get("rotation_recovery_attempts",0)
        if attempts>=self.limits["navigation"]["goal_retry_max"]:
            self.finish("FAILED","Rotation recovery retry budget exhausted");return
        if not self.forward_bt:self.finish("FAILED","Forward recovery tree unavailable");return
        self.record["rotation_recovery_attempts"]=attempts+1;self.ledger.save()
        self.motion_recovery_pending=True;self.rotation_escape_active=True;self.state="PAUSED"
        self.fine=False;self.precision_pending=False;self.fine_pub.publish(Twist());self.fine_active.publish(Bool(data=False))
        self.emit("ROTATION_RECOVERY_REQUESTED",reason=reason,attempt=attempts+1)
        if self.goal_handle:self.goal_handle.cancel_goal_async()
        else:self.motion_recovery_ready=self.now()+self.limits["navigation"]["goal_recovery_wait_s"]
    def motion_watchdog(self,here):
        if not self.record or self.record.get("status")!="RUNNING" or self.canceling:return
        cfg=self.limits["navigation"];now=self.now()
        if self.request_started is not None and now-self.request_started>cfg["goal_timeout_s"]:
            self.finish("FAILED","Configured goal_timeout_s exceeded");return
        if self.pending and time.monotonic()-self.accept_sent_wall>cfg["goal_accept_timeout_s"]:
            self.finish("FAILED","Goal acceptance timeout; no resend until terminal confirmation");return
        if self.motion_recovery_pending:
            if self.motion_recovery_ready is not None and now>=self.motion_recovery_ready and not self.goal_handle and not self.pending:
                if here and self.nav.server_is_ready() and self.dynamic.get("sensor_fresh") is True:
                    self.motion_recovery_pending=False;self.motion_recovery_ready=None
                    self.state="NAVIGATING";self.progress_anchor=None;self.progress_at=now
                    self.emit("ROTATION_RECOVERY_FORWARD_GOAL");self.send_navigation(force_forward=True)
            return
        if self.state!="NAVIGATING":return
        reason=self.rotation_status.get("reason") if time.monotonic()-self.rotation_status_wall<self.limits["safety"]["command_timeout_s"] else None
        if reason and not self.rotation_escape_active:
            self.begin_rotation_recovery(reason);return
        if not reason:self.rotation_escape_active=False
        if self.goal_handle and not self.fine and not self.precision_pending:
            heartbeat=max(self.record.get("feedback_time_s",now),self.record.get("status_time_s",now))
            if now-heartbeat>cfg["goal_status_timeout_s"]:
                self.finish("FAILED","Configured Goal status/feedback timeout");return
        # Do not count an intentional obstacle/sensor hold as a motion recovery failure.
        if self.fine or self.dynamic.get("hold_navigation",True):
            self.progress_at=now;return
        if self.odom:
            p=self.odom.pose.pose.position;point=(p.x,p.y)
            if self.progress_anchor is None or math.hypot(point[0]-self.progress_anchor[0],point[1]-self.progress_anchor[1])>=cfg["stuck_distance_m"]:
                self.progress_anchor=point;self.progress_at=now
            elif self.progress_at is not None and now-self.progress_at>=cfg["stuck_window_s"]:
                if self.rotation_escape_active:self.finish("FAILED","Forward recovery made no configured progress")
                else:self.begin_rotation_recovery("Configured stuck_window_s exceeded")
    def detour(self,state,ids,**data):
        self.detour_pub.publish(String(data=json.dumps(dict(state=state,track_ids=ids,**data))))
        self.emit("DETOUR_"+state,track_ids=ids,**data)
    def dynamic_event(self,m):
        try:
            event=json.loads(m.data);kind=event["event"]
            if self.state!="NAVIGATING" or self.planning or self.fine or self.canceling:return
            if kind=="DETOUR_EXHAUSTED":self.finish("FAILED","Maximum three detour attempts");return
            if kind not in ("REPLAN_CHECK","CONTACT_PATH_CHECK","DYNAMIC_DETOUR_REQUEST"):return
            execute=kind=="DYNAMIC_DETOUR_REQUEST";cfg=self.limits["dynamic_obstacles"]
            if execute and min(event.get("stationary_s",0),event.get("blocked_s",0))<cfg["detour_after_s"]:return
            if execute and self.record["detour_attempts"]>=cfg["max_detour_attempts"]:self.finish("FAILED","Maximum three detour attempts");return
            if not self.planner.server_is_ready():self.detour("NO_PATH",event.get("track_ids",[]));return
            if execute:self.record["detour_attempts"]+=1;self.ledger.save()
            self.planning=True;self.plan_at=self.now();self.plan_generation+=1;generation=self.plan_generation
            goal=ComputePathToPose.Goal();goal.goal=self.pose_msg();goal.planner_id="ForwardPath"
            self.emit("PLANNER_REQUEST",execute=execute,attempt=self.record["detour_attempts"])
            def accepted(f):
                try:
                    h=f.result()
                    if not h or not h.accepted:raise ValueError("Planner rejected")
                    h.get_result_async().add_done_callback(lambda f:self.plan_result(f,event,execute,generation))
                except Exception as e:
                    if generation==self.plan_generation:self.planning=False;self.detour("NO_PATH",event.get("track_ids",[]),reason=str(e))
            self.planner.send_goal_async(goal).add_done_callback(accepted)
        except (ValueError,KeyError,TypeError) as e:self.emit("BAD_DYNAMIC_EVENT",reason=str(e))
    def plan_result(self,f,event,execute,generation):
        if generation!=self.plan_generation:return
        self.planning=False;ids=event.get("track_ids",[])
        try:
            result=f.result();path=result.result.path
            if result.status!=4 or result.result.error_code or len(path.poses)<2:raise ValueError("No valid planner path")
            if not execute:self.detour("CHECKED",ids,execution_started=False,path_points=len(path.poses));return
            if not set(ids)&set(self.dynamic.get("blocker_ids",[])):self.detour("CANCELED",ids,reason="Blocker moved");return
            points=[(p.pose.position.x,p.pose.position.y) for p in path.poses]
            for tr in self.dynamic.get("tracks",[]):
                if not tr.get("blocks_path"):continue
                clearance=min(segment_distance((tr["x"],tr["y"]),a,b) for a,b in zip(points,points[1:]))
                if clearance<self.limits["dynamic_obstacles"]["robot_radius_m"]+tr["radius_m"]+.05:raise ValueError("Path intersects tracked obstacle")
            for x,y in points:
                if zone_ratio(self.zones,x,y,self.current_grants())==0:raise ValueError("Path enters prohibited/unreserved Zone")
            digest=hashlib.sha256(json.dumps(points).encode()).hexdigest()
            if digest in self.record["failed_paths"]:raise ValueError("Previously failed path")
            self.record["path_hash"]=digest;self.ledger.save()
            if self.pending:raise ValueError("Original goal acceptance unresolved")
            if self.goal_handle:
                self.replacement=(path,ids);self.goal_handle.cancel_goal_async()
            else:self.execute_path(path,ids)
        except Exception as e:self.detour("NO_PATH",ids,reason=str(e))
    def execute_path(self,path,ids):
        if self.state!="NAVIGATING" or self.canceling:return
        self.attempt_ids=list(ids);self.detour_origin=self.pose();self.motion_logged=False
        self.detour("EXECUTING",ids,path=[[p.pose.position.x,p.pose.position.y] for p in path.poses])
        goal=FollowPath.Goal();goal.path=path;goal.controller_id="ForwardController";goal.goal_checker_id="general_goal_checker";goal.progress_checker_id="progress_checker"
        self.send(self.follow,goal,"follow_path")
    def current_grants(self):return self.grants if time.monotonic()-self.grant_time<1. else []
    def finish(self,status,reason=""):
        self.motion_recovery_pending=False;self.motion_recovery_ready=None;self.fine=False;self.fine_pub.publish(Twist());self.fine_active.publish(Bool(data=False))
        if self.goal_handle or self.pending:
            self.terminal_pending=(status,reason);self.canceling=True;self.replacement=None
            self.state="ERROR" if status=="FAILED" else "PAUSED"
            if self.goal_handle:self.goal_handle.cancel_goal_async()
            return
        self.state="ERROR" if status=="FAILED" else "IDLE"
        if self.task and self.record:
            self.ledger.data["error_latched"]=(status=="FAILED");self.record["reason"]=reason;self.ledger.finish(self.task,status)
        self.emit("TASK_"+status,reason=reason);self.recovering=False
    def tick(self):
        here=self.pose();self.motion_watchdog(here);self.hold_pub.publish(Bool(data=here is None or self.state not in ("NAVIGATING","RETURNING")))
        self.zone_pub.publish(Float64(data=zone_ratio(self.zones,*here[:2],self.current_grants()) if here else 0.))
        if self.recovering and time.monotonic()-self.recover_started>self.limits["navigation"]["goal_recovery_wait_s"]:self.finish("FAILED","Cannot verify existing Goal; no automatic resend")
        if self.planning and self.now()-self.plan_at>self.limits["dynamic_obstacles"]["detour_plan_timeout_s"]:
            self.planning=False;self.plan_generation+=1;self.detour("NO_PATH",self.attempt_ids,reason="Planner timeout")
        if here and self.detour_origin and not self.motion_logged:
            moved=math.hypot(here[0]-self.detour_origin[0],here[1]-self.detour_origin[1])
            if moved>=.08:self.motion_logged=True;self.emit("DETOUR_MAP_MOTION_OBSERVED",displacement_m=moved)
        if here and self.goal_handle and not self.pending and not self.canceling and not self.fine and not self.precision_pending and not self.planning and not self.replacement and self.state=="NAVIGATING" and not self.dynamic.get("hold_navigation",True):
            dist=math.hypot(here[0]-self.target[0],here[1]-self.target[1])
            angle=math.atan2(math.sin(self.target[2]-here[2]),math.cos(self.target[2]-here[2]))
            fresh_guard=time.monotonic()-self.rotation_status_wall<self.limits["safety"]["command_timeout_s"]
            if fine_handoff_allowed(dist,angle,self.rotation_status,self.limits["navigation"],self.limits["arrival"],self.forward_mode or self.rotation_escape_active,fresh_guard):
                self.precision_pending=True;self.goal_handle.cancel_goal_async();self.emit("PRECISION_HANDOFF_REQUESTED",distance_m=dist,yaw_error_rad=angle,guard_budget_checked=self.forward_mode or self.rotation_escape_active)
        self.fine_active.publish(Bool(data=self.fine))
        if self.fine:
            cmd=Twist();cfg=self.limits["arrival"]
            if here:
                dist=math.hypot(self.target[0]-here[0],self.target[1]-here[1]);angle=math.atan2(math.sin(self.target[2]-here[2]),math.cos(self.target[2]-here[2]))
                fresh_guard=time.monotonic()-self.rotation_status_wall<self.limits["safety"]["command_timeout_s"]
                if not fresh_guard:
                    self.settled=None;self.yaw_settling=False
                    if self.guard_wait_started is None:self.guard_wait_started=time.monotonic()
                    elif time.monotonic()-self.guard_wait_started>self.limits["navigation"]["goal_status_timeout_s"]:self.finish("FAILED","Fine Safety status stale")
                    verdict="WAIT_STATUS";angular=0.
                else:
                    angular,self.yaw_settling,verdict=yaw_step(angle,self.rotation_status,self.limits["navigation"],cfg,self.yaw_settling)
                    if verdict!="WAIT_GUARD":self.guard_wait_started=None
                if dist<=cfg["xy_tolerance_m"] and verdict=="ALIGNED":
                    still=self.odom and time.monotonic()-self.last_odom_time<.5 and abs(self.odom.twist.twist.linear.x)<cfg["settle_linear_mps"] and abs(self.odom.twist.twist.angular.z)<cfg["settle_angular_rps"]
                    if still:
                        if self.settled is None:self.settled=self.now()
                        if self.now()-self.settled>=cfg["settle_s"]:self.finish("SUCCEEDED")
                    else:self.settled=None
                elif dist>cfg["fine_engage_max_m"]:self.finish("FAILED","Arrival outside fine-correction range")
                elif dist>cfg["xy_tolerance_m"]:
                    # Never turn back toward a centimetre XY error in fine mode.
                    # Return translation to collision-aware Nav2 with the same destination.
                    self.settled=None;self.begin_rotation_recovery("Arrival XY drift outside acceptance")
                else:
                    self.settled=None
                    if verdict=="RECOVER":
                        self.emit("FINE_ROTATION_BUDGET_INSUFFICIENT",yaw_error_rad=angle,rotation=self.rotation_status)
                        self.begin_rotation_recovery("Fine yaw exceeds remaining measured rotation budget")
                    elif verdict=="WAIT_GUARD":
                        if self.guard_wait_started is None:self.guard_wait_started=time.monotonic()
                        elif time.monotonic()-self.guard_wait_started>self.limits["navigation"]["goal_status_timeout_s"]:
                            self.begin_rotation_recovery("Continuous guard did not clear after bounded stop")
                    else:cmd.angular.z=angular
            if self.now()-self.fine_since>cfg["fine_timeout_s"]:self.finish("FAILED","Fine arrival timed out")
            self.fine_pub.publish(cmd if self.fine else Twist())
        body=dict(robot_id=self.robot,state=self.state,task_id=self.task,destination_id=self.record["command"]["destination_id"] if self.record else None,target_pose=self.target,task_state=self.record["status"] if self.record else None,goal_publish_count=self.record["goal_publish_count"] if self.record else 0)
        self.status_pub.publish(String(data=json.dumps(body)));self.public_pub.publish(String(data=json.dumps(body)))

def main():
    rclpy.init();node=NavigationExecutor()
    try:rclpy.spin(node)
    except (KeyboardInterrupt,ExternalShutdownException):pass
    finally:
        node.destroy_node()
        if rclpy.ok():rclpy.shutdown()
