"""Mission extension: 20s planning check, 30s computed-path execution.

The controller remains Nav2 FollowPath. Its output still passes through the one
Semantic Safety publisher and Collision Monitor. No base command is published.
"""
import json,math,time,hashlib
from geometry_msgs.msg import Twist
from nav2_msgs.action import ComputePathToPose,FollowPath
from action_msgs.msg import GoalStatus
from rclpy.action import ActionClient
from std_msgs.msg import String,Bool
from .dynamic_policy import segment_distance,distance


def candidate_evidence(points,obstacles,cfg,planner_id):
    """Telemetry only: keep raw coordinates and both existing clearance definitions."""
    raw=[[float(x),float(y)] for x,y in points]
    rounded=[[round(x,2),round(y,2)] for x,y in raw]
    clearances=[]
    for obstacle in obstacles:
        minimum=min(segment_distance((obstacle['x'],obstacle['y']),a,b)
                    for a,b in zip(raw,raw[1:]))
        clearances.append({'x':obstacle['x'],'y':obstacle['y'],'radius_m':obstacle['radius'],
                           'minimum_clearance_m':minimum,
                           'approval_required_m':cfg['robot_radius_m']+obstacle['radius']+cfg['detour_path_margin_m'],
                           'policy_required_m':cfg['robot_radius_m']+obstacle['radius']+cfg['path_margin_m']})
    return {'planner_id':planner_id,'candidate_path':raw,
            'candidate_hash':hashlib.sha256(json.dumps(rounded).encode()).hexdigest(),
            'raw_path_sha256':hashlib.sha256(json.dumps(raw).encode()).hexdigest(),
            'obstacle_clearances':clearances}


class DynamicMissionMixin:
    def init_dynamic_mission(self):
        self.dynamic_cfg=self.limits['dynamic_obstacles'];self.traffic_hold=True;self.dynamic_hold=True
        self.dynamic_report={};self.dynamic_stamp=0.;self.detour_request=None;self.detour_planning=False
        self.detour_attempt_count=0;self.detour_plan_count=0;self.failed_detour_paths=set();self.last_detour_hash=None
        self.plan_generation=0;self.plan_started=0.;self.planner_goal=None
        self.detour_path=None;self.detour_ids=[];self.detour_start=None;self.detour_last=None;self.detour_travel=0.;self.detour_motion_sent=False
        self.planning_client=ActionClient(self,ComputePathToPose,'compute_path_to_pose')
        self.follow_client=ActionClient(self,FollowPath,'follow_path')
        self.detour_state_pub=self.create_publisher(String,'detour_state',20)
        self.create_subscription(String,'dynamic_safety',self.on_dynamic_safety,10)
        self.create_subscription(String,'dynamic_event',self.on_dynamic_event,20)
    def set_hold(self,msg):
        self.traffic_hold=bool(msg.data);self._apply_hold()
    def on_dynamic_safety(self,msg):
        try:self.dynamic_report=json.loads(msg.data);self.dynamic_stamp=time.monotonic()
        except ValueError:self.dynamic_report={'sensor_fresh':False,'hold_navigation':True}
        self.dynamic_hold=self.dynamic_report.get('sensor_fresh') is not True or bool(self.dynamic_report.get('hold_navigation',True))
        self._apply_hold()
    def resume_held_goal(self):
        self.paused_nav=False
        if self.detour_path is not None:self.start_follow_path(self.detour_path,self.detour_ids,resume=True)
        else:self.start_goal(self.target_name,self.state,self.target)
        self.emit('TRAFFIC_OR_DYNAMIC_NAV_RESUMED')
    def publish_detour(self,state,ids,**details):
        payload={'robot_id':self.robot,'state':state,'track_ids':list(ids),'request_id':self.request_id,'detour_attempt':self.detour_attempt_count,'planner_requests':self.detour_plan_count,**details}
        self.detour_state_pub.publish(String(data=json.dumps(payload)))
        self.emit('DETOUR_'+state,**{k:v for k,v in payload.items() if k not in ('robot_id','request_id')})
    def on_dynamic_event(self,msg):
        if self.state not in ('GOING_PICKUP','GOING_TABLE','GOING_DIRECT','GOING_KITCHEN','RETURNING_HOME','YIELDING'):return
        try:data=json.loads(msg.data)
        except ValueError:return
        if data.get('robot_id')!=self.robot:return
        event=data.get('event')
        if event=='DETOUR_EXHAUSTED':
            # The policy emits this once per unresolved blockage episode. Keep the
            # destination and mission alive while the downstream safety chain stops.
            self.dynamic_hold=True;self.paused_nav=True;self._apply_hold()
            self.emit('DYNAMIC_DETOUR_SAFE_WAIT',track_ids=data.get('track_ids',[]),
                      episode_id=data.get('episode_id'),failed_plans=data.get('attempts'))
            return
        if event=='CONTACT_PATH_CHECK' and not self.detour_planning:self.request_dynamic_plan('CHECK',data)
        elif event=='BLOCKED_ALERT':self.emit('DYNAMIC_BLOCKED_15S',**{k:v for k,v in data.items() if k not in ('event','robot_id')})
        elif event=='REPLAN_CHECK' and not self.detour_planning:self.request_dynamic_plan('CHECK',data)
        elif event=='DYNAMIC_DETOUR_REQUEST':
            # The policy has checked BOTH stationary time and uninterrupted blocking time.
            if data.get('stationary_s',0)<self.dynamic_cfg['detour_after_s'] or data.get('blocked_s',0)<self.dynamic_cfg['detour_after_s']:return
            self.detour_request=data;self.dynamic_hold=True;self._apply_hold()
    def dynamic_tick(self):
        if time.monotonic()-self.dynamic_stamp>self.limits['safety']['command_timeout_s']:
            self.dynamic_hold=True;self._apply_hold()
        if self.detour_request and self.goal_handle and not self.pause_pending:
            self.pause_pending=True;self.goal_handle.cancel_goal_async();self.emit('DETOUR_SWITCH_CANCEL_REQUEST')
        if self.detour_request and not self.detour_planning and not self.goal_handle and not self.goal_pending:
            event=self.detour_request;self.detour_request=None
            self.fine=False;self.fine_pub.publish(Twist());self.fine_active.publish(Bool(data=False));self.paused_nav=True
            self.request_dynamic_plan('EXECUTE',event)
        if self.detour_planning:
            now=self.get_clock().now().nanoseconds*1e-9
            if now-self.plan_started>self.dynamic_cfg['detour_plan_timeout_s']:
                self.plan_generation+=1;self.detour_planning=False
                if self.planner_goal:self.planner_goal.cancel_goal_async()
                self.planner_goal=None;self.publish_detour('CANCELED',self.detour_ids,reason='PLANNER_TIMEOUT')
        if self.detour_path is not None:
            p=self.robot_pose()
            if p:
                if self.detour_last and distance(p,self.detour_last)<.5:self.detour_travel+=distance(p,self.detour_last)
                self.detour_last=p
                if self.detour_travel>=self.dynamic_cfg['detour_progress_m'] and not self.detour_motion_sent:
                    self.detour_motion_sent=True;self.emit('DETOUR_MOTION_OBSERVED',travel_m=self.detour_travel,map_pose=p)
    def request_dynamic_plan(self,kind,event,planner_id='ForwardPath',retry_same_attempt=False):
        ids=event.get('track_ids',[])
        if self.target is None or not self.planning_client.server_is_ready():
            self.publish_detour('CANCELED',ids,reason='PLANNER_NOT_READY');return
        if self.detour_planning:return
        if kind=='EXECUTE' and not retry_same_attempt:
            # Cumulative count is telemetry only; the policy limits consecutive
            # failed plans for each physical blockage episode.
            self.detour_attempt_count+=1
        self.detour_plan_count+=1
        self.detour_planning=True;self.plan_generation+=1;generation=self.plan_generation
        self.plan_started=self.get_clock().now().nanoseconds*1e-9;self.detour_ids=list(ids)
        goal=ComputePathToPose.Goal();goal.planner_id=planner_id;goal.use_start=False
        goal.goal.header.frame_id='map';goal.goal.header.stamp=self.get_clock().now().to_msg()
        goal.goal.pose.position.x=self.target[0];goal.goal.pose.position.y=self.target[1]
        goal.goal.pose.orientation.z=math.sin(self.target[2]/2);goal.goal.pose.orientation.w=math.cos(self.target[2]/2)
        self.emit('DYNAMIC_PATH_CHECK_SENT' if kind=='CHECK' else 'DYNAMIC_DETOUR_PLAN_SENT',track_ids=ids,policy_time_s=event.get('time_s'),detour_attempt=self.detour_attempt_count,planner_requests=self.detour_plan_count,planner_id=planner_id)
        def response(f):
            try:handle=f.result()
            except Exception as exc:self.dynamic_plan_failed(generation,ids,str(exc));return
            if generation!=self.plan_generation:
                if handle and handle.accepted:handle.cancel_goal_async()
                return
            if not handle or not handle.accepted:self.dynamic_plan_failed(generation,ids,'PLANNER_REJECTED');return
            self.planner_goal=handle
            handle.get_result_async().add_done_callback(lambda f:self.dynamic_plan_result(f,generation,kind,event,planner_id))
        self.planning_client.send_goal_async(goal).add_done_callback(response)
    def dynamic_plan_failed(self,generation,ids,reason):
        if generation!=self.plan_generation:return
        self.detour_planning=False;self.planner_goal=None;self.paused_nav=True
        # No planner result was obtained: do not count this as a failed path.
        self.publish_detour('CANCELED',ids,reason=reason)
    def dynamic_plan_result(self,f,generation,kind,event,planner_id):
        if generation!=self.plan_generation:return
        self.detour_planning=False;self.planner_goal=None;ids=event.get('track_ids',[])
        if self.state not in ('GOING_PICKUP','GOING_TABLE','GOING_DIRECT','GOING_KITCHEN','RETURNING_HOME','YIELDING'):
            self.publish_detour('CANCELED',ids,reason='MISSION_NO_LONGER_ACTIVE');return
        try:result=f.result();path=result.result.path;code=result.result.error_code
        except Exception as exc:self.dynamic_plan_failed(generation,ids,str(exc));return
        if kind=='CHECK':
            self.publish_detour('CHECKED',ids,path_available=result.status==GoalStatus.STATUS_SUCCEEDED and code==0,
                                path_points=len(path.poses),execution_started=False)
            return
        if result.status!=GoalStatus.STATUS_SUCCEEDED or code!=0 or len(path.poses)<2:
            if kind=='EXECUTE' and planner_id=='ForwardPath' and code==ComputePathToPose.Result.NO_VALID_PATH:
                self.request_dynamic_plan(kind,event,planner_id='GridBased',retry_same_attempt=True);return
            self.publish_detour('NO_PATH',ids,reason='PLANNER_NO_VALID_PATH',error_code=int(code));return
        points=[(p.pose.position.x,p.pose.position.y) for p in path.poses]
        evidence=candidate_evidence(points,event.get('obstacles',[]),self.dynamic_cfg,planner_id)
        for clearance in evidence['obstacle_clearances']:
            minimum=clearance['minimum_clearance_m'];required=max(clearance['approval_required_m'],clearance['policy_required_m'])
            if minimum<required:
                self.publish_detour('NO_PATH',ids,reason='PATH_STILL_INTERSECTS_BLOCKER',
                                    minimum_clearance_m=minimum,required_m=required,**evidence);return
        # If the object moved away while planning, normal navigation may already have resumed.
        if not set(ids)&set(self.dynamic_report.get('blocker_ids',[])):
            self.publish_detour('CANCELED',ids,reason='BLOCKER_MOVED_OR_PATH_CLEARED',**evidence);self._apply_hold();return
        digest=evidence['candidate_hash']
        if digest in self.failed_detour_paths:
            self.publish_detour('NO_PATH',ids,reason='PREVIOUS_FAILED_PATH_REJECTED',**evidence);return
        self.last_detour_hash=digest
        self.start_follow_path(path,ids,candidate=evidence)
    def start_follow_path(self,path,ids,resume=False,candidate=None):
        if self.goal_handle or self.goal_pending:return
        self.detour_path=path;self.detour_ids=list(ids);self.paused_nav=False;self.dynamic_hold=False
        if not resume:self.detour_start=self.robot_pose();self.detour_last=self.detour_start;self.detour_travel=0.;self.detour_motion_sent=False
        path.header.stamp=self.get_clock().now().to_msg()
        details={k:v for k,v in (candidate or {}).items() if k!='candidate_path'}
        self.publish_detour('EXECUTING',ids,path=[[p.pose.position.x,p.pose.position.y] for p in path.poses],
                            resume=resume,approval_reason='VALIDATED_FOR_FOLLOW_PATH',**details)
        self.hold=bool(self.traffic_hold)
        goal=FollowPath.Goal();goal.path=path;goal.controller_id='FollowPath';goal.goal_checker_id='general_goal_checker';goal.progress_checker_id='progress_checker'
        self.goal_seq+=1;sequence=self.goal_seq;self.goal_pending=True;self.nav_started=time.monotonic()
        self.send_tracked_goal(self.follow_client,goal,sequence,kind='follow_path',reason='DETOUR_RESUME' if resume else 'DETOUR_EXECUTE')
    def detour_goal_result(self,status):
        if self.detour_path is None and self.goal_kind!='follow_path':return False
        if status==GoalStatus.STATUS_CANCELED:return False
        if status!=GoalStatus.STATUS_SUCCEEDED:
            if self.last_detour_hash:self.failed_detour_paths.add(self.last_detour_hash)
            self.publish_detour('NO_PATH',self.detour_ids,reason='FOLLOW_PATH_ABORTED',nav_status=int(status))
            self.detour_path=None;self.paused_nav=True;self.dynamic_hold=True;self._apply_hold();return True
        self.publish_detour('COMPLETED',self.detour_ids,map_travel_m=self.detour_travel)
        self.detour_path=None;return False
    def cancel_dynamic_planning(self):
        self.plan_generation+=1;self.detour_request=None;self.detour_planning=False
        if self.planner_goal:self.planner_goal.cancel_goal_async()
        self.planner_goal=None
        if self.detour_ids:self.publish_detour('CANCELED',self.detour_ids)
        self.detour_path=None;self.detour_ids=[]
