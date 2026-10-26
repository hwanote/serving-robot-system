"""Goal bookkeeping/reconciliation inside the existing per-robot Supervisor."""
import json,time,uuid,os
from pathlib import Path
from rclpy.qos import QoSProfile,DurabilityPolicy,ReliabilityPolicy
from unique_identifier_msgs.msg import UUID
from action_msgs.msg import GoalStatus,GoalStatusArray
from action_msgs.srv import CancelGoal
from nav2_msgs.action import NavigateToPose,FollowPath
from std_msgs.msg import String

class RemoteGoal:
    def __init__(self,owner,goal_id,kind):self.owner=owner;self.goal_id=goal_id;self.accepted=True;self.kind=kind
    def cancel_goal_async(self):
        request=CancelGoal.Request();request.goal_info.goal_id=self.goal_id
        return self.owner.cancel_clients[self.kind].call_async(request)
    def get_result_async(self):
        action=NavigateToPose if self.kind=='navigate_to_pose' else FollowPath
        request=action.Impl.GetResultService.Request();request.goal_id=self.goal_id
        return self.owner.result_clients[self.kind].call_async(request)

class GoalRecoveryMixin:
    def init_goal_recovery(self):
        self.state_path=Path(self.declare_parameter('state_file','').value)
        if not str(self.state_path) or str(self.state_path)=='.':raise ValueError('state_file required')
        self.state_path.parent.mkdir(parents=True,exist_ok=True)
        self.accepted_requests={};self.confirmed_signals={};self.goal_uuid=None;self.goal_kind='navigate_to_pose'
        self.goal_publish_count=0;self.goal_retry_count=0;self.goal_accepted=False;self.goal_last_feedback=None;self.goal_feedback={};self.goal_sent_wall=0.;self.accept_future=None;self.server_restart_seen=False
        self.goal_statuses={};self.result_seen=set();self.query_pending=False;self.remote_result_future=None
        self.recovery_phase=None;self.recovery_reason=None;self.recovery_at=0.;self.goal_confirmed_inactive=True;self.reconcile_after_restart=False;self.wait_pose_recheck=False
        self.result_clients={};self.cancel_clients={};self.status_subs=[];self.previous_nav_ready=True;self.ready_lost_at=None
        for name,action in [('navigate_to_pose',NavigateToPose),('follow_path',FollowPath)]:
            self.result_clients[name]=self.create_client(action.Impl.GetResultService,name+'/_action/get_result')
            self.cancel_clients[name]=self.create_client(CancelGoal,name+'/_action/cancel_goal')
            qos=QoSProfile(depth=1,reliability=ReliabilityPolicy.RELIABLE,durability=DurabilityPolicy.TRANSIENT_LOCAL)
            self.status_subs.append(self.create_subscription(GoalStatusArray,name+'/_action/status',lambda m,k=name:self.on_goal_status(m,k),qos))
            self.status_subs.append(self.create_subscription(action.Impl.FeedbackMessage,name+'/_action/feedback',self.on_remote_feedback,10))
        if self.state_path.is_file():
            data=json.loads(self.state_path.read_text());self.accepted_requests=data.get('accepted_requests',{});self.confirmed_signals=data.get('confirmed_signals',{})
            for key in ('request_id','mode','table','side','target_name','goal_uuid','goal_kind','goal_publish_count','goal_retry_count','fail_reason','detour_attempt_count','detour_plan_count','detour_ids','forward_curve_goal'):
                if key in data:setattr(self,key,data[key])
            self.failed_detour_paths=set(data.get('failed_detour_paths',[]))
            self.target=tuple(data['target']) if data.get('target') else None
            old=data.get('state','IDLE');self.recovery_phase=data.get('recovery_phase') or old
            if old not in ('IDLE','COMPLETED','CANCELED','ERROR','WAIT_LOAD','WAIT_HANDOVER'):
                self.state='RECOVERY_WAIT';self.goal_confirmed_inactive=False;self.reconcile_after_restart=True;self.recovery_reason='SUPERVISOR_RESTART'
            else:self.state=old;self.wait_pose_recheck=old in ('WAIT_LOAD','WAIT_HANDOVER')
    def persist_mission(self):
        if not hasattr(self,'state_path'):return
        data={k:getattr(self,k,None) for k in ('request_id','mode','table','side','target_name','target','state','goal_uuid','goal_kind','goal_publish_count','goal_retry_count','recovery_phase','recovery_reason','accepted_requests','confirmed_signals','fail_reason','detour_attempt_count','detour_plan_count','detour_ids','forward_curve_goal')}
        data['failed_detour_paths']=list(self.failed_detour_paths)
        temp=self.state_path.with_suffix('.tmp');temp.write_text(json.dumps(data,ensure_ascii=False));temp.replace(self.state_path)
    def goal_metadata(self):
        return {'process_id':os.getpid(),'accepted_signals':self.confirmed_signals.get(self.request_id,[]),'mission_phase':self.state,'goal_uuid':self.goal_uuid,'goal_kind':self.goal_kind,'goal_accepted':self.goal_accepted,'goal_publish_count':self.goal_publish_count,'goal_retry_count':self.goal_retry_count,'goal_feedback':self.goal_feedback,'goal_recovery_reason':self.recovery_reason,'goal_execution_verified_idle':self.goal_confirmed_inactive and not self.goal_handle and not self.goal_pending}
    def send_tracked_goal(self,client,goal,seq,kind='navigate_to_pose',reason='NEW_PHASE'):
        self.goal_kind=kind;goal_id=UUID(uuid=list(uuid.uuid4().bytes));self.goal_uuid=bytes(goal_id.uuid).hex()
        self.goal_publish_count+=1;self.goal_accepted=False;self.goal_confirmed_inactive=False;self.goal_sent_wall=time.monotonic();self.goal_last_feedback=None;self.query_pending=False
        # Persist the UUID intent before sending. Reconciliation, never blind resend,
        # handles a crash between this write, server acceptance and its reply.
        self.persist_mission()
        def feedback(message):
            if bytes(message.goal_id.uuid).hex()!=self.goal_uuid:return
            self.goal_last_feedback=self.get_clock().now().nanoseconds*1e-9
            f=message.feedback;self.goal_feedback={key:getattr(f,key) for key in ('distance_remaining','number_of_recoveries','distance_to_goal','speed') if hasattr(f,key)}
            if time.monotonic()-getattr(self,'feedback_log_wall',0.)>=1.:
                self.feedback_log_wall=time.monotonic();self.emit('NAV_FEEDBACK',feedback=self.goal_feedback)
        self.accept_future=client.send_goal_async(goal,feedback_callback=feedback,goal_uuid=goal_id)
        self.accept_future.add_done_callback(lambda f:self.goal_response(f,seq))
        self.emit('NAV_ACTION_PUBLISHED',reason=reason)
    def on_goal_status(self,msg,kind):
        self.goal_statuses[kind]={bytes(s.goal_info.goal_id.uuid).hex():int(s.status) for s in msg.status_list}
        status=self.goal_statuses[kind].get(self.goal_uuid)
        if kind==self.goal_kind and status in (1,2,3):self.goal_accepted=True
    def on_remote_feedback(self,msg):
        if not isinstance(self.goal_handle,RemoteGoal) or bytes(msg.goal_id.uuid).hex()!=self.goal_uuid:return
        self.goal_last_feedback=self.get_clock().now().nanoseconds*1e-9;f=msg.feedback
        self.goal_feedback={key:getattr(f,key) for key in ('distance_remaining','number_of_recoveries','distance_to_goal','speed') if hasattr(f,key)}
        if time.monotonic()-getattr(self,'feedback_log_wall',0.)>=1.:
            self.feedback_log_wall=time.monotonic();self.emit('NAV_FEEDBACK_REATTACHED',feedback=self.goal_feedback)
    def schedule_goal_recovery(self,reason):
        if self.state!='RECOVERY_WAIT':self.recovery_phase=self.state
        self.state='RECOVERY_WAIT';self.recovery_reason=reason;self.recovery_at=time.monotonic();self.fine=False
        self.goal_handle=None;self.goal_pending=False;self.goal_confirmed_inactive=True;self.goal_accepted=False
        self.pause_pending=False;self.precision_pending=False;self.paused_nav=False
        self.emit('GOAL_RECOVERY_WAIT',reason=reason)
    def reconcile_goal(self):
        if self.query_pending or not self.goal_uuid:return
        client=self.result_clients[self.goal_kind]
        if not client.service_is_ready():return
        self.query_pending=True;self.goal_confirmed_inactive=False
        goal_id=UUID(uuid=list(bytes.fromhex(self.goal_uuid)));remote=RemoteGoal(self,goal_id,self.goal_kind)
        status=self.goal_statuses.get(self.goal_kind,{}).get(self.goal_uuid)
        future=remote.get_result_async();self.remote_result_future=future;sequence=self.goal_seq
        if status in (1,2,3):
            self.goal_handle=remote;self.goal_pending=False;self.goal_accepted=True;self.nav_started=time.monotonic();self.history.clear()
            if self.state=='RECOVERY_WAIT':self.state=self.recovery_phase
            self.emit('NAV_GOAL_REATTACHED',remote_status=status)
        self.emit('NAV_GOAL_RECONCILE_SENT',remote_status=status)
        def result(f):
            self.query_pending=False;self.reconcile_after_restart=False
            try:value=f.result()
            except Exception as exc:self.emit('GOAL_RECONCILE_UNCERTAIN',reason=str(exc));return
            if sequence!=self.goal_seq:return
            if value.status==GoalStatus.STATUS_UNKNOWN:
                if self.accept_future is not None and not self.accept_future.done() and not self.server_restart_seen:
                    self.goal_confirmed_inactive=False;self.emit('GOAL_ACCEPTANCE_STILL_UNCERTAIN');return
                self.schedule_goal_recovery('SERVER_CONFIRMED_UUID_UNKNOWN');return
            if self.state=='RECOVERY_WAIT':self.state=self.recovery_phase
            self.goal_result(f,sequence)
        future.add_done_callback(result)
    def recovery_tick(self):
        now=time.monotonic();ready=self.nav_ready()
        if not ready:
            if self.ready_lost_at is None:self.ready_lost_at=now
        else:self.ready_lost_at=None
        if self.query_pending and self.goal_handle is None and self.goal_statuses.get(self.goal_kind,{}).get(self.goal_uuid) in (1,2,3):
            self.goal_handle=RemoteGoal(self,UUID(uuid=list(bytes.fromhex(self.goal_uuid))),self.goal_kind);self.goal_pending=False;self.goal_accepted=True;self.nav_started=time.monotonic();self.history.clear()
            if self.state=='RECOVERY_WAIT':self.state=self.recovery_phase
            self.emit('NAV_GOAL_REATTACHED',reason='STATUS_DISCOVERED_AFTER_RESULT_QUERY')
        if self.wait_pose_recheck and ready and self.robot_pose():
            self.wait_pose_recheck=False;error=self.pose_error()
            if error and error[0]>self.limits['navigation']['restart_displacement_m']:
                self.recovery_phase='GOING_PICKUP' if self.state=='WAIT_LOAD' else 'GOING_TABLE' if self.mode=='KITCHEN_PICKUP' else 'GOING_DIRECT'
                self.state='RECOVERY_WAIT';self.recovery_reason='WAIT_POSITION_CHANGED_ON_RESTART';self.goal_confirmed_inactive=False;self.reconcile_after_restart=True
        if self.reconcile_after_restart and ready:self.reconcile_goal()
        if self.goal_pending and now-self.goal_sent_wall>self.limits['navigation']['goal_accept_timeout_s'] and ready:self.reconcile_goal()
        if self.goal_handle and self.ready_lost_at is not None and now-self.ready_lost_at>self.limits['navigation']['goal_status_timeout_s']:
            if self.state!='RECOVERY_WAIT':self.recovery_phase=self.state
            self.state='RECOVERY_WAIT';self.reconcile_after_restart=True;self.goal_confirmed_inactive=False;self.server_restart_seen=True
        if self.state=='RECOVERY_WAIT' and self.goal_confirmed_inactive and not self.query_pending and not self.hold and ready and now-self.recovery_at>=self.limits['navigation']['goal_recovery_wait_s']:
            if self.goal_retry_count>=self.limits['navigation']['goal_retry_max']:
                self.fail('GOAL_RETRY_EXHAUSTED',self.recovery_reason or 'Goal recovery limit');return
            self.goal_retry_count+=1;phase=self.recovery_phase;self.emit('GOAL_RECOVERY_RETRY',reason=self.recovery_reason)
            error=self.pose_error()
            if error and error[0]<=self.fine_engage_max and self.recovery_reason!='ROTATION_LIMIT_FORWARD_PATH' and abs(error[1])+.08<self.limits['navigation']['cumulative_rotation_rad']-self.rotation_status.get('cumulative_rad',0.):
                self.state=phase;self.fine=True;self.fine_since=time.monotonic();self.nav_started=time.monotonic();self.emit('RECOVERY_FINE_ALIGNMENT')
            else:self.start_goal(self.target_name,phase,self.target)
