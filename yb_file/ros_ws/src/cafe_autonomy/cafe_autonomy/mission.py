"""Serving mission and constrained fine correction. Never treats Nav2 ABORT as successful arrival."""
import json
import math
import time
from collections import deque
from pathlib import Path
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.time import Time
from rclpy.duration import Duration
from std_msgs.msg import String, Bool
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Odometry
from nav2_msgs.action import NavigateToPose
from action_msgs.msg import GoalStatus
from tf2_ros import Buffer, TransformListener
from lifecycle_msgs.srv import GetState
from .policy import (ACTIVE, norm_angle, distance, velocity_limit,
                     GOAL_NAV_XY_M, ARRIVAL_DETECT_M, FINAL_PHYSICAL_M, next_after_nav)
from .waypoints import read_waypoints, choose_waypoint
from .runtime_config import load_limits
from .dynamic_mission import DynamicMissionMixin
from .goal_recovery import GoalRecoveryMixin


def yaw(q):
    return math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))

class Mission(DynamicMissionMixin,GoalRecoveryMixin,Node):
    def __init__(self):
        super().__init__('serving_mission')
        self.declare_parameter('robot_id','robot_01')
        self.declare_parameter('waypoints_file','')
        self.declare_parameter('fine_timeout_s',45.0)
        self.declare_parameter('fine_engage_max_m',0.20)
        self.declare_parameter('runtime_limits_file','')
        self.limits=load_limits(self.get_parameter('runtime_limits_file').value)
        self.xy_tolerance=self.limits['arrival']['xy_tolerance_m']
        self.yaw_tolerance=self.limits['arrival']['yaw_tolerance_rad']
        self.fine_engage_max=self.limits['arrival']['fine_engage_max_m']
        self.fine_timeout=self.limits['arrival']['fine_timeout_s']
        self.max_v=self.limits['navigation']['max_forward_mps']
        self.max_w=self.limits['navigation']['max_yaw_rps']
        self.robot=self.get_parameter('robot_id').value
        path=self.get_parameter('waypoints_file').value
        self.poses=read_waypoints(path)
        self.action=ActionClient(self,NavigateToPose,f'/{self.robot}/navigate_to_pose')
        self.tf=Buffer(cache_time=Duration(seconds=15))
        self.tf_listener=TransformListener(self.tf,self)
        self.pub=self.create_publisher(String,'serving/status',10)
        self.event=self.create_publisher(String,'navigation_event',10)
        self.fine_pub=self.create_publisher(Twist,'cmd_vel_fine',10)
        self.fine_active=self.create_publisher(Bool,'fine_active',10)
        self.create_subscription(String,'serving/request',self.command,10)
        self.create_subscription(String,'serving/complete',self.confirm,10)
        self.create_subscription(Bool,'traffic_hold',self.set_hold,10)
        self.create_subscription(Odometry,'odom',self.on_odom,10)
        self.hold=True;self.odom=None;self.replan_pending=False;self.replan_count=0
        self.history=deque();self.nav_started=0.;self.cancel_requested=False;self.precision_pending=False;self.yield_pending=False;self.yield_saved=None;self.pause_pending=False;self.paused_nav=False
        self.state='IDLE';self.request_id=None;self.mode=None;self.table=None;self.side='auto'
        self.rotation_status={};self.target_name=None;self.target=None;self.goal_handle=None;self.goal_pending=False
        self.goal_seq=0;self.fine=False;self.fine_since=0.;self.fail_reason=None
        self.init_dynamic_mission()
        self.arrival_settle_since=None;self.forward_curve_goal=False;self.recovery_switch_pending=False
        self.init_goal_recovery()
        self.create_subscription(String,'safety_status',self.on_safety_status,10)
        self.nav_states={};self.health_clients={name:self.create_client(GetState,name+'/get_state') for name in ('amcl','map_server','controller_server','bt_navigator','collision_monitor')}
        self.health_pending=set();self.create_timer(1.0,self.check_health)
        self.create_timer(0.05,self.tick)
        self.get_logger().info(self.robot+' mission ready')

    def check_health(self):
        for name,client in self.health_clients.items():
            if name in self.health_pending or not client.service_is_ready():continue
            self.health_pending.add(name)
            def done(f,name=name):
                self.health_pending.discard(name)
                try:self.nav_states[name]=(f.result().current_state.id,time.monotonic())
                except Exception:self.nav_states.pop(name,None)
            client.call_async(GetState.Request()).add_done_callback(done)

    def nav_ready(self):
        return self.action.server_is_ready() and all(name in self.nav_states and self.nav_states[name][0]==3 and time.monotonic()-self.nav_states[name][1]<3. for name in self.health_clients)

    def emit(self,type_,**kv):
        o={'robot_id':self.robot,'event':type_,'request_id':self.request_id,**self.goal_metadata(),'time_s':self.get_clock().now().nanoseconds*1e-9,'wall_time':time.time(),**kv}
        self.persist_mission()
        m=String(data=json.dumps(o,ensure_ascii=False));self.event.publish(m)

    def on_odom(self,msg):self.odom=msg
    def on_safety_status(self,msg):
        try:self.rotation_status=json.loads(msg.data).get('rotation',{});reason=self.rotation_status.get('reason')
        except ValueError:return
        if reason and self.state in ('GOING_PICKUP','GOING_TABLE','GOING_DIRECT','GOING_KITCHEN','RETURNING_HOME','YIELDING'):
            if getattr(self,'rotation_notice',None)!=reason:self.emit('ROTATION_COMMAND_BLOCKED',reason=reason)
            self.rotation_notice=reason
            if self.goal_handle and not self.forward_curve_goal and not (self.recovery_switch_pending or self.precision_pending or self.pause_pending):
                self.recovery_switch_pending=True;self.recovery_reason='ROTATION_LIMIT_FORWARD_PATH'
                self.goal_handle.cancel_goal_async();self.emit('ROTATION_LIMIT_PLANNER_SWITCH_REQUEST')

    def _apply_hold(self):
        self.hold=bool(self.traffic_hold or self.dynamic_hold)
        # Keep a healthy NavigateToPose UUID during intentional safety waits.
        # Only a 30s detour switch, explicit cancel, precision handoff or recovery
        # cancels it. The downstream safety chain still outputs zero immediately.
        if not self.hold and self.paused_nav and self.goal_confirmed_inactive and not self.goal_pending and not self.goal_handle and not self.detour_planning and self.detour_request is None and self.state in ('GOING_PICKUP','GOING_TABLE','GOING_DIRECT','GOING_KITCHEN','RETURNING_HOME','YIELDING'):
            self.resume_held_goal()

    def robot_pose(self):
        try:
            t=self.tf.lookup_transform('map',self.robot+'/base_footprint',Time(),timeout=Duration(seconds=0.07))
            tr=t.transform.translation
            return float(tr.x),float(tr.y),yaw(t.transform.rotation)
        except Exception:return None

    def pose_error(self):
        p=self.robot_pose()
        if p is None or self.target is None:return None
        return distance(p,self.target[:2]), norm_angle(self.target[2]-p[2]),p

    def command(self,msg):
        v={}
        try:
            v=json.loads(msg.data)
            command=v.get('command')
            if command=='RECOVER':
                if v.get('request_id')!=self.request_id:raise ValueError('Recovery request mismatch')
                if self.state=='ERROR':
                    self.state='RECOVERY_WAIT';self.recovery_reason='OPERATOR_RECOVER';self.recovery_at=time.monotonic()
                    self.recovery_phase=self.recovery_phase or ('RETURNING_HOME' if self.mode=='GO_HOME' else 'GOING_KITCHEN' if self.mode=='GO_KITCHEN' else 'GOING_TABLE' if self.mode=='KITCHEN_PICKUP' and self.target_name==self.table else 'GOING_DIRECT')
                    self.goal_retry_count=0
                if self.state!='RECOVERY_WAIT':raise ValueError('No goal recovery pending')
                if not self.goal_confirmed_inactive:self.reconcile_goal()
                self.emit('RECOVERY_REQUESTED');return
            if command=='YIELD':
                if self.yield_saved or self.fine:return
                if self.paused_nav:
                    self.yield_saved=(self.target_name,self.state,self.target);self.paused_nav=False
                    self.start_goal(self.robot+'_HOME','YIELDING');self.emit('YIELD_RETREAT_STARTED');return
                if not self.goal_handle:return
                self.yield_saved=(self.target_name,self.state,self.target);self.yield_pending=True
                self.goal_handle.cancel_goal_async();self.emit('YIELD_CANCEL_REQUEST');return
            if command=='RESUME':
                if self.state!='YIELD_WAIT' or not self.yield_saved:return
                target,state,pose=self.yield_saved;self.yield_saved=None
                self.start_goal(target,state,pose);self.emit('YIELD_RESUMED');return
            if command=='RESET':
                if v.get('robot_id')!=self.robot or self.state not in ('ERROR','CANCELED','COMPLETED'):
                    raise ValueError('RESET allowed only after terminal state')
                if self.goal_handle or self.goal_pending or self.fine:
                    raise ValueError('Cannot reset while goal active')
                self.request_id=None;self.mode=None;self.table=None;self.target=None;self.target_name=None
                self.fail_reason=None;self.state='IDLE';self.emit('MISSION_RESET');return
            if command=='CANCEL':
                if v.get('request_id')!=self.request_id:raise ValueError('CANCEL request mismatch')
                self.cancel();return
            if command!='START':raise ValueError('Unknown mission command')
            fingerprint=json.dumps({k:v.get(k) for k in ('request_id','robot_id','mode','table','side')},sort_keys=True)
            if v.get('request_id') in self.accepted_requests:
                if self.accepted_requests[v['request_id']]!=fingerprint:raise ValueError('request_id content conflict')
                self.emit('MISSION_DUPLICATE_IGNORED',duplicate_request_id=v['request_id']);return
            if self.state not in ('IDLE','COMPLETED','CANCELED'):
                raise ValueError('Mission busy: '+self.state)
            if not self.nav_ready():raise ValueError('Nav2 action server not ready')
            if self.hold:raise ValueError('Traffic / sensor hold active')
            if not self.robot_pose():raise ValueError('TF map to base_footprint unavailable')
            if v.get('robot_id')!=self.robot:raise ValueError('Wrong robot')
            if v.get('mode') not in ('KITCHEN_PICKUP','DELIVER_DIRECT','TABLE_COLLECT','GO_KITCHEN','GO_HOME'):
                raise ValueError('Unknown mode')
            if v.get('mode') not in ('GO_HOME','GO_KITCHEN') and v.get('table') not in ('table_01','table_02','table_03','table_04'):
                raise ValueError('Unknown table')
            self.request_id=v['request_id'];self.mode=v['mode'];self.table=v.get('table');self.side=v.get('side','auto')
            self.accepted_requests[self.request_id]=fingerprint
            self.fail_reason=None;self.replan_count=0;self.replan_pending=False;self.goal_retry_count=0;self.goal_publish_count=0;self.recovery_phase=None;self.recovery_reason=None
            self.detour_attempt_count=0;self.failed_detour_paths=set()
            if self.mode=='KITCHEN_PICKUP':self.start_goal('kitchen','GOING_PICKUP')
            elif self.mode=='GO_KITCHEN':self.start_goal('kitchen','GOING_KITCHEN')
            elif self.mode=='GO_HOME':self.start_goal(self.robot+'_HOME','RETURNING_HOME')
            else:self.start_goal(self.table,'GOING_DIRECT')
            self.emit('MISSION_ACCEPTED',state=self.state)
        except Exception as exc:self.emit('MISSION_REJECTED',request_id=v.get('request_id'),reason=str(exc))

    def confirm(self,msg):
        try:
            v=json.loads(msg.data)
            if v.get('request_id')!=self.request_id:raise ValueError('Request mismatch')
            command=v.get('command')
            already=self.confirmed_signals.setdefault(self.request_id,[])
            if command in already:self.emit('CONFIRM_DUPLICATE_IGNORED',command=command);return
            err=self.pose_error()
            if not err or err[0]>self.xy_tolerance or abs(err[1])>self.yaw_tolerance:raise ValueError('Confirmation location/yaw mismatch')
            if command=='LOAD' and self.mode=='KITCHEN_PICKUP' and self.state=='WAIT_LOAD':
                already.append(command);self.start_goal(self.table,'GOING_TABLE')
            elif ((command=='COMPLETE' and self.mode in ('KITCHEN_PICKUP','DELIVER_DIRECT')) or (command=='COLLECTED' and self.mode=='TABLE_COLLECT')) and self.state=='WAIT_HANDOVER':
                already.append(command);self.state='COMPLETED';self.emit('MISSION_SIGNAL_COMPLETE',command=command)
            else:raise ValueError('Unexpected confirmation '+str(command)+' at '+self.state)
        except Exception as exc:self.emit('CONFIRM_REJECTED',reason=str(exc))

    def start_goal(self,name,new_state,pose_override=None):
        if self.goal_pending or self.goal_handle or self.fine:
            raise ValueError('Previous goal still active')
        here=self.robot_pose()
        goal=({'x':pose_override[0],'y':pose_override[1],'yaw':pose_override[2]} if pose_override else choose_waypoint(self.poses,name,here[:2] if here else None,self.side if name==self.table else 'auto'))
        self.target_name=name
        self.target=(goal['x'],goal['y'],goal['yaw'])
        self.state=new_state;self.nav_started=time.monotonic();self.history.clear();self.cancel_requested=False
        g=NavigateToPose.Goal();g.pose.header.frame_id='map'
        g.pose.header.stamp=self.get_clock().now().to_msg()
        g.pose.pose.position.x=goal['x'];g.pose.pose.position.y=goal['y']
        g.pose.pose.orientation.z=math.sin(goal['yaw']/2)
        g.pose.pose.orientation.w=math.cos(goal['yaw']/2)
        heading=abs(norm_angle(math.atan2(goal['y']-here[1],goal['x']-here[0])-here[2])) if here else 0.
        remaining=self.limits['navigation']['cumulative_rotation_rad']-self.rotation_status.get('cumulative_rad',0.)
        self.forward_curve_goal=new_state in ('RETURNING_HOME','GOING_KITCHEN') or heading+.15>=remaining or self.recovery_reason=='ROTATION_LIMIT_FORWARD_PATH'
        if self.forward_curve_goal:
            from ament_index_python.packages import get_package_share_directory
            g.behavior_tree=str(Path(get_package_share_directory('cafe_autonomy'))/'config/navigate_forward_curves.xml')
        self.goal_seq+=1;seq=self.goal_seq;self.goal_pending=True
        self.send_tracked_goal(self.action,g,seq,reason='RECOVERY' if self.recovery_reason else 'NEW_PHASE')
        self.emit('NAV_GOAL_SENT',target=name,xy=[goal['x'],goal['y']],yaw=goal['yaw'])

    def goal_response(self,f,seq):
        if seq!=self.goal_seq:return
        self.goal_pending=False
        try:
            h=f.result()
            if not h or not h.accepted:
                self.schedule_goal_recovery('NAV_GOAL_REJECTED');return
            self.goal_handle=h;self.goal_accepted=True;self.goal_confirmed_inactive=False
            self.emit('NAV_GOAL_ACCEPTED')
            if self.cancel_requested:h.cancel_goal_async()
            h.get_result_async().add_done_callback(lambda f,s=seq:self.goal_result(f,s))
        except Exception as exc:
            self.recovery_phase=self.state;self.state='RECOVERY_WAIT';self.goal_confirmed_inactive=False
            self.reconcile_after_restart=True;self.recovery_reason='GOAL_ACCEPTANCE_UNCERTAIN: '+str(exc)
            self.emit('GOAL_ACCEPTANCE_UNCERTAIN',reason=str(exc))

    def goal_result(self,f,seq):
        if seq!=self.goal_seq:return
        if seq in self.result_seen:return
        self.goal_handle=None;self.goal_pending=False
        try:status=f.result().status
        except Exception as exc:self.goal_confirmed_inactive=False;self.reconcile_goal();self.emit('NAV_RESULT_UNCERTAIN',reason=str(exc));return
        self.result_seen.add(seq);self.goal_confirmed_inactive=True;self.goal_accepted=False;self.emit('NAV_ACTION_TERMINAL',nav_status=int(status))
        if self.cancel_requested:
            self.cancel_requested=False;self.state='CANCELED';self.emit('MISSION_CANCELED');return
        if self.state=='ERROR':return
        if self.recovery_switch_pending:
            self.recovery_switch_pending=False
            if status==GoalStatus.STATUS_CANCELED:self.schedule_goal_recovery('ROTATION_LIMIT_FORWARD_PATH');return
            if status!=GoalStatus.STATUS_SUCCEEDED:self.fail('ROTATION_RECOVERY_CANCEL_FAILED','Goal switch cancellation not acknowledged');return
        if self.pause_pending:
            self.pause_pending=False
            if status==GoalStatus.STATUS_CANCELED:
                self.paused_nav=True;self.emit('TRAFFIC_NAV_PAUSED');return
            if status!=GoalStatus.STATUS_SUCCEEDED:self.fail('TRAFFIC_PAUSE_FAILED','Nav2 cancellation failed');return
        if self.yield_pending:
            self.yield_pending=False
            if status!=GoalStatus.STATUS_CANCELED:self.fail('YIELD_CANCEL_FAILED','Cancellation not acknowledged');return
            self.start_goal(self.robot+'_HOME','YIELDING');self.emit('YIELD_RETREAT_STARTED');return
        if self.precision_pending:
            self.precision_pending=False
            if status!=GoalStatus.STATUS_CANCELED:self.fail('PRECISION_CANCEL_FAILED','Nav2 cancel not acknowledged');return
            self.fine=True;self.fine_since=time.monotonic();self.emit('CONTROLLED_FINAL_ALIGNMENT');return
        if status==GoalStatus.STATUS_CANCELED and self.replan_pending:
            self.replan_pending=False
            self.emit('PERSON_REPLAN_CANCEL_ACK',count=self.replan_count)
            try:self.start_goal(self.target_name,self.state,self.target)
            except Exception as exc:self.fail('REPLAN_FAILED',str(exc))
            return
        if self.state=='ERROR':return
        if status==GoalStatus.STATUS_CANCELED:
            self.schedule_goal_recovery('UNEXPECTED_GOAL_CANCELED');return
        if self.detour_goal_result(status):return
        if self.hold and status in (GoalStatus.STATUS_CANCELED,GoalStatus.STATUS_ABORTED):
            self.paused_nav=True;self.emit('DYNAMIC_NAV_PAUSED');return
        err=self.pose_error()
        if err is None:
            self.fail('POSE_UNAVAILABLE','TF unavailable for final verification');return
        d,angle,_=err
        self.emit('NAV_RESULT',nav_status=int(status),map_error_m=d,yaw_error_rad=angle)
        # Nav2 ABORT must never silently turn into mission success, even if pose looks near target.
        if status != GoalStatus.STATUS_SUCCEEDED:
            self.schedule_goal_recovery('Nav2 status %d, map error %.4fm'%(status,d));return
        if d<=self.fine_engage_max:
            self.fine=True;self.fine_since=time.monotonic()
            self.emit('FINE_APPROACH_BEGIN',nav_status=int(status),map_error_m=d)
        else:self.fail('NAV_SUCCEEDED_OUTSIDE_ACCEPTANCE','map error %.4fm'%d)

    def arrived(self,proof):
        if self.detour_path is not None:
            self.publish_detour('COMPLETED',self.detour_ids,map_travel_m=self.detour_travel)
            self.detour_path=None
        self.arrival_settle_since=None
        self.fine=False;self.fine_pub.publish(Twist());self.fine_active.publish(Bool(data=False))
        if self.state=='YIELDING':self.state='YIELD_WAIT';self.emit('YIELD_RETREAT_COMPLETE');return
        try:self.state=next_after_nav(self.state,self.mode)
        except ValueError as exc:self.fail('STATE_ERROR',str(exc));return
        self.emit('NAV_ARRIVED',state=self.state,proof=proof)

    def fail(self,event,reason):
        self.cancel_dynamic_planning()
        self.fine=False;self.fine_pub.publish(Twist());self.fine_active.publish(Bool(data=False))
        if self.goal_handle:self.goal_handle.cancel_goal_async()
        if self.state!='RECOVERY_WAIT':self.recovery_phase=self.state
        self.state='ERROR';self.fail_reason=reason
        self.emit(event,reason=reason)
        self.get_logger().error(self.robot+' '+event+': '+reason)

    def cancel(self):
        self.cancel_dynamic_planning()
        self.cancel_requested=True;self.replan_pending=False;self.pause_pending=False;self.paused_nav=False;self.precision_pending=False;self.yield_pending=False;self.yield_saved=None
        if self.goal_handle is not None:self.goal_handle.cancel_goal_async()
        self.fine=False;self.fine_pub.publish(Twist());self.fine_active.publish(Bool(data=False))
        self.state='CANCELING' if self.goal_handle or self.goal_pending else 'CANCELED'
        self.emit('MISSION_CANCEL_REQUESTED')

    def tick(self):
        self.dynamic_tick()
        self.recovery_tick()
        now=time.monotonic();p=self.robot_pose()
        moving=self.state in ('GOING_PICKUP','GOING_TABLE','GOING_DIRECT','GOING_KITCHEN','RETURNING_HOME','YIELDING')
        if moving and p and not self.hold:
            err=self.pose_error()
            fine_turn=max(abs(err[1]),abs(norm_angle(math.atan2(self.target[1]-p[1],self.target[0]-p[0])-p[2])) if err and err[0]>self.xy_tolerance else 0.) if err else float('inf')
            rotation_remaining=self.limits['navigation']['cumulative_rotation_rad']-self.rotation_status.get('cumulative_rad',0.)
            if err and err[0]<=self.fine_engage_max and fine_turn+.08<rotation_remaining and self.goal_handle and not self.precision_pending:
                self.precision_pending=True;self.goal_handle.cancel_goal_async()
            elif err and err[0]<=self.fine_engage_max and not self.forward_curve_goal and self.goal_handle and not (self.recovery_switch_pending or self.precision_pending or self.pause_pending):
                self.recovery_switch_pending=True;self.recovery_reason='ROTATION_LIMIT_FORWARD_PATH'
                self.goal_handle.cancel_goal_async();self.emit('ROTATION_BUDGET_PLANNER_SWITCH_REQUEST')
            policy_now=self.get_clock().now().nanoseconds*1e-9
            if self.odom:
                odom_pose=self.odom.pose.pose;self.history.append((policy_now,odom_pose.position.x,odom_pose.position.y,yaw(odom_pose.orientation)))
            while self.history and policy_now-self.history[0][0]>self.limits['navigation']['stuck_window_s']+1.:self.history.popleft()
            if len(self.history)>2 and policy_now-self.history[0][0]>=self.limits['navigation']['stuck_window_s']:
                drift=max(math.hypot(v[1]-self.history[0][1],v[2]-self.history[0][2]) for v in self.history)
                if drift<self.limits['navigation']['stuck_distance_m']:
                    self.fail('DRIVING_STUCK_20S','Less than 8cm displacement during 20s of permitted driving')
            if now-self.nav_started>self.limits['navigation']['goal_timeout_s']:self.fail('NAV_TIMEOUT','Configured goal timeout exceeded')
        else:self.history.clear()
        self.fine_active.publish(Bool(data=bool(self.fine)))
        if self.fine:self.fine_step()
        report={'robot_id':self.robot,**self.goal_metadata(),'confirmation_pose_valid':bool(self.pose_error() and self.pose_error()[0]<=self.xy_tolerance and abs(self.pose_error()[1])<=self.yaw_tolerance),'state':self.state,'map_valid':self.robot_pose() is not None,
                'request_id':self.request_id,'mode':self.mode,'table':self.table,
                'target':self.target_name,'reason':self.fail_reason,'fine':self.fine,'dynamic_hold':self.dynamic_hold,'dynamic_stage':self.dynamic_report.get('stage'),'detour_active':self.detour_path is not None,'pose_error':self.pose_error()[:2] if self.pose_error() else None,'target_pose':self.target,'nav_ready':self.nav_ready(),'lifecycle':{name:value[0] for name,value in self.nav_states.items()}}
        self.pub.publish(String(data=json.dumps(report,ensure_ascii=False)))

    def fine_step(self):
        # Every command still flows through semantic safety and Collision Monitor.
        if self.hold:self.fine_since=time.monotonic();self.fine_pub.publish(Twist());return
        if time.monotonic()-self.fine_since > self.fine_timeout:
            self.fail('PRECISION_TIMEOUT','Could not reach configured map acceptance in time');return
        p=self.robot_pose()
        if p is None:
            self.fine_pub.publish(Twist());return
        tx,ty,ta=self.target;x,y,yaw_now=p
        d=math.hypot(tx-x,ty-y);eyaw=norm_angle(ta-yaw_now)
        if d<=self.xy_tolerance and abs(eyaw)<=self.limits['arrival']['control_yaw_target_rad']:
            self.fine_pub.publish(Twist())
            still=self.odom and abs(self.odom.twist.twist.linear.x)<=self.limits['arrival']['settle_linear_mps'] and abs(self.odom.twist.twist.angular.z)<=self.limits['arrival']['settle_angular_rps']
            if still:
                clock=self.get_clock().now().nanoseconds*1e-9
                if self.arrival_settle_since is None:self.arrival_settle_since=clock
                if clock-self.arrival_settle_since>=self.limits['arrival']['settle_s']:self.arrived('SETTLED_MAP_ESTIMATE_10CM_AND_YAW')
            else:self.arrival_settle_since=None
            return
        self.arrival_settle_since=None
        cmd=Twist()
        if self.hold:
            self.fine_pub.publish(cmd);return
        if d>self.xy_tolerance:
            heading=math.atan2(ty-y,tx-x)
            turn=norm_angle(heading-yaw_now)
            if abs(turn)>0.12:
                cmd.angular.z=max(-0.25,min(0.25,1.0*turn))
            else:
                # no reverse; near-goal velocity decreases continuously
                cmd.linear.x=min(0.05,max(0.005,0.8*d))
                cmd.angular.z=max(-0.20,min(0.20,turn))
        elif abs(eyaw)>0.03:
            cmd.angular.z=max(-0.50,min(0.50,eyaw))
        cmd.linear.x,cmd.angular.z=velocity_limit(cmd.linear.x,cmd.angular.z,self.max_v,self.max_w)
        self.fine_pub.publish(cmd)

def main():
    rclpy.init();n=Mission()
    try:rclpy.spin(n)
    finally:n.destroy_node();rclpy.shutdown()
