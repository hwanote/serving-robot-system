"""Fail-closed semantic velocity gate, then Collision Monitor, then Gazebo."""
import json, math, time
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool, Float64
from geometry_msgs.msg import Twist, PoseArray
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovarianceStamped
from .policy import velocity_limit, person_factor
from .runtime_config import load_limits
from .rotation_guard import RotationGuard

class Safety(Node):
    def __init__(self):
        super().__init__('semantic_safety')
        self.declare_parameter('robot_id','robot_01')
        self.robot=self.get_parameter('robot_id').value
        self.declare_parameter('runtime_limits_file','')
        self.limits=load_limits(self.get_parameter('runtime_limits_file').value)
        self.safe_cfg=self.limits['safety'];self.nav_cfg=self.limits['navigation']
        self.rotation=RotationGuard(self.nav_cfg);self.output_v=0.;self.last_output_s=None;self.last_mission_state=None
        self.cmd_nav=Twist();self.cmd_fine=Twist()
        self.nav_stamp=0;self.fine_stamp=0;self.scan_stamp=0
        self.fine=False;self.hold=True;self.people=[];self.xy=None
        self.person_stationary_start={}
        self.output=self.create_publisher(Twist,'cmd_vel_semantic',10)
        self.status=self.create_publisher(String,'safety_status',10)
        self.create_subscription(Twist,'cmd_vel_nav_smooth',self.nav,10)
        self.create_subscription(Twist,'cmd_vel_fine',self.fine_cmd,10)
        self.create_subscription(Bool,'fine_active',self.fine_switch,10)
        self.create_subscription(Bool,'traffic_hold',self.set_hold,10)
        self.create_subscription(LaserScan,'scan',self.on_scan,10)
        self.create_subscription(PoseWithCovarianceStamped,'amcl_pose',self.on_amcl,10)
        self.create_subscription(Odometry,'odom',self.on_odom,10)
        self.create_subscription(PoseArray,'people_detected',self.on_people,10)
        self.track_anchors={};self.track_velocity={};self.track_time={};self.next_track=0
        self.people_last={};self.people_stationary={};self.replan_fired=set();self.people_stamp=0.0;self.perception_stamp=0.;self.perception_fresh=False;self.hold_stamp=0.;self.mission_moving=False;self.zone_ratio=0.;self.zone_stamp=0.
        self.dynamic_stamp=0.;self.dynamic_report={'stop':True,'hold_navigation':True,'sensor_fresh':False}
        self.create_subscription(String,'dynamic_safety',self.on_dynamic,10)
        self.create_subscription(Float64,'zone_speed_ratio',self.on_zone,10)
        self.create_subscription(String,'perception_status',self.on_perception,10)
        self.create_subscription(String,'serving/status',self.on_mission,10)
        self.create_timer(0.05,self.tick)

    def nav(self,msg):self.cmd_nav=msg;self.nav_stamp=time.monotonic()
    def fine_cmd(self,msg):self.cmd_fine=msg;self.fine_stamp=time.monotonic()
    def fine_switch(self,msg):self.fine=msg.data
    def set_hold(self,msg):self.hold=msg.data;self.hold_stamp=time.monotonic()
    def on_dynamic(self,msg):
        try:self.dynamic_report=json.loads(msg.data);self.dynamic_stamp=time.monotonic()
        except ValueError:self.dynamic_report={'stop':True,'sensor_fresh':False}
    def on_zone(self,msg):
        self.zone_ratio=max(0.,min(1.,msg.data)) if math.isfinite(msg.data) else 0.;self.zone_stamp=time.monotonic()
    def on_perception(self,msg):
        try:self.perception_fresh=json.loads(msg.data).get('fresh') is True;self.perception_stamp=time.monotonic()
        except Exception:self.perception_fresh=False
    def on_mission(self,msg):
        try:
            state=json.loads(msg.data).get('state')
            self.mission_moving=state in ('GOING_PICKUP','GOING_TABLE','GOING_DIRECT','GOING_KITCHEN','RETURNING_HOME','YIELDING')
            self.last_mission_state=state
        except Exception:self.mission_moving=False
    def on_scan(self,msg):
        # A finite, plausible scan proves sensor freshness; polygon-level safety is
        # handled separately by Nav2 Collision Monitor. Do not call a wall a person.
        age=(self.get_clock().now().nanoseconds-(msg.header.stamp.sec*10**9+msg.header.stamp.nanosec))*1e-9
        if 0.<=age<self.safe_cfg['scan_timeout_s'] and msg.header.frame_id==self.robot+'/laser_link' and msg.ranges and msg.range_max>msg.range_min and any(math.isfinite(v) and msg.range_min<=v<=msg.range_max for v in msg.ranges):
            self.scan_stamp=time.monotonic()
    def on_odom(self,msg):
        p=msg.pose.pose.position;q=msg.pose.pose.orientation
        angle=math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
        self.rotation.observe(p.x,p.y,angle,msg.twist.twist.angular.z,self.get_clock().now().nanoseconds*1e-9)
    def on_amcl(self,msg):
        p=msg.pose.pose.position;self.xy=(p.x,p.y)
    def on_people(self,msg):
        # The detector contract uses global map poses, never untransformed camera coordinates.
        if msg.header.frame_id!='map':
            self.get_logger().warning('Ignored people detections without map-frame coordinates')
            return
        now=time.monotonic()
        poses=[(p.position.x,p.position.y) for p in msg.poses]
        remaining=set(self.people_last);matched={}
        if not all(math.isfinite(v) for point in poses for v in point):
            self.perception_fresh=False;return
        for point in poses:
            candidate=min(remaining,key=lambda i:math.hypot(point[0]-self.people_last[i][0],point[1]-self.people_last[i][1])) if remaining else None
            if candidate is not None and math.hypot(point[0]-self.people_last[candidate][0],point[1]-self.people_last[candidate][1])<=self.safe_cfg['track_association_m']:
                i=candidate;remaining.remove(i)
            else:
                i=self.next_track;self.next_track+=1
                self.track_anchors[i]=point;self.people_stationary[i]=now
            previous=self.people_last.get(i,point);dt=now-self.track_time.get(i,now)
            self.track_velocity[i]=math.hypot(point[0]-previous[0],point[1]-previous[1])/dt if dt>.01 else 0.
            anchor=self.track_anchors[i]
            if math.hypot(point[0]-anchor[0],point[1]-anchor[1])>self.safe_cfg['stationary_displacement_m']:
                self.track_anchors[i]=point;self.people_stationary[i]=now;self.replan_fired.discard(i)
            self.track_time[i]=now;matched[i]=point
        for i in remaining:
            self.track_anchors.pop(i,None);self.track_velocity.pop(i,None);self.track_time.pop(i,None);self.people_stationary.pop(i,None);self.replan_fired.discard(i)
        self.people_last=matched
        self.people=poses
        self.people_stamp=now

    def tick(self):
        now=time.monotonic();fresh=now-self.scan_stamp<self.safe_cfg['scan_timeout_s']
        source_stamp=self.fine_stamp if self.fine else self.nav_stamp
        cmd=self.cmd_fine if self.fine else self.cmd_nav
        detector_ok=self.perception_fresh and now-self.perception_stamp<self.safe_cfg['detector_timeout_s']
        fleet_ok=now-self.hold_stamp<self.safe_cfg['fleet_timeout_s']
        valid=fresh and detector_ok and fleet_ok and self.mission_moving and not self.hold and now-source_stamp<self.safe_cfg['command_timeout_s']
        result=Twist();dmin=None
        if self.xy and self.people:
            dmin=min(math.hypot(self.xy[0]-p[0],self.xy[1]-p[1]) for p in self.people)
        # Detector data must also be fresh. The LiDAR collision monitor remains downstream.
        if now-self.people_stamp>1.0:
            dmin=None;self.people=[];self.people_last.clear();self.people_stationary.clear();self.replan_fired.clear()
        fac=person_factor(self.dynamic_report.get('front_person_m'),self.safe_cfg['person_stop_m'],self.safe_cfg['person_slow_m'])
        dynamic_ok=now-self.dynamic_stamp<self.safe_cfg['command_timeout_s'] and self.dynamic_report.get('sensor_fresh') is True
        valid=valid and dynamic_ok and not self.dynamic_report.get('stop',True) and not self.dynamic_report.get('hold_navigation',True)
        fac=min(fac,max(0.,min(1.,float(self.dynamic_report.get('speed_ratio',0.)))))
        if valid:
            v,w=velocity_limit(cmd.linear.x,cmd.angular.z,self.nav_cfg['max_forward_mps'],self.nav_cfg['max_yaw_rps'])
            result.linear.x=min(v,self.nav_cfg['max_forward_mps']*self.zone_ratio)*fac if now-self.zone_stamp<1.0 else 0.
            result.angular.z=w if fac>0 else 0.0
        if self.dynamic_report.get('linear_stop'):result.linear.x=0.
        rotation_ok=self.rotation.permits(result.angular.z)
        if not rotation_ok:result.angular.z=0.
        clock=self.get_clock().now().nanoseconds*1e-9
        dt=max(0.,min(.2,clock-self.last_output_s)) if self.last_output_s is not None else 0.
        # Emergency/invalid output goes to zero immediately. Only safe normal
        # acceleration/deceleration uses the existing smoother limits.
        if valid and result.linear.x>0.:
            upper=self.output_v+self.safe_cfg['acceleration_mps2']*dt
            lower=max(0.,self.output_v-self.safe_cfg['deceleration_mps2']*dt)
            result.linear.x=max(lower,min(upper,result.linear.x))
        self.output_v=result.linear.x;self.last_output_s=clock
        self.output.publish(result)
        message={'robot_id':self.robot,'sensor_fresh':fresh,'rotation':self.rotation.snapshot(),'policy_time_s':self.dynamic_report.get('time_s'),'command_time_s':clock,'command_wall_time':time.time(),'output_v':result.linear.x,'output_w':result.angular.z,'traffic_hold':self.hold,
                 'dynamic_fresh':dynamic_ok,'dynamic_stage':self.dynamic_report.get('stage'),'dynamic_stop':self.dynamic_report.get('stop',True),'detector_fresh':detector_ok,'fleet_fresh':fleet_ok,'zone_speed_ratio':self.zone_ratio,'people_motion':{str(i):{'speed_mps':self.track_velocity.get(i,0.),'stationary_for_s':now-self.people_stationary.get(i,now)} for i in self.people_last},'person_distance_m':dmin,'state':('ROTATION_LIMIT' if not rotation_ok else 'STOP' if not valid or fac==0 else
                 'SLOW' if fac<1 else 'NORMAL'),'fine':self.fine}
        self.status.publish(String(data=json.dumps(message)))

def main():
    rclpy.init();n=Safety()
    try:rclpy.spin(n)
    finally:n.destroy_node();rclpy.shutdown()
