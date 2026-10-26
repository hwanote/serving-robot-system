"""Independent simulation evidence recorder: commands, odom, TF and Gazebo truth.
Never interprets Nav2 command or odometry as ground truth. Reports absent GT explicitly.
"""
import csv
import json
import math
import os
import time
from pathlib import Path
import rclpy
from rclpy.node import Node
from rclpy.time import Time
from rclpy.duration import Duration
from tf2_ros import Buffer, TransformListener
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from tf2_msgs.msg import TFMessage

ROBOTS=('robot_01','robot_02')
FIELDS=['wall_time','robot','gt_x','gt_y','gt_yaw','odom_x','odom_y','odom_yaw',
        'tf_x','tf_y','tf_yaw','xy_error_odom_gt','xy_error_tf_gt','cmd_nav_v','cmd_nav_w',
        'cmd_semantic_v','cmd_semantic_w','cmd_safe_v','cmd_safe_w','safety_state','mission_state',
        'gt_sim_s','tf_sim_s','tf_gt_dt_s','xy_error_tf_gt_at_gt_time','odom_sim_s','odom_gt_dt_s']

def stamp_seconds(stamp):
    return stamp.sec + stamp.nanosec * 1e-9

def heading(q):
    return math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))

def xy_error(a,b):
    return None if a is None or b is None else math.hypot(a[0]-b[0],a[1]-b[1])

def truth_robot_id(child):
    # Gazebo tf2 bridge may publish scoped names such as robot_01::base_footprint.
    for robot in ROBOTS:
        if child==robot:
            return robot
    return None

class Metrics(Node):
    def __init__(self):
        super().__init__('cafe_metrics')
        self.declare_parameter('log_directory',str(Path.home()/'cafe_sim_results'))
        d=Path(self.get_parameter('log_directory').value);d.mkdir(parents=True,exist_ok=True)
        self.output=d/('measure_'+time.strftime('%Y%m%d_%H%M%S')+'.csv')
        self.f=self.output.open('w',newline='',encoding='utf-8')
        self.writer=csv.DictWriter(self.f,fieldnames=FIELDS);self.writer.writeheader()
        self.align=[self.declare_parameter('alignment_'+k,0.).value for k in ('x','y','yaw')]
        self.odom_alignment={};self.truth={};self.odom={};self.cmd={};self.states={};self.truth_stamp={}
        self.truth_sim_stamp={};self.odom_sim_stamp={}
        self.tf=Buffer(cache_time=Duration(seconds=10));self.listener=TransformListener(self.tf,self)
        self.create_subscription(TFMessage,'/cafe/gazebo_pose_truth',self.on_truth,10)
        for robot in ROBOTS:
            self.create_subscription(Odometry,f'/{robot}/odom',lambda m,r=robot:self.on_odom(r,m),10)
            for suffix in ('cmd_vel_nav','cmd_vel_semantic','cmd_vel'):
                self.create_subscription(Twist,f'/{robot}/{suffix}',
                    lambda m,r=robot,k=suffix:self.on_cmd(r,k,m),10)
            for suffix in ('serving/status','safety_status'):
                self.create_subscription(String,f'/{robot}/{suffix}',
                    lambda m,r=robot,k=suffix:self.on_state(r,k,m),10)
        self.create_timer(0.2,self.tick)
        self.get_logger().info(f'Metric log: {self.output}')

    def on_truth(self,msg):
        for t in msg.transforms:
            robot=truth_robot_id(t.child_frame_id)
            if robot:
                p=t.transform.translation
                ox,oy,a=self.align
                self.truth[robot]=(ox+p.x*math.cos(a)-p.y*math.sin(a),oy+p.x*math.sin(a)+p.y*math.cos(a),heading(t.transform.rotation)+a)
                self.truth_stamp[robot]=time.monotonic()
                self.truth_sim_stamp[robot]=stamp_seconds(t.header.stamp)

    def on_odom(self,r,msg):
        p=msg.pose.pose.position
        self.odom[r]=(p.x,p.y,heading(msg.pose.pose.orientation))
        self.odom_sim_stamp[r]=stamp_seconds(msg.header.stamp)

    def on_cmd(self,r,k,msg):self.cmd[(r,k)]=(msg.linear.x,msg.angular.z)
    def on_state(self,r,k,msg):
        try:self.states[(r,k)]=json.loads(msg.data).get('state','UNKNOWN')
        except (ValueError,TypeError):self.states[(r,k)]='INVALID'

    def tick(self):
        now=time.monotonic()
        for robot in ROBOTS:
            truth=self.truth.get(robot) if now-self.truth_stamp.get(robot,-1e9)<1 else None
            raw_odom=self.odom.get(robot)
            if truth and raw_odom and robot not in self.odom_alignment:
                dt=truth[2]-raw_odom[2]
                ca,sa=math.cos(dt),math.sin(dt)
                self.odom_alignment[robot]=(dt,truth[0]-(ca*raw_odom[0]-sa*raw_odom[1]),
                                           truth[1]-(sa*raw_odom[0]+ca*raw_odom[1]))
            odom=raw_odom
            if raw_odom and robot in self.odom_alignment:
                dt,ox,oy=self.odom_alignment[robot]
                ca,sa=math.cos(dt),math.sin(dt)
                odom=(ox+ca*raw_odom[0]-sa*raw_odom[1],
                      oy+sa*raw_odom[0]+ca*raw_odom[1],raw_odom[2]+dt)
            tf_stamp=None
            try:
                t=self.tf.lookup_transform('map',robot+'/base_footprint',Time(),timeout=Duration(seconds=0.01))
                pos=t.transform.translation
                tf=(pos.x,pos.y,heading(t.transform.rotation))
                tf_stamp=stamp_seconds(t.header.stamp)
            except Exception:tf=None
            record=dict.fromkeys(FIELDS,'')
            record['wall_time']=time.time();record['robot']=robot
            for prefix,p in [('gt',truth),('odom',odom),('tf',tf)]:
                if p is not None:record.update({prefix+'_x':p[0],prefix+'_y':p[1],prefix+'_yaw':p[2]})
            record['xy_error_odom_gt']=xy_error(odom,truth) or (0.0 if truth and odom else '')
            record['xy_error_tf_gt']=xy_error(tf,truth) or (0.0 if truth and tf else '')
            if truth:
                gt_stamp=self.truth_sim_stamp[robot]
                record['gt_sim_s']=gt_stamp
                odom_stamp=self.odom_sim_stamp.get(robot)
                if odom_stamp is not None:
                    record['odom_sim_s']=odom_stamp
                    record['odom_gt_dt_s']=odom_stamp-gt_stamp
                if tf_stamp is not None:
                    record['tf_sim_s']=tf_stamp
                    record['tf_gt_dt_s']=tf_stamp-gt_stamp
                try:
                    aligned=self.tf.lookup_transform('map',robot+'/base_footprint',
                        Time(seconds=int(gt_stamp),nanoseconds=round((gt_stamp%1)*1e9)),
                        timeout=Duration(seconds=0.01))
                    pos=aligned.transform.translation
                    record['xy_error_tf_gt_at_gt_time']=xy_error((pos.x,pos.y),truth)
                except Exception:
                    pass
            for name,pre in [('cmd_vel_nav','cmd_nav'),('cmd_vel_semantic','cmd_semantic'),('cmd_vel','cmd_safe')]:
                cmd=self.cmd.get((robot,name))
                if cmd:record.update({pre+'_v':cmd[0],pre+'_w':cmd[1]})
            record['safety_state']=self.states.get((robot,'safety_status'),'')
            record['mission_state']=self.states.get((robot,'serving/status'),'')
            self.writer.writerow(record)
        self.f.flush()

    def destroy_node(self):
        self.f.close();return super().destroy_node()

def main():
    rclpy.init();n=Metrics()
    try:rclpy.spin(n)
    finally:n.destroy_node();rclpy.shutdown()
