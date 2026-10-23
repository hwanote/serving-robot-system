"""Actual YOLO RGB + synchronized registered depth, transformed into map.

Never consumes simulator truth. Empty fresh detections differ from sensor failure.
"""
import json, math, time, threading
from collections import deque
from pathlib import Path
import numpy as np
import cv2
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PoseArray, Pose
from std_msgs.msg import String
from tf2_ros import Buffer,TransformListener


class Detector(Node):
    def __init__(self):
        super().__init__('rgbd_people')
        self.robot=self.declare_parameter('robot_id','robot_01').value
        weights=self.declare_parameter('weights','').value
        self.timeout=self.declare_parameter('timeout_s',3.0).value
        if not Path(weights).is_file():raise FileNotFoundError('YOLO weights missing: '+weights)
        from ultralytics import YOLO
        import torch
        torch.set_num_threads(1);torch.set_num_interop_threads(1);cv2.setNumThreads(1)
        self.model=YOLO(weights);self.tf=Buffer();self.listener=TransformListener(self.tf,self)
        self.frames={};self.depth={};self.info={};self.results={};self.lock=threading.Lock();self.running=True
        self.subs=[];self.views=('front','rear_left','rear_right')
        self.out=self.create_publisher(PoseArray,'people_detected',10)
        self.health=self.create_publisher(String,'perception_status',10)
        for view in self.views:
            base=f'/{self.robot}/camera/{view}/'
            self.subs.append(self.create_subscription(Image,base+'image',lambda m,v=view:self.rgb(m,v),qos_profile_sensor_data))
            self.subs.append(self.create_subscription(Image,base+'depth_image',lambda m,v=view:self.dep(m,v),qos_profile_sensor_data))
            self.subs.append(self.create_subscription(CameraInfo,base+'camera_info',lambda m,v=view:self.cal(m,v),qos_profile_sensor_data))
        self.worker=threading.Thread(target=self.infer,daemon=True);self.worker.start()
        self.create_timer(.1,self.publish)

    def rgb(self,m,v):
        with self.lock:self.frames.setdefault(v,deque(maxlen=12)).append((m,time.monotonic()))
    def dep(self,m,v):
        with self.lock:self.depth.setdefault(v,deque(maxlen=12)).append((m,time.monotonic()))
    def cal(self,m,v):self.info[v]=m
    @staticmethod
    def stamp(m):return m.header.stamp.sec+m.header.stamp.nanosec*1e-9
    @staticmethod
    def image(m):
        if m.encoding in ('rgb8','bgr8'):
            rows=np.frombuffer(m.data,dtype=np.uint8).reshape(m.height,m.step)
            arr=rows[:,:m.width*3].reshape(m.height,m.width,3)
            return arr[:,:,::-1].copy() if m.encoding=='rgb8' else arr.copy()
        dtype=np.dtype('>f4' if m.is_bigendian else '<f4') if m.encoding=='32FC1' else np.dtype('>u2' if m.is_bigendian else '<u2')
        if m.encoding not in ('32FC1','16UC1'):raise ValueError('Unsupported image encoding '+m.encoding)
        return np.frombuffer(m.data,dtype=dtype).reshape(m.height,m.step//dtype.itemsize)[:,:m.width].copy()
    def infer(self):
        done={}
        while self.running:
            jobs=[]
            with self.lock:
                for v,frames in self.frames.items():
                    if v not in self.info or not self.depth.get(v):continue
                    for rgb,received in reversed(frames):
                        if self.stamp(rgb)<=done.get(v,-1.):continue
                        dep,dr=min(self.depth[v],key=lambda pair:abs(self.stamp(pair[0])-self.stamp(rgb)))
                        if abs(self.stamp(dep)-self.stamp(rgb))>.12:continue
                        if time.monotonic()-min(received,dr)>self.timeout:continue
                        jobs.append((v,rgb,dep,min(received,dr),self.info[v]));done[v]=self.stamp(rgb);break
            if not jobs:
                time.sleep(.02);continue
            try:
                images=[self.image(rgb) for _,rgb,_,_,_ in jobs]
                outputs=self.model.predict(images,classes=[0],conf=.4,imgsz=320,device='cpu',verbose=False)
                for (v,rgb,dep,received,info),out in zip(jobs,outputs):
                    depth=self.image(dep).astype(np.float32)
                    if dep.encoding=='16UC1':depth*=.001
                    if depth.shape!=images[0].shape[:2]:raise ValueError('RGB and depth are not registered')
                    detections=[]
                    for box in out.boxes:
                        x1,y1,x2,y2=map(int,box.xyxy[0].tolist());h,w=depth.shape
                        x1,x2=max(0,x1),min(w,x2);y1,y2=max(0,y1),min(h,y2)
                        roi=depth[y1:y2,x1:x2];valid=roi[np.isfinite(roi)&(roi>.1)&(roi<8.)]
                        if not valid.size:raise ValueError('Person has no usable depth')
                        d=float(np.percentile(valid,25));u=(x1+x2)/2;vv=(y1+y2)/2
                        # Gazebo camera_link: x forward, y left, z up. Intrinsics use optical convention.
                        q=np.array([d,-(u-info.k[2])*d/info.k[0],-(vv-info.k[5])*d/info.k[4]])
                        tf=self.tf.lookup_transform('map',self.robot+'/'+v+'_camera_link',Time.from_msg(rgb.header.stamp))
                        rot=tf.transform.rotation;xyz=np.array([rot.x,rot.y,rot.z]);qw=rot.w
                        q=q+2*np.cross(xyz,np.cross(xyz,q)+qw*q)
                        tr=tf.transform.translation;detections.append([q[0]+tr.x,q[1]+tr.y,q[2]+tr.z])
                    with self.lock:self.results[v]={'received':received,'detections':detections,'error':None}
            except Exception as exc:
                with self.lock:
                    for v,*_ in jobs:self.results[v]={'received':0.,'detections':[],'error':str(exc)}
                self.get_logger().error('RGB-D inference failed: '+str(exc))

    def publish(self):
        now=time.monotonic()
        with self.lock:result=dict(self.results)
        fresh=all(v in result and now-result[v]['received']<self.timeout and result[v]['error'] is None for v in self.views)
        msg=PoseArray();msg.header.frame_id='map';msg.header.stamp=self.get_clock().now().to_msg()
        points=[]
        if fresh:
            for view in result.values():
                for p in view['detections']:
                    if any(math.hypot(p[0]-q[0],p[1]-q[1])<.35 for q in points):continue
                    points.append(p);pose=Pose();pose.position.x=float(p[0]);pose.position.y=float(p[1]);pose.position.z=float(p[2]);pose.orientation.w=1.;msg.poses.append(pose)
            self.out.publish(msg)
        self.health.publish(String(data=json.dumps({'fresh':fresh,'detector':'YOLO_RGBD','count':len(points),'views':result})))

def main():
    rclpy.init();node=Detector()
    try:rclpy.spin(node)
    finally:node.running=False;node.destroy_node();rclpy.shutdown()
