"""RGB-D person + LiDAR foreground tracking and predictive safety adapter.

Uses immutable SLAM occupancy to reject known furniture/walls. Never consumes
Gazebo pose truth. Temporary obstacles are published only into Nav2 costmaps.
"""
import json,math,time
import xml.etree.ElementTree as ET
from pathlib import Path
from collections import deque
import numpy as np
import cv2
import rclpy
from rclpy.node import Node
from rclpy.time import Time
from rclpy.qos import qos_profile_sensor_data,QoSProfile,DurabilityPolicy,ReliabilityPolicy
from geometry_msgs.msg import PoseArray,Twist
from sensor_msgs.msg import LaserScan,PointCloud2
from sensor_msgs_py.point_cloud2 import create_cloud_xyz32
from nav_msgs.msg import OccupancyGrid,Path as NavPath,Odometry
from std_msgs.msg import String,Header
from ros_gz_interfaces.msg import Contacts
from tf2_ros import Buffer,TransformListener
from .dynamic_policy import DynamicPolicy,distance,path_blocks
from .runtime_config import load_limits
from .static_reference import register_static,closest,static_echo


def yaw(q):return math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))


class DynamicObstacles(Node):
    def __init__(self):
        super().__init__('dynamic_obstacles')
        self.robot=self.declare_parameter('robot_id','robot_01').value
        limits=load_limits(self.declare_parameter('runtime_limits_file','').value)
        self.cfg=limits['dynamic_obstacles'];self.engine=DynamicPolicy(self.cfg)
        self.tf=Buffer();self.listener=TransformListener(self.tf,self)
        self.static_shapes=[]
        world=self.declare_parameter('static_world_file','').value
        self.alignment=[self.declare_parameter('alignment_'+k,0.).value for k in ('x','y','yaw')]
        if world:self.load_static_world(world)
        self.grid=None;self.map_meta=None;self.people=[];self.people_wall=0.;self.perception_fresh=False;self.perception_wall=0.
        self.scan_wall=0.;self.contact_wall={i:0. for i in range(3)};self.contact_pairs={i:set() for i in range(3)};self.front_clear=0.;self.scan=None;self.scan_pose=None
        self.active=False;self.path=[];self.path_history=deque();self.target=None;self.cmd=0.;self.angular_cmd=0.;self.odom_speed=0.;self.last_leg=None
        self.last_pose=None;self.last_observe=-1e9
        self.status=self.create_publisher(String,'dynamic_safety',10)
        self.event=self.create_publisher(String,'dynamic_event',20)
        self.cloud=self.create_publisher(PointCloud2,'dynamic_obstacle_cloud',10)
        map_qos=QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL,reliability=ReliabilityPolicy.RELIABLE)
        self.create_subscription(OccupancyGrid,'map',self.on_map,map_qos)
        self.create_subscription(LaserScan,'scan',self.on_scan,qos_profile_sensor_data)
        self.create_subscription(PoseArray,'people_detected',self.on_people,10)
        self.create_subscription(String,'perception_status',self.on_perception,10)
        self.create_subscription(NavPath,'plan',self.on_path,10)
        self.create_subscription(String,'serving/status',self.on_mission,10)
        self.create_subscription(Twist,'cmd_vel_nav_smooth',self.on_cmd,10)
        self.create_subscription(Odometry,'odom',self.on_odom,qos_profile_sensor_data)
        self.contact_subscriptions=[self.create_subscription(Contacts,'contacts/sensor_'+str(i),lambda m,i=i:self.on_contact(m,i),qos_profile_sensor_data) for i in range(3)]
        self.create_subscription(String,'detour_state',self.on_detour,10)
        directory=Path(self.declare_parameter('log_directory',str(Path.home()/'team_dynamic_logs')).value);directory.mkdir(parents=True,exist_ok=True)
        self.log=(directory/(self.robot+'_'+time.strftime('%Y%m%d_%H%M%S')+'.jsonl')).open('w')
        self.create_timer(self.cfg['update_period_s'],self.tick)

    def load_static_world(self,path):
        # Read immutable authored structures, never simulator entity poses/truth.
        def pose(node):return list(map(float,(node.findtext('pose') or '0 0 0 0 0 0').split()))
        def combine(a,b):
            c,s=math.cos(a[5]),math.sin(a[5])
            return [a[0]+c*b[0]-s*b[1],a[1]+s*b[0]+c*b[1],a[2]+b[2],0,0,a[5]+b[5]]
        for model in ET.parse(path).findall('.//world/model'):
            if model.findtext('static')!='true' or model.get('name')=='ground_plane':continue
            for link in model.findall('link'):
                for col in link.findall('collision'):
                    at=combine(combine(pose(model),pose(link)),pose(col));geom=col.find('geometry')
                    if geom is None:continue
                    if geom.find('box') is not None:
                        size=list(map(float,geom.findtext('box/size').split()));shape=('box',size[0]/2,size[1]/2)
                    elif geom.find('cylinder') is not None:shape=('circle',float(geom.findtext('cylinder/radius')),0.)
                    else:continue
                    self.static_shapes.append((at[0],at[1],at[5],*shape))

    def on_map(self,msg):
        if msg.header.frame_id!='map':return
        self.grid=np.array(msg.data,dtype=np.int16).reshape(msg.info.height,msg.info.width)
        self.distance_field=cv2.distanceTransform((self.grid<65).astype(np.uint8),cv2.DIST_L2,3)*msg.info.resolution
        self.map_meta=msg.info
    def on_people(self,msg):
        if msg.header.frame_id!='map':return
        self.people=[(p.position.x,p.position.y) for p in msg.poses if math.isfinite(p.position.x) and math.isfinite(p.position.y)]
        self.people_wall=time.monotonic()
    def on_perception(self,msg):
        try:self.perception_fresh=json.loads(msg.data).get('fresh') is True;self.perception_wall=time.monotonic()
        except ValueError:self.perception_fresh=False
    def on_path(self,msg):
        if msg.header.frame_id=='map':
            now=self.get_clock().now().nanoseconds*1e-9
            self.path=[(p.pose.position.x,p.pose.position.y) for p in msg.poses]
            self.path_history.append((now,list(self.path)))
            while self.path_history and now-self.path_history[0][0]>self.cfg['path_memory_s']:self.path_history.popleft()
    def on_mission(self,msg):
        try:
            data=json.loads(msg.data);self.active=(self.active and data.get('state')=='RECOVERY_WAIT') or data.get('state') in ('GOING_PICKUP','GOING_TABLE','GOING_DIRECT','GOING_KITCHEN','RETURNING_HOME','YIELDING')
            leg=tuple(data.get('target_pose') or ())
            if leg!=self.last_leg:
                self.last_leg=leg;self.path_history.clear()
                for tr in self.engine.tracks.items.values():tr.blocked_since=None;tr.blocked_path=[];tr.emitted.clear()
            self.target=leg
        except (ValueError,TypeError):self.active=False
    def on_cmd(self,msg):
        self.cmd=max(0.,msg.linear.x) if math.isfinite(msg.linear.x) else 0.;self.angular_cmd=msg.angular.z if math.isfinite(msg.angular.z) else 0.
    def on_odom(self,msg):self.odom_speed=math.hypot(msg.twist.twist.linear.x,msg.twist.twist.linear.y)
    def on_contact(self,msg,index):
        now=self.get_clock().now().nanoseconds*1e-9;self.contact_wall[index]=time.monotonic()
        pairs=set()
        for contact in msg.contacts:
            names=(contact.collision1.name,contact.collision2.name)
            # The sensor covers chassis/bumper collisions only. Wheels/casters do not trigger a driving lock.
            if not any('ground' in name for name in names):pairs.add('|'.join(sorted(names)))
        self.contact_pairs[index]=pairs;combined=set().union(*self.contact_pairs.values())
        self.engine.contact(bool(combined),now,combined)
    def on_detour(self,msg):
        try:
            data=json.loads(msg.data);self.engine.detour_result(data.get('track_ids',[]),data.get('state'),self.get_clock().now().nanoseconds*1e-9)
            if data.get('state')=='EXECUTING' and data.get('path'):
                self.path=[tuple(point[:2]) for point in data['path']]
        except (ValueError,TypeError):pass
    def pose(self,frame,stamp=None):
        t=self.tf.lookup_transform('map',frame,stamp or Time())
        return t.transform.translation.x,t.transform.translation.y,yaw(t.transform.rotation)
    def static_points(self,x,y):
        info=self.map_meta;a=yaw(info.origin.orientation);dx=x-info.origin.position.x;dy=y-info.origin.position.y
        ix=((dx*math.cos(a)+dy*math.sin(a))/info.resolution).astype(int)
        iy=((-dx*math.sin(a)+dy*math.cos(a))/info.resolution).astype(int)
        valid=(ix>=0)&(iy>=0)&(ix<info.width)&(iy<info.height)
        result=np.zeros(len(x),dtype=bool)
        result[valid]=self.distance_field[iy[valid],ix[valid]]<=self.cfg['static_match_margin_m']
        ax,ay,aa=self.alignment;dx=x-ax;dy=y-ay
        wx=dx*math.cos(aa)+dy*math.sin(aa);wy=-dx*math.sin(aa)+dy*math.cos(aa)
        world_points=np.column_stack((wx,wy))
        aligned,self.static_alignment=register_static(world_points,self.static_shapes,self.cfg['static_registration_shift_m'],self.cfg['static_registration_yaw_rad'],match_limit=self.cfg['static_registration_match_m'])
        residual,_=closest(aligned,self.static_shapes)
        result|=residual<=self.cfg['static_match_margin_m']
        return result

    def on_scan(self,msg):
        stamp=msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9
        now=self.get_clock().now().nanoseconds*1e-9
        if not msg.ranges or now-stamp>self.cfg['source_timeout_s'] or now<stamp:return
        try:
            laser=self.pose(msg.header.frame_id,Time.from_msg(msg.header.stamp));robot=self.pose(self.robot+'/base_footprint');motion_frame=self.pose(self.robot+'/odom',Time.from_msg(msg.header.stamp))
        except Exception:return
        self.scan_wall=time.monotonic();self.scan=msg;self.scan_pose=laser;self.last_pose=robot
        angles=msg.angle_min+np.arange(len(msg.ranges))*msg.angle_increment
        ranges=np.array(msg.ranges,dtype=float)
        front_angles=np.arctan2(np.sin(angles+laser[2]-robot[2]),np.cos(angles+laser[2]-robot[2]))
        forward=(np.abs(front_angles)<self.cfg['front_clear_half_angle_rad'])&(ranges>=msg.range_min)
        self.front_clear=min(msg.range_max,float(np.min(ranges[forward]))) if np.any(forward) else 0.
        if now-self.last_observe<self.cfg['observation_period_s'] or self.grid is None:return
        self.last_observe=now
        valid=np.isfinite(ranges)&(ranges>=msg.range_min)&(ranges<=self.cfg['recognition_m'])
        indices=np.where(valid)[0];rr=ranges[valid];aa=angles[valid]+laser[2]
        x=laser[0]+rr*np.cos(aa);y=laser[1]+rr*np.sin(aa)
        foreground=~self.static_points(x,y)
        peer=[]
        for r in ('robot_01','robot_02'):
            if r==self.robot:continue
            try:
                p=self.pose(r+'/base_footprint')
                if distance(robot,p)<=self.cfg['recognition_m']:
                    foreground&=(x-p[0])**2+(y-p[1])**2>self.cfg['peer_exclusion_radius_m']**2
                    peer.append((p[0],p[1],self.cfg['robot_radius_m'],'robot'))
            except Exception:pass
        points=np.stack((x[foreground],y[foreground]),axis=1) if np.any(foreground) else np.empty((0,2))
        groups=[];current=[]
        for point in points:
            if current and np.linalg.norm(point-current[-1])>self.cfg['cluster_gap_m']:
                if len(current)>=self.cfg['cluster_min_points']:groups.append(np.array(current))
                current=[]
            current.append(point)
        if len(current)>=self.cfg['cluster_min_points']:groups.append(np.array(current))
        people=[]
        if time.monotonic()-self.people_wall<self.cfg['person_timeout_s']:
            for px,py in self.people:
                d=distance(robot,(px,py))
                if 0.<d<=self.cfg['recognition_m']:
                    radius=self.cfg['person_radius_m']
                    people.append((px+(px-robot[0])*radius/d,py+(py-robot[1])*radius/d,radius,'person'))
        objects=[]
        for group in groups:
            centre=np.mean(group,axis=0);radius=float(np.max(np.linalg.norm(group-centre,axis=1)))
            if len(group)>=5:
                # Fit circular visible arcs (people/cylinders); the mean of a near
                # surface alone underestimates both body position and radius.
                relative=group-centre
                matrix=np.column_stack((2*relative[:,0],2*relative[:,1],np.ones(len(group))))
                fit,_,rank,_=np.linalg.lstsq(matrix,np.sum(relative**2,axis=1),rcond=None)
                square=fit[2]+fit[0]**2+fit[1]**2
                if rank==3 and square>0.:
                    fitted=math.sqrt(square);candidate=centre+fit[:2]
                    residual=float(np.max(np.abs(np.linalg.norm(group-candidate,axis=1)-fitted)))
                    if self.cfg['minimum_object_radius_m']<=fitted<=self.cfg['cluster_max_radius_m'] and residual<self.cfg['circle_fit_residual_m']:
                        centre=candidate;radius=fitted
            if radius>self.cfg['cluster_max_radius_m']:continue
            if any(distance(centre,p)<self.cfg['person_lidar_merge_m']+p[2] for p in people):continue
            known_people=[tr for tr in self.engine.tracks.items.values() if tr.label=='person' and distance(centre,(tr.x,tr.y))<self.cfg['person_lidar_merge_m']]
            if known_people:
                # A close cropped person may temporarily lose YOLO classification.
                # Retain identity and body extent; LiDAR observes the near surface.
                d=distance(robot,centre);body=self.cfg['person_radius_m']
                if d>0. and radius<body*.7:centre=centre+(centre-np.array(robot[:2]))*body/d
                objects.append((float(centre[0]),float(centre[1]),body,'person'))
            else:objects.append((float(centre[0]),float(centre[1]),max(self.cfg['minimum_object_radius_m'],radius),'unclassified_object'))
        def motion_positions(observations):
            if not getattr(self,'static_alignment',{}).get('accepted'):return None
            # Sensor-derived static-surface registration removes ego odometry and
            # AMCL drift from motion judgement only; map poses/control TF stay intact.
            ax,ay,aa=self.alignment;rotation=np.array(self.static_alignment['rotation']);translation=np.array(self.static_alignment['translation'])
            output=[]
            for px,py,*_ in observations:
                dx=px-ax;dy=py-ay
                world=np.array([dx*math.cos(aa)+dy*math.sin(aa),-dx*math.sin(aa)+dy*math.cos(aa)])
                output.append(rotation@world+translation)
            return output
        self.engine.tracks.observe(people,now,'RGBD_YOLO',motion_frame,motion_positions(people),self.alignment[2],getattr(self,'static_alignment',{}).get('accepted',False))
        self.engine.tracks.observe(objects,now,'LiDAR_FOREGROUND',motion_frame,motion_positions(objects),self.alignment[2],getattr(self,'static_alignment',{}).get('accepted',False))
        self.engine.tracks.observe(peer,now,'PEER_MAP_TF',motion_frame)
        if self.active:
            for tr in self.engine.tracks.items.values():
                if tr.first_seen!=now or tr.blocked_path or tr.label=='robot':continue
                for stamp,previous_path in reversed(self.path_history):
                    if 0.<=now-stamp<=self.cfg['path_memory_s'] and path_blocks((tr.x,tr.y),tr.radius,robot,previous_path,self.cfg):
                        tr.blocked_path=list(previous_path);break

    def verified_clear(self,tr):
        if self.scan is None or self.scan_pose is None:return False
        x,y,a=self.scan_pose;angle=math.atan2(tr.y-y,tr.x-x)-a
        angle=math.atan2(math.sin(angle),math.cos(angle))
        index=int(round((angle-self.scan.angle_min)/self.scan.angle_increment))
        if not 0<=index<len(self.scan.ranges):return False
        observed=self.scan.ranges[index]
        center_distance=distance((x,y),(tr.x,tr.y))
        if (math.isfinite(observed) or math.isinf(observed)) and observed>center_distance+tr.radius+self.cfg['clear_ray_margin_m']:
            return True
        # An authored chair or wall can keep the ray occupied after an erroneous
        # foreground track has gone stale. Remove only LiDAR-only tracks when the
        # current echo intersects that track and matches an aligned static surface.
        if (tr.label=='person' or 'LiDAR_FOREGROUND' not in tr.sources or
                len(tr.sources)!=1 or not math.isfinite(observed) or
                observed<self.scan.range_min or
                abs(observed-center_distance)>tr.radius+self.cfg['clear_ray_margin_m']):
            return False
        ray_angle=a+self.scan.angle_min+index*self.scan.angle_increment
        point=(x+observed*math.cos(ray_angle),y+observed*math.sin(ray_angle))
        return static_echo(point,self.alignment,self.static_alignment,self.static_shapes,
                           self.cfg['static_match_margin_m'])

    def tick(self):
        now=self.get_clock().now().nanoseconds*1e-9;wall=time.monotonic()
        camera=self.perception_fresh and wall-self.perception_wall<self.cfg['person_timeout_s']
        lidar=wall-self.scan_wall<self.cfg['source_timeout_s']
        fresh=bool(camera and lidar and self.last_pose and self.map_meta is not None)
        contact_fresh=all(wall-stamp<self.cfg['contact_timeout_s'] for stamp in self.contact_wall.values())
        if self.cfg['require_contact_sensor']:fresh=fresh and contact_fresh
        if self.last_pose:
            self.engine.tracks.age(now,self.verified_clear)
            path=self.path
            if not path and self.target:path=[self.last_pose[:2],self.target[:2]]
            report=self.engine.update(now,self.last_pose,max(self.cmd,self.odom_speed),path,self.front_clear,self.active,self.angular_cmd,planned_path=bool(self.path_history))
        else:report={'stage':'SENSOR_STALE','stop':True,'linear_stop':True,'hold_navigation':True,'speed_ratio':0.,'tracks':[],'events':[],'time_s':now}
        if not fresh:report.update({'stage':'SENSOR_STALE','stop':True,'hold_navigation':True,'speed_ratio':0.})
        report.update({'robot_id':self.robot,'sensor_fresh':bool(fresh),'camera_fresh':bool(camera),'lidar_fresh':bool(lidar),
                       'contact_fresh':contact_fresh,'static_registration':getattr(self,'static_alignment',{}),'robot_pose':self.last_pose,'active_mission':self.active,'wall_time':time.time()})
        for evt in report['events']:
            if fresh:
                self.event.publish(String(data=json.dumps({'robot_id':self.robot,**evt})))
                self.get_logger().info(json.dumps(evt))
        header=Header();header.frame_id='map';header.stamp=self.get_clock().now().to_msg();points=[]
        for tr in self.engine.tracks.items.values():
            if self.last_pose and now-tr.last_seen<=self.cfg['lost_timeout_s'] and distance(self.last_pose,(tr.x,tr.y))-tr.radius<=self.cfg['recognition_m']:
                for a in np.linspace(0.,2*math.pi,24,endpoint=False):points.append([tr.x+(tr.radius+self.cfg['costmap_padding_m'])*math.cos(a),tr.y+(tr.radius+self.cfg['costmap_padding_m'])*math.sin(a),.3])
                points.append([tr.x,tr.y,.3])
        self.cloud.publish(create_cloud_xyz32(header,points))
        self.status.publish(String(data=json.dumps(report)))
        self.log.write(json.dumps(report)+'\n');self.log.flush()
    def destroy_node(self):self.log.close();return super().destroy_node()


def main():
    rclpy.init();n=DynamicObstacles()
    try:rclpy.spin(n)
    finally:n.destroy_node();rclpy.shutdown()
