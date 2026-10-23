"""Record real ROS clock, tracked objects, control chain, plans and Gazebo truth."""
import argparse,json,time
from pathlib import Path
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from nav_msgs.msg import Path as NavPath,Odometry
from sensor_msgs.msg import LaserScan,Image
from ros_gz_interfaces.msg import Contacts
from tf2_msgs.msg import TFMessage
p=argparse.ArgumentParser();p.add_argument('--duration',type=float,default=300.);p.add_argument('--label',default='dynamic');a=p.parse_args()
rclpy.init();n=Node('dynamic_acceptance_recorder',parameter_overrides=[rclpy.parameter.Parameter('use_sim_time',value=True)])
out=Path(__file__).resolve().parents[1]/'_local_only/dynamic';out.mkdir(parents=True,exist_ok=True);file=out/(a.label+'_'+time.strftime('%Y%m%d_%H%M%S')+'.jsonl');f=file.open('w');subs=[]
def record(m,topic):
 data={'wall_time':time.time(),'sim_s':n.get_clock().now().nanoseconds*1e-9,'topic':topic}
 if isinstance(m,String):
  try:data['data']=json.loads(m.data)
  except ValueError:data['data']=m.data
 elif isinstance(m,Twist):data['data']={'v':m.linear.x,'w':m.angular.z}
 elif isinstance(m,NavPath):data['data']={'frame':m.header.frame_id,'points':[[p.pose.position.x,p.pose.position.y] for p in m.poses]}
 elif isinstance(m,Odometry):
  p=m.pose.pose.position;q=m.pose.pose.orientation
  data['data']={'x':p.x,'y':p.y,'q':[q.x,q.y,q.z,q.w],'v':m.twist.twist.linear.x,'w':m.twist.twist.angular.z}
 elif isinstance(m,LaserScan):data['data']={'stamp':m.header.stamp.sec+m.header.stamp.nanosec*1e-9,'frame':m.header.frame_id,'points':len(m.ranges),'range_min':m.range_min,'range_max':m.range_max,'angle_min':m.angle_min,'angle_max':m.angle_max}
 elif isinstance(m,Image):data['data']={'stamp':m.header.stamp.sec+m.header.stamp.nanosec*1e-9,'frame':m.header.frame_id,'width':m.width,'height':m.height,'encoding':m.encoding}
 elif isinstance(m,Contacts):data['data']={'stamp':m.header.stamp.sec+m.header.stamp.nanosec*1e-9,'pairs':[[c.collision1.name,c.collision2.name] for c in m.contacts]}
 else:data['data']=[{'name':t.child_frame_id,'x':t.transform.translation.x,'y':t.transform.translation.y,'z':t.transform.translation.z,'q':[t.transform.rotation.x,t.transform.rotation.y,t.transform.rotation.z,t.transform.rotation.w]} for t in m.transforms]
 f.write(json.dumps(data)+'\n');f.flush()
for r in ('robot_01','robot_02'):
 for topic,typ in [('safety_status',String),('perception_status',String),('odom',Odometry),('scan',LaserScan),('contacts/sensor_0',Contacts),('contacts/sensor_1',Contacts),('contacts/sensor_2',Contacts),('camera/front/image',Image),('camera/rear_left/image',Image),('camera/rear_right/image',Image),('dynamic_safety',String),('dynamic_event',String),('detour_state',String),('navigation_event',String),('serving/status',String),('cmd_vel_nav_smooth',Twist),('cmd_vel_semantic',Twist),('cmd_vel',Twist),('plan',NavPath)]:
  topic='/'+r+'/'+topic;subs.append(n.create_subscription(typ,topic,lambda m,t=topic:record(m,t),qos_profile_sensor_data))
subs.append(n.create_subscription(TFMessage,'/cafe/gazebo_pose_truth',lambda m:record(m,'/cafe/gazebo_pose_truth'),qos_profile_sensor_data))
print(str(file),flush=True);start=time.monotonic()
try:
 while time.monotonic()-start<a.duration:rclpy.spin_once(n,timeout_sec=.1)
finally:f.close();n.destroy_node();rclpy.shutdown()
