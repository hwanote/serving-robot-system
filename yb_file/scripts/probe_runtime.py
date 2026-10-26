import json,time,math
from pathlib import Path
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan,Image
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from lifecycle_msgs.srv import GetState
from tf2_ros import Buffer,TransformListener
from rclpy.time import Time
import cv2,numpy as np

root=Path(__file__).resolve().parents[1];out=root/'_local_only';out.mkdir(exist_ok=True)
rclpy.init();n=Node('integration_probe');subs=[];counts={};latest={};edges={};states={};tf=Buffer();listener=TransformListener(tf,n)
def cb(m,key):
    counts.setdefault(key,[]).append(time.monotonic())
    if isinstance(m,String):
        try:latest[key]=json.loads(m.data)
        except ValueError:pass
    elif isinstance(m,Twist):latest[key]={'v':m.linear.x,'w':m.angular.z}
    elif isinstance(m,Image) and key not in latest:
        image=np.frombuffer(m.data,dtype=np.uint8).reshape(m.height,m.step)[:,:m.width*3].reshape(m.height,m.width,3)
        if m.encoding=='rgb8':image=cv2.cvtColor(image,cv2.COLOR_RGB2BGR)
        path=out/(key.replace('/','_')+'.png');cv2.imwrite(str(path),image);latest[key]=str(path)
for r in ('robot_01','robot_02'):
    for topic,typ in [('scan',LaserScan),('odom',Odometry),('cmd_vel_nav',Twist),('cmd_vel_nav_smooth',Twist),('cmd_vel_semantic',Twist),('cmd_vel',Twist),('perception_status',String),('safety_status',String),('serving/status',String),('camera/front/image',Image)]:
        key=f'/{r}/{topic}';subs.append(n.create_subscription(typ,key,lambda m,k=key:cb(m,k),qos_profile_sensor_data))
clients=[];futures=[]
for r in ('robot_01','robot_02'):
    for name in ('map_server','amcl','planner_server','controller_server','bt_navigator','velocity_smoother','collision_monitor'):
        key=f'/{r}/{name}';client=n.create_client(GetState,key+'/get_state');clients.append(client)
        if client.wait_for_service(timeout_sec=.5):
            f=client.call_async(GetState.Request());f.add_done_callback(lambda f,k=key:states.update({k:f.result().current_state.label}));futures.append(f)
start=time.monotonic()
while time.monotonic()-start<8:rclpy.spin_once(n,timeout_sec=.05)
poses={}
for r in ('robot_01','robot_02'):
    try:
        t=tf.lookup_transform('map',r+'/base_footprint',Time());q=t.transform.rotation;poses[r]=[t.transform.translation.x,t.transform.translation.y,math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))]
    except Exception as e:poses[r]=str(e)
    edges[r]=[{'name':i.node_name,'namespace':i.node_namespace} for i in n.get_publishers_info_by_topic(f'/{r}/cmd_vel')]
rates={k:(len(v)-1)/(v[-1]-v[0]) if len(v)>1 else 0 for k,v in counts.items()}
result={'rates_hz':rates,'lifecycle':states,'latest':latest,'map_poses':poses,'base_velocity_publishers':edges}
path=out/('probe_'+time.strftime('%Y%m%d_%H%M%S')+'.json');path.write_text(json.dumps(result,indent=2));print(json.dumps(result,indent=2));n.destroy_node();rclpy.shutdown()
