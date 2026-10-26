"""Observe real detector input and velocity outputs during a blocking-person mission.

Run person_scenario.py spawn --relative-to robot_02 --distance 1.3 first.
This tool never publishes a person detection or bypasses the safety chain.
"""
import json,time,uuid,urllib.request
from pathlib import Path
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import Twist
from std_msgs.msg import String

root=Path(__file__).resolve().parents[1];out=root/'_local_only';out.mkdir(exist_ok=True)
def post(data):
    req=urllib.request.Request('http://127.0.0.1:8765/api/request',json.dumps(data).encode(),{'Content-Type':'application/json'})
    with urllib.request.urlopen(req,timeout=5) as r:return json.load(r)
rclpy.init();n=Node('human_acceptance');samples=[];latest={};subscriptions=[];events=[]
def cb(msg,key):
    if isinstance(msg,Twist):latest[key]={'v':msg.linear.x,'w':msg.angular.z}
    else:
        try:latest[key]=json.loads(msg.data)
        except ValueError:return
        if key in ('navigation_event','person_replan_event'):events.append(latest[key])
        if key=='safety_status':samples.append({'time':time.monotonic(),**{k:dict(v) for k,v in latest.items()}})
for topic,typ in [('safety_status',String),('navigation_event',String),('person_replan_event',String),('cmd_vel_nav_smooth',Twist),('cmd_vel_semantic',Twist),('cmd_vel',Twist)]:
    subscriptions.append(n.create_subscription(typ,'/robot_02/'+topic,lambda m,k=topic:cb(m,k),qos_profile_sensor_data))
start=time.monotonic()
while time.monotonic()-start<2:rclpy.spin_once(n,timeout_sec=.05)
rid='person-test-'+uuid.uuid4().hex[:12]
reply=post({'action':'SUBMIT','request_id':rid,'robot_id':'robot_02','table':'table_04','mode':'DELIVER_DIRECT'})
print(reply,flush=True)
start=time.monotonic()
while time.monotonic()-start<35:rclpy.spin_once(n,timeout_sec=.05)
slow=[s for s in samples if s['safety_status'].get('state')=='SLOW' and s.get('cmd_vel_nav_smooth',{}).get('v',0)>.01 and 0<s.get('cmd_vel_semantic',{}).get('v',0)<s['cmd_vel_nav_smooth']['v']]
stop=[s for s in samples if s['safety_status'].get('person_distance_m') is not None and s['safety_status']['person_distance_m']<=.8 and abs(s.get('cmd_vel',{}).get('v',1))<.001 and abs(s.get('cmd_vel_semantic',{}).get('v',1))<.001]
report={'request_id':rid,'actual_detector_only':True,'slow_samples':len(slow),'person_stop_samples':len(stop),'stationary_replan_events':[e for e in events if 'REPLAN' in e.get('event','')],'samples':samples,'events':events}
path=out/('human_acceptance_'+time.strftime('%Y%m%d_%H%M%S')+'.json');path.write_text(json.dumps(report,indent=2));print(json.dumps({k:v for k,v in report.items() if k not in ('samples','events')},indent=2),flush=True)
print(post({'action':'CANCEL','request_id':rid}),flush=True)
n.destroy_node();rclpy.shutdown()
raise SystemExit(0 if slow and stop else 1)
