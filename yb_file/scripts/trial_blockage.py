"""Real path blockage fixture and Gazebo ground-truth detour verification."""
import argparse,json,time,math,subprocess,urllib.request,uuid
from pathlib import Path
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from nav_msgs.msg import Path as NavPath
from std_msgs.msg import String
from tf2_msgs.msg import TFMessage
p=argparse.ArgumentParser();p.add_argument('--request-id');p.add_argument('--robot',default='robot_01');p.add_argument('--table',default='table_01');p.add_argument('--side',default='auto');p.add_argument('--insert-displacement',type=float,default=.25);p.add_argument('--timeout',type=float,default=420.);a=p.parse_args();root=Path(__file__).resolve().parents[1]
def post(d):
 q=urllib.request.Request('http://localhost:8765/api/request',json.dumps(d).encode(),{'Content-Type':'application/json'});return json.load(urllib.request.urlopen(q,timeout=5))
def status():
 end=time.monotonic()+15
 while True:
  try:return json.load(urllib.request.urlopen('http://localhost:8765/api/status',timeout=3))
  except Exception:
   if time.monotonic()>end:raise
   rclpy.spin_once(n,timeout_sec=.1);time.sleep(.2)
rclpy.init();n=Node('real_blockage_trial');path=[];truth={};detours=[]
def on_path(m):path[:]=[(p.pose.position.x,p.pose.position.y) for p in m.poses]
def on_truth(m):
 for t in m.transforms:
  if t.child_frame_id==a.robot:truth.update({'x':t.transform.translation.x,'y':t.transform.translation.y})
subs=[n.create_subscription(NavPath,'/'+a.robot+'/plan',on_path,qos_profile_sensor_data),n.create_subscription(TFMessage,'/cafe/gazebo_pose_truth',on_truth,qos_profile_sensor_data),n.create_subscription(String,'/'+a.robot+'/detour_state',lambda m:detours.append(json.loads(m.data)),20)]
rid=a.request_id or 'blockage-'+uuid.uuid4().hex[:10];file=root/'_local_only/dynamic'/('blockage_trial_'+time.strftime('%Y%m%d_%H%M%S')+'.jsonl');f=file.open('w');start=time.monotonic();initial=None;fixture=None;detour_origin=None;success=False;confirmed=False;physical_motion=0.;previous=None;s={}
print(post({'action':'SUBMIT','request_id':rid,'robot_id':a.robot,'mode':'DELIVER_DIRECT','table':a.table,'side':a.side}),flush=True)
try:
 while time.monotonic()-start<a.timeout:
  for _ in range(5):rclpy.spin_once(n,timeout_sec=.03)
  d=status();s=d.get('supervisors',{}).get(a.robot,{});xy=d.get('robot_poses',{}).get(a.robot);dynamic=d.get('dynamic',{}).get(a.robot,{})
  if initial is None and xy and s.get('request_id')==rid and s.get('state') in ('GOING_DIRECT','GOING_TABLE'):initial=xy
  if fixture is None and xy and initial and math.dist(initial,xy)>a.insert_displacement and len(path)>2:
   closest=min(range(len(path)),key=lambda i:math.dist(path[i],xy));length=0.;point=None
   for u,v in zip(path[closest:],path[closest+1:]):
    length+=math.dist(u,v)
    if length>=1.1:point=v;break
   if point:
    fixture={'world_x':point[0]-3.2,'world_y':point[1]-2.25,'map_point':point,'robot_pose':xy,'time':time.time(),'original_path':list(path)}
    subprocess.run(['python3',str(root/'scripts/object_scenario.py'),'spawn','--name','moving_obstacle_trial','--x',str(fixture['world_x']),'--y',str(fixture['world_y'])],check=True)
    print('fixture',fixture,flush=True)
  if detour_origin is None and any(e.get('state')=='EXECUTING' for e in detours) and truth:detour_origin=dict(truth);previous=dict(truth)
  if detour_origin is not None and truth and previous:
   step=math.hypot(truth['x']-previous['x'],truth['y']-previous['y'])
   if step<.5:physical_motion+=step
   previous=dict(truth)
  f.write(json.dumps({'wall':time.time(),'supervisor':s,'dynamic':dynamic,'truth':truth,'fixture':fixture,'detours':detours,'physical_detour_motion_m':physical_motion})+'\n');f.flush()
  if s.get('request_id')==rid and s.get('state')=='WAIT_HANDOVER' and s.get('confirmation_pose_valid') and not confirmed:print(post({'action':'COMPLETE','request_id':rid,'source':'table'}),flush=True);confirmed=True
  if s.get('request_id')==rid and s.get('state') in ('COMPLETED','ERROR','CANCELED'):success=s['state']=='COMPLETED' and physical_motion>=.08;break
finally:
 report={'request_id':rid,'robot_id':a.robot,'fixture':fixture,'detour_states':detours,'physical_detour_motion_m':physical_motion,'success':success,'final_status':s,'elapsed_wall_s':time.monotonic()-start}
 f.close();file.with_suffix('.summary.json').write_text(json.dumps(report,indent=2));print(json.dumps(report,indent=2),flush=True);n.destroy_node();rclpy.shutdown()
