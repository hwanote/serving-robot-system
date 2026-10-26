"""Temporarily pause this session's Robot 02 detector; always resume it in finally."""
import json,os,signal,time
from pathlib import Path
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import String
from geometry_msgs.msg import Twist

root=Path(__file__).resolve().parents[1];candidates=[]
for proc in Path('/proc').iterdir():
    if not proc.name.isdigit():continue
    try:
        raw=(proc/'cmdline').read_bytes().replace(b'\x00',b' ').decode()
        if str(root/'ros_ws/install/cafe_autonomy/lib/cafe_autonomy/rgbd_people') in raw and '__ns:=/robot_02' in raw:candidates.append(int(proc.name))
    except (FileNotFoundError,PermissionError,ProcessLookupError):pass
if len(candidates)!=1:raise RuntimeError('Expected exactly one Robot 02 detector in this checkout: '+str(candidates))
pid=candidates[0];rclpy.init();n=Node('stale_detector_acceptance');latest={};samples=[]
def cb(m,k):
    latest[k]=json.loads(m.data) if isinstance(m,String) else {'v':m.linear.x,'w':m.angular.z}
    if k=='safety':samples.append({'wall':time.monotonic(),**{key:dict(value) for key,value in latest.items()}})
subs=[n.create_subscription(String,'/robot_02/safety_status',lambda m:cb(m,'safety'),qos_profile_sensor_data),n.create_subscription(Twist,'/robot_02/cmd_vel',lambda m:cb(m,'base'),qos_profile_sensor_data),n.create_subscription(Twist,'/robot_02/cmd_vel_nav_smooth',lambda m:cb(m,'nav'),qos_profile_sensor_data)]
def observe(seconds):
    deadline=time.monotonic()+seconds
    while time.monotonic()<deadline:rclpy.spin_once(n,timeout_sec=.05)
observe(2);stopped=time.monotonic()
try:
    os.kill(pid,signal.SIGSTOP);observe(7)
finally:os.kill(pid,signal.SIGCONT)
resumed=time.monotonic();observe(4)
closed=[s for s in samples if s['wall']>stopped+3.5 and s['wall']<resumed and s['safety'].get('detector_fresh') is False and abs(s.get('base',{}).get('v',1))<.001 and abs(s.get('base',{}).get('w',1))<.001]
report={'pid':pid,'paused_at':stopped,'resumed_at':resumed,'fail_closed_samples':len(closed),'samples':samples}
path=root/'_local_only'/('detector_stale_'+time.strftime('%Y%m%d_%H%M%S')+'.json');path.write_text(json.dumps(report,indent=2));print(json.dumps({k:v for k,v in report.items() if k!='samples'},indent=2));n.destroy_node();rclpy.shutdown()
raise SystemExit(0 if closed else 1)
