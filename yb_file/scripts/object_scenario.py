"""Create/move/remove a real non-person Gazebo obstacle; never publish detections."""
import argparse,subprocess,json,math,time
from pathlib import Path
p=argparse.ArgumentParser();p.add_argument('action',choices=['spawn','move','clear']);p.add_argument('--name',default='moving_obstacle');p.add_argument('--x',type=float,default=-1.8);p.add_argument('--y',type=float,default=-2.25);p.add_argument('--to-x',type=float);p.add_argument('--to-y',type=float);p.add_argument('--duration',type=float,default=0.);p.add_argument('--radius',type=float,default=.2);a=p.parse_args()
def service(name,typ,request):
 r=subprocess.run(['gz','service','-s','/world/cafe/'+name,'--reqtype',typ,'--reptype','gz.msgs.Boolean','--timeout','5000','--req',request],capture_output=True,text=True)
 if r.returncode or 'data: true' not in r.stdout:raise RuntimeError(r.stdout+r.stderr)
if a.action=='spawn':
 out=Path(__file__).resolve().parents[1]/'_local_only/dynamic/scenarios';out.mkdir(parents=True,exist_ok=True)
 sdf=f'''<sdf version="1.9"><model name="{a.name}"><static>true</static><link name="body"><collision name="obstacle"><pose>0 0 .6 0 0 0</pose><geometry><cylinder><radius>{a.radius}</radius><length>1.2</length></cylinder></geometry></collision><visual name="object_visual"><pose>0 0 .6 0 0 0</pose><geometry><cylinder><radius>{a.radius}</radius><length>1.2</length></cylinder></geometry><material><diffuse>1 .2 .05 1</diffuse></material></visual></link></model></sdf>'''
 file=out/(a.name+'.sdf');file.write_text(sdf)
 service('create','gz.msgs.EntityFactory',f'sdf_filename:"{file}" name:"{a.name}" allow_renaming:false pose{{position{{x:{a.x} y:{a.y}}}}}')
elif a.action=='clear':service('remove','gz.msgs.Entity',f'name:"{a.name}" type:MODEL')
else:
 n=max(1,round(a.duration/.1));tx=a.to_x if a.to_x is not None else a.x;ty=a.to_y if a.to_y is not None else a.y
 for i in range(n+1):
  t=i/n;service('set_pose','gz.msgs.Pose',f'name:"{a.name}" position{{x:{a.x+(tx-a.x)*t} y:{a.y+(ty-a.y)*t}}}')
  if n>1:time.sleep(a.duration/n)
print(json.dumps({'action':a.action,'name':a.name,'source':'actual Gazebo entity services'}))
