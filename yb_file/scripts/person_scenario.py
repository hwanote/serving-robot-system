"""Insert a real textured person into the running Gazebo world. No detector injection."""
import argparse,json,math,subprocess,time,re,xml.etree.ElementTree as ET
from pathlib import Path
ap=argparse.ArgumentParser();ap.add_argument('action',choices=['spawn','move','clear']);ap.add_argument('--name',default='test_person');ap.add_argument('--x',type=float,default=-1.9);ap.add_argument('--y',type=float,default=-2.25);ap.add_argument('--yaw',type=float,default=math.pi);ap.add_argument('--to-x',type=float);ap.add_argument('--to-y',type=float);ap.add_argument('--duration',type=float,default=10.);ap.add_argument('--relative-to',choices=['robot_01','robot_02']);ap.add_argument('--distance',type=float,default=1.3);args=ap.parse_args()
if args.relative_to:
    state=subprocess.run(['gz','model','-m',args.relative_to,'-p'],capture_output=True,text=True,check=True).stdout
    values=re.findall(r'^\s*\[([-\d. ]+)\]\s*$',state,re.M)
    if len(values)<2:raise ValueError('Cannot read actual robot pose')
    xyz=list(map(float,values[-2].split()));rpy=list(map(float,values[-1].split()))
    args.x=xyz[0]+args.distance*math.cos(rpy[2]);args.y=xyz[1]+args.distance*math.sin(rpy[2]);args.yaw=rpy[2]+math.pi
root=Path(__file__).resolve().parents[1];out=root/'_local_only/people';out.mkdir(parents=True,exist_ok=True)
def service(name,typ,request):
    result=subprocess.run(['gz','service','-s','/world/cafe/'+name,'--reqtype',typ,'--reptype','gz.msgs.Boolean','--timeout','5000','--req',request],capture_output=True,text=True)
    if result.returncode or 'data: true' not in result.stdout:raise RuntimeError(result.stdout+result.stderr)
    return result.stdout
def pose(x,y):return f'name: "{args.name}" position {{ x: {x} y: {y} z: 0.0 }} orientation {{ z: {math.sin(args.yaw/2)} w: {math.cos(args.yaw/2)} }}'
if args.action=='clear':print(service('remove','gz.msgs.Entity',f'name: "{args.name}" type: MODEL'));raise SystemExit(0)
if args.action=='spawn':
    cache=Path.home()/'.gz/fuel/fuel.gazebosim.org/openrobotics/models/standing%20person/3'
    if not (cache/'model.sdf').is_file():raise FileNotFoundError('Download the official Standing person Fuel model first')
    doc=ET.parse(cache/'model.sdf');model=doc.getroot().find('model');model.set('name',args.name);ET.SubElement(model,'static').text='true'
    for uri in doc.findall('.//mesh/uri'):uri.text=str(cache/'meshes/standing.dae')
    # Keep realistic visual mesh, use a stable human body collision for LiDAR and physics.
    for link in model.findall('link'):
        for col in list(link.findall('collision')):link.remove(col)
        col=ET.SubElement(link,'collision',{'name':'body'});ET.SubElement(col,'pose').text='0 0 .85 0 0 0';geom=ET.SubElement(col,'geometry');c=ET.SubElement(geom,'cylinder');ET.SubElement(c,'radius').text='.22';ET.SubElement(c,'length').text='1.7'
    path=out/(args.name+'.sdf');doc.write(path,encoding='unicode')
    result=service('create','gz.msgs.EntityFactory',f'sdf_filename: "{path}" name: "{args.name}" allow_renaming: false pose {{ position {{ x: {args.x} y: {args.y} z: 0 }} orientation {{ z: {math.sin(args.yaw/2)} w: {math.cos(args.yaw/2)} }} }}')
    print(result)
else:
    tx=args.to_x if args.to_x is not None else args.x;ty=args.to_y if args.to_y is not None else args.y
    steps=max(1,int(args.duration/.2))
    for i in range(steps+1):
        t=i/steps;service('set_pose','gz.msgs.Pose',pose(args.x+(tx-args.x)*t,args.y+(ty-args.y)*t))
        if steps>1:time.sleep(args.duration/steps)
print(json.dumps({'model':args.name,'action':args.action,'source':'Gazebo visual+collision; no person-detection topic written'}))
