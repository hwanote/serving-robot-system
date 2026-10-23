"""Real Fleet concurrent retry, phase confirmation and no automatic follow-up test."""
import concurrent.futures,json,time,urllib.request,uuid,math
from pathlib import Path
root=Path(__file__).resolve().parents[1];out=root/'_local_only';log=out/('signal_trial_'+time.strftime('%Y%m%d_%H%M%S')+'.jsonl');f=log.open('w');report={};bodies={};start=time.monotonic()
def get():return json.load(urllib.request.urlopen('http://localhost:8765/api/status',timeout=5))
def post(body):
 req=urllib.request.Request('http://localhost:8765/api/request',json.dumps(body).encode(),{'Content-Type':'application/json'})
 return json.load(urllib.request.urlopen(req,timeout=8))
def record(stage,data):f.write(json.dumps({'wall':time.time(),'elapsed':time.monotonic()-start,'stage':stage,'data':data})+'\n');f.flush()
try:
 for r,mode,table in [('robot_01','DELIVER_DIRECT','table_03'),('robot_02','TABLE_COLLECT','table_04')]:
  body={'action':'SUBMIT','robot_id':r,'request_id':'signals-'+uuid.uuid4().hex[:10],'mode':mode,'table':table};bodies[r]=body
  with concurrent.futures.ThreadPoolExecutor(max_workers=8) as pool:replies=list(pool.map(post,[body]*12))
  record('CONCURRENT_SUBMIT',replies);report[r]={'id':body['request_id'],'one_creation':sum(x.get('new') is True for x in replies)==1,'all_accepted':all(x.get('ok') for x in replies)}
  bad=post({**body,'table':'table_01'});report[r]['conflict_rejected']=not bad.get('ok');record('CONFLICT',bad)
 confirmed=set();terminal={}
 while time.monotonic()-start<360:
  status=get();record('STATUS',status)
  for r,body in bodies.items():
   s=status.get('supervisors',{}).get(r,{})
   if s.get('request_id')!=body['request_id']:continue
   if s.get('state')=='WAIT_HANDOVER' and s.get('confirmation_pose_valid') and r not in confirmed:
    action='COLLECTED' if body['mode']=='TABLE_COLLECT' else 'COMPLETE';signal={'action':action,'request_id':body['request_id'],'source':'table'}
    replies=[post(signal),post(signal)];record('CONFIRM_RETRY',replies);report[r]['confirmation_idempotent']=all(x.get('ok') for x in replies);confirmed.add(r)
   if s.get('state') in ('COMPLETED','ERROR','CANCELED'):terminal[r]=s
  if len(terminal)==2:break
  time.sleep(.3)
 before=get();record('POST_COMPLETE_BEGIN',before)
 for _ in range(20):time.sleep(.5);record('POST_COMPLETE_HOLD',get())
 after=get()
 for r in bodies:
  a=before.get('supervisors',{}).get(r,{});b=after.get('supervisors',{}).get(r,{})
  pa=before.get('robot_poses',{}).get(r);pb=after.get('robot_poses',{}).get(r)
  report[r].update({'completed':b.get('state')=='COMPLETED','no_followup_goal':a.get('goal_publish_count')==b.get('goal_publish_count'),'post_complete_displacement_m':math.dist(pa,pb) if pa and pb else None,'supervisor':b})
 report['success']=all(v.get('one_creation') and v.get('conflict_rejected') and v.get('completed') and v.get('no_followup_goal') and v.get('post_complete_displacement_m',1.)<.02 for k,v in report.items() if k in bodies)
finally:
 f.close();log.with_suffix('.summary.json').write_text(json.dumps(report,indent=2));print(json.dumps(report,indent=2),flush=True)
