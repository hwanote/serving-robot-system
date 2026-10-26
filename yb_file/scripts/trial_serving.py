"""Run real Fleet/Nav2 missions; only simulate the two operator confirmations.

Success requires supervisor COMPLETED after HOME, never merely accepting a request.
"""
import argparse,json,time,urllib.request,uuid
from pathlib import Path
ap=argparse.ArgumentParser();ap.add_argument('--mode',default='DELIVER_DIRECT',choices=['DELIVER_DIRECT','KITCHEN_PICKUP']);ap.add_argument('--timeout',type=float,default=300);ap.add_argument('--port',type=int,default=8765);ap.add_argument('--cross',action='store_true');ap.add_argument('--north',action='store_true');args=ap.parse_args()
root=Path(__file__).resolve().parents[1];out=root/'_local_only';out.mkdir(exist_ok=True)
def get(path):
    with urllib.request.urlopen(f'http://127.0.0.1:{args.port}'+path,timeout=4) as res:return json.load(res)
def post(data):
    req=urllib.request.Request(f'http://127.0.0.1:{args.port}/api/request',json.dumps(data).encode(),{'Content-Type':'application/json'})
    with urllib.request.urlopen(req,timeout=4) as res:reply=json.load(res)
    if not reply.get('ok'):raise RuntimeError(reply)
    return reply
ids={r:'trial-'+uuid.uuid4().hex[:12] for r in ('robot_01','robot_02')}
log=out/('serving_trial_'+time.strftime('%Y%m%d_%H%M%S')+'.jsonl');terminal={};confirmed=set();start=time.monotonic();previous={}
with log.open('w') as f:
    for r,table in ([('robot_01','table_01'),('robot_02','table_02')] if args.north else [('robot_01','table_04'),('robot_02','table_03')] if args.cross else [('robot_01','table_03'),('robot_02','table_04')]):
        print(post({'action':'SUBMIT','request_id':ids[r],'robot_id':r,'table':table,'mode':args.mode}),flush=True)
    while time.monotonic()-start<args.timeout:
        status=get('/api/status');f.write(json.dumps({'elapsed_s':time.monotonic()-start,'fleet':status})+'\n');f.flush()
        for r,s in status.get('supervisors',{}).items():
            if previous.get(r)!=s.get('state'):print(r,s,flush=True);previous[r]=s.get('state')
            if s.get('request_id')!=ids[r]:continue
            st=s.get('state')
            if st in ('WAIT_LOAD','WAIT_HANDOVER') and (r,st) not in confirmed:
                cmd='LOAD' if st=='WAIT_LOAD' else 'COMPLETE'
                post({'action':cmd,'request_id':ids[r],'source':'kitchen' if cmd=='LOAD' else 'table'});confirmed.add((r,st))
            if st in ('COMPLETED','ERROR','CANCELED'):terminal[r]=s
        if len(terminal)==2:break
        time.sleep(.5)
report={'elapsed_s':time.monotonic()-start,'operator_confirmations_simulated':True,'request_ids':ids,'results':terminal,'success':len(terminal)==2 and all(s['state']=='COMPLETED' for s in terminal.values()),'log':str(log)}
log.with_suffix('.summary.json').write_text(json.dumps(report,indent=2));print(json.dumps(report,indent=2),flush=True)
raise SystemExit(0 if report['success'] else 1)
