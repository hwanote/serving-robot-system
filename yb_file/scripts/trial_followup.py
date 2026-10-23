"""Continue explicitly named trial requests after the first observer times out."""
import argparse,json,time,urllib.request
from pathlib import Path
ap=argparse.ArgumentParser();ap.add_argument('--request-id',action='append',required=True);ap.add_argument('--timeout',type=float,default=240);args=ap.parse_args()
root=Path(__file__).resolve().parents[1];out=root/'_local_only';seen=set();start=time.monotonic();records=[]
def get():
    with urllib.request.urlopen('http://127.0.0.1:8765/api/status',timeout=5) as res:return json.load(res)
def post(data):
    req=urllib.request.Request('http://127.0.0.1:8765/api/request',json.dumps(data).encode(),{'Content-Type':'application/json'})
    with urllib.request.urlopen(req,timeout=5) as res:return json.load(res)
while time.monotonic()-start<args.timeout:
    status=get();records.append({'elapsed':time.monotonic()-start,'status':status})
    terminal=[]
    for r,s in status['supervisors'].items():
        rid=s.get('request_id')
        if rid not in args.request_id:continue
        st=s.get('state')
        if st in ('WAIT_LOAD','WAIT_HANDOVER') and (rid,st) not in seen:
            action='LOAD' if st=='WAIT_LOAD' else 'COMPLETE'
            reply=post({'action':action,'source':'kitchen' if action=='LOAD' else 'table','request_id':rid});print(reply,flush=True)
            if reply.get('ok'):seen.add((rid,st))
        if st in ('COMPLETED','CANCELED','ERROR'):terminal.append(s)
    if len(terminal)==len(args.request_id):break
    time.sleep(.5)
path=out/('trial_followup_'+time.strftime('%Y%m%d_%H%M%S')+'.json');path.write_text(json.dumps(records,indent=2));print(json.dumps(status,indent=2),flush=True)
raise SystemExit(0 if len(terminal)==len(args.request_id) and all(s['state']=='COMPLETED' for s in terminal) else 1)
