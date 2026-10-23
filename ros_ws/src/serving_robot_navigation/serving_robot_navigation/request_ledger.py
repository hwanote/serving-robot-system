"""Atomic durable request/Goal identity; callers never resend unknown active goals."""
import hashlib,json,os
from pathlib import Path
class RequestLedger:
    def __init__(self,path):
        self.path=Path(path)
        self.data=json.loads(self.path.read_text()) if self.path.exists() else {"requests":{},"active":None}
    def save(self):
        self.path.parent.mkdir(parents=True,exist_ok=True)
        temp=self.path.with_suffix(".tmp")
        with temp.open("w") as stream:
            json.dump(self.data,stream,sort_keys=True);stream.flush();os.fsync(stream.fileno())
        os.replace(temp,self.path)
    def check(self,request_id,command):
        digest=hashlib.sha256(json.dumps(command,sort_keys=True).encode()).hexdigest()
        old=self.data["requests"].get(request_id)
        if old and old["fingerprint"]!=digest:raise ValueError("request_id content conflict")
        return digest,old
    def accept(self,request_id,command):
        digest,old=self.check(request_id,command)
        if old:return old,False
        if self.data["active"]:raise ValueError("Navigation busy")
        record={"fingerprint":digest,"command":command,"status":"RUNNING","goal_publish_count":0,"detour_attempts":0,"failed_paths":[]}
        self.data["requests"][request_id]=record;self.data["active"]=request_id;self.save()
        return record,True
    def finish(self,request_id,status):
        self.data["requests"][request_id]["status"]=status
        if self.data["active"]==request_id:self.data["active"]=None
        self.save()
