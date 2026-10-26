"""Two robot dispatcher: idempotent requests, state confirmation, traffic control, local web UI."""
import json
import os
import time
import math
import sqlite3
import threading
import queue
from pathlib import Path
from http.server import ThreadingHTTPServer, BaseHTTPRequestHandler

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovarianceStamped
from .runtime_config import load_limits
from tf2_ros import Buffer,TransformListener
from rclpy.time import Time
import yaml
from .policy import Task, FleetCore, ROBOT_IDS, ACTIVE, distance, may_confirm, ZoneArbiter, in_bounds


def jparse(text):
    try: return json.loads(text)
    except (ValueError,TypeError): return {}

class Fleet(Node):
    def __init__(self):
        super().__init__('cafe_fleet')
        self.declare_parameter('database_path',str(Path.home()/'cafe_autonomy_fleet.sqlite3'))
        self.declare_parameter('http_port',8765)
        self.declare_parameter('runtime_limits_file','')
        self.declare_parameter('zones_file','')
        self.limits=load_limits(self.get_parameter('runtime_limits_file').value)
        zone_file=self.get_parameter('zones_file').value
        zone_map=yaml.safe_load(Path(zone_file).read_text(encoding='utf8'))['zones'] if zone_file else {}
        if not zone_map:raise ValueError('Missing Fleet zones from SETUP contract')
        for name,b in zone_map.items():
            if not isinstance(b,list) or len(b)!=4 or not (b[0]<b[1] and b[2]<b[3]):
                raise ValueError('Invalid zone rectangle '+name)
        self.core=FleetCore();self.zones=ZoneArbiter(zones=zone_map); self.status={r:{'state':'UNKNOWN'} for r in ROBOT_IDS}
        self.tf=Buffer();self.listener=TransformListener(self.tf,self)
        self.status_wall={}
        self.poses={};self.pose_wall={};self.odom_wall={};self.scan_wall={}
        self.safety={};self.last_error=None;self.dispatch_started={};self.replies={};self.terminal_seen=set()
        self.traffic={r:True for r in ROBOT_IDS}; self.person_zones={}
        self.recovery=None;self.zone_config=yaml.safe_load(Path(zone_file).read_text())
        self.blocked_since={};self.block_alerted=set()
        self.commands={r:self.create_publisher(String,f'/{r}/serving/request',10) for r in ROBOT_IDS}
        self.complete={r:self.create_publisher(String,f'/{r}/serving/complete',10) for r in ROBOT_IDS}
        self.holds={r:self.create_publisher(Bool,f'/{r}/traffic_hold',10) for r in ROBOT_IDS}
        self.pub=self.create_publisher(String,'/fleet/status',10)
        self.reply_pub=self.create_publisher(String,'/fleet/reply',10)
        self.create_subscription(String,'/fleet/request',self.on_request,10)
        for r in ROBOT_IDS:
            self.create_subscription(String,f'/{r}/serving/status',lambda m,bot=r:self.on_status(bot,m),10)
            self.create_subscription(String,f'/{r}/navigation_event',lambda m,bot=r:self.on_navigation(bot,m),10)
            self.create_subscription(String,f'/{r}/dynamic_safety',lambda m,bot=r:self.dynamic.update({bot:jparse(m.data)}),10)
            self.create_subscription(String,f'/{r}/dynamic_event',lambda m,bot=r:self.on_dynamic_event(bot,m),20)
            self.create_subscription(Odometry,f'/{r}/odom',lambda m,bot=r:self.on_odom(bot,m),10)
            self.create_subscription(PoseWithCovarianceStamped,f'/{r}/amcl_pose',lambda m,bot=r:self.on_amcl(bot,m),10)
            # The safety node publishes regular heartbeats, distinct from geometric scan distances.
            self.create_subscription(String,f'/{r}/safety_status',lambda m,bot=r:self.on_safety(bot,m),10)
        dbpath=self.get_parameter('database_path').value
        Path(dbpath).parent.mkdir(parents=True,exist_ok=True)
        self.db=sqlite3.connect(dbpath,check_same_thread=False)
        self.db.execute('CREATE TABLE IF NOT EXISTS events (id INTEGER PRIMARY KEY AUTOINCREMENT, stamp REAL, event TEXT, payload TEXT)')
        self.db.execute('CREATE TABLE IF NOT EXISTS requests (request_id TEXT PRIMARY KEY, fingerprint TEXT NOT NULL, task TEXT NOT NULL)')
        self.db.execute('CREATE TABLE IF NOT EXISTS signals (request_id TEXT, action TEXT, fingerprint TEXT, reply TEXT, PRIMARY KEY(request_id,action))')
        if 'applied' not in [r[1] for r in self.db.execute('PRAGMA table_info(signals)')]:self.db.execute('ALTER TABLE signals ADD COLUMN applied INTEGER NOT NULL DEFAULT 0')
        self.db.commit();self.database_lock=threading.Lock();self.signal_sent_wall={}
        for rid,fingerprint,payload in self.db.execute('SELECT request_id,fingerprint,task FROM requests'):
            task=Task(**json.loads(payload))
            if task.state in ('ASSIGNED','RUNNING'):task.state='RECOVERY_REQUIRED'
            self.core.tasks[rid]=task
            if task.state=='QUEUED':self.core.queue.append(rid)
        self.dynamic={};self.dynamic_events={};self.navigation_events={}
        self.http_queue=queue.Queue();self.latest_status={};self.snapshot_lock=threading.Lock()
        self.http_port=int(self.get_parameter('http_port').value)
        self.start_http()
        self.create_timer(0.25,self.tick)
        self.get_logger().info('Fleet started; requests require a matching supervisor IDLE heartbeat')

    def record(self,event,payload):
        with self.database_lock:
            self.db.execute('INSERT INTO events(stamp,event,payload) VALUES (?,?,?)',(time.time(),event,json.dumps(payload,ensure_ascii=False)))
            self.db.commit()

    def publish_msg(self,publisher,data):
        m=String();m.data=json.dumps(data,ensure_ascii=False);publisher.publish(m)

    def save_task(self,task):
        with self.database_lock:
            self.db.execute('UPDATE requests SET task=? WHERE request_id=?',(json.dumps(vars(task)),task.request_id));self.db.commit()
    def on_dynamic_event(self,robot,msg):
        evt=jparse(msg.data);self.dynamic_events[robot]=evt;self.record('DYNAMIC_EVENT',evt)
    def on_status(self,robot,msg):
        obj=jparse(msg.data)
        if obj.get('robot_id')!=robot:return
        self.status[robot]=obj;self.status_wall[robot]=time.monotonic()
        rid=obj.get('request_id');task=self.core.tasks.get(rid)
        if task and task.state=='RECOVERY_REQUIRED' and task.robot==robot and obj.get('state') not in ('IDLE','UNKNOWN'):
            self.core.active[robot]=rid
        before=task.state if task else None
        self.core.update(robot,obj.get('state',''),rid)
        if rid:
            with self.database_lock:
                for action in obj.get('accepted_signals',[]):self.db.execute('UPDATE signals SET applied=1 WHERE request_id=? AND action=? AND applied=0',(rid,action))
                self.db.commit()
        if task and task.state!=before:self.save_task(task)
        rid=obj.get('request_id')
        if obj.get('state') in ('COMPLETED','CANCELED','ERROR') and rid and (rid,obj['state']) not in self.terminal_seen:
            self.terminal_seen.add((rid,obj['state']))
            self.record('MISSION_TERMINAL',{'robot':robot,'request_id':rid,'state':obj['state'], 'reason':obj.get('reason')})


    def on_navigation(self,robot,msg):
        evt=jparse(msg.data);self.navigation_events[robot]=evt;self.record('NAVIGATION_EVENT',evt)
        rid=evt.get('request_id')
        if evt.get('event')=='MISSION_REJECTED' and rid and self.core.active.get(robot)==rid:
            self.core.tasks[rid].state='FAILED';self.save_task(self.core.tasks[rid])
            self.core.active.pop(robot,None)
            self.last_error='MISSION_REJECTED: '+str(evt.get('reason'))
            self.record('MISSION_REJECTED',evt)

    def on_odom(self,robot,msg):
        self.odom_wall[robot]=time.monotonic()

    def on_amcl(self,robot,msg):
        # AMCL is in map frame; robot-specific odom frames cannot be compared directly.
        if msg.header.frame_id!='map':return
        p=msg.pose.pose.position
        self.poses[robot]=(p.x,p.y)
        self.pose_wall[robot]=time.monotonic()

    def on_safety(self,robot,msg):
        obj=jparse(msg.data)
        self.safety[robot]=obj
        self.scan_wall[robot]=time.monotonic() if obj.get('sensor_fresh') and obj.get('detector_fresh') and obj.get('dynamic_fresh') else 0.0

    def on_request(self,msg):
        try:self.process(jparse(msg.data))
        except Exception as exc:self.get_logger().error('Unexpected request error: '+str(exc))

    def process(self,data):
        rid=data.get('request_id')
        action=data.get('action','SUBMIT')
        try:
            if action=='SUBMIT':
                robot=data.get('robot_id')
                if not robot:
                    if rid in self.core.tasks:robot=self.core.tasks[rid].robot
                    else:
                        eligible=[r for r in ROBOT_IDS if self.status[r].get('state') not in ('ERROR','UNKNOWN')]
                        if not eligible:raise ValueError('No available robot')
                        robot=min(eligible,key=lambda r:sum(t.robot==r and t.state in ('QUEUED','RUNNING','ASSIGNED') for t in self.core.tasks.values()))
                mode=data.get('mode','KITCHEN_PICKUP');table=data.get('table') if mode not in ('GO_HOME','GO_KITCHEN') else None
                if mode not in ('GO_HOME','GO_KITCHEN') and table not in ('table_01','table_02','table_03','table_04'):
                    raise ValueError('Unknown table')
                fingerprint=json.dumps({'robot_id':data.get('robot_id') or None,'table':table,'mode':mode,'side':data.get('side','auto')},sort_keys=True)
                with self.database_lock:
                    old=self.db.execute('SELECT fingerprint FROM requests WHERE request_id=?',(rid,)).fetchone()
                    if old and old[0]!=fingerprint:raise ValueError('request_id content conflict')
                    task,new=self.core.submit(rid,table,robot,mode,data.get('side','auto'))
                    if new:
                        self.db.execute('INSERT INTO requests VALUES (?,?,?)',(rid,fingerprint,json.dumps(vars(task))));self.db.commit()
                if new:self.record('TASK_QUEUED',vars(task))
                reply={'ok':True,'action':action,'request_id':rid,'state':task.state, 'new':new,'robot_id':task.robot}
            elif action=='RECOVER':
                task=self.core.tasks.get(rid)
                if not task:raise ValueError('Unknown request_id')
                robot=task.robot;state=self.status[robot]
                if task.state=='RECOVERY_REQUIRED' and state.get('state')=='IDLE' and state.get('request_id') is None:
                    if not state.get('goal_execution_verified_idle'):raise ValueError('Goal idle not verified')
                    task.state='QUEUED';self.core.queue.append(rid);self.save_task(task)
                else:
                    if state.get('request_id')!=rid or state.get('state') not in ('ERROR','RECOVERY_WAIT'):raise ValueError('Recovery does not match current failed mission')
                    task.state='RECOVERY_REQUIRED';self.core.active[robot]=rid;self.save_task(task)
                    self.publish_msg(self.commands[robot],{'command':'RECOVER','request_id':rid,'robot_id':robot})
                reply={'ok':True,'action':action,'request_id':rid,'robot_id':robot}
            elif action=='RESET':
                robot=data.get('robot_id')
                if robot not in ROBOT_IDS:raise ValueError('Unknown robot')
                if self.status[robot].get('state') not in ('ERROR','CANCELED','COMPLETED') or robot in self.core.active:
                    raise ValueError('Robot has not reached a terminal state')
                self.publish_msg(self.commands[robot],{'command':'RESET','robot_id':robot})
                self.record('ROBOT_RESET_REQUEST',{'robot':robot})
                reply={'ok':True,'action':'RESET','robot_id':robot,'request_id':rid}
            elif action in ('LOAD','COMPLETE','COLLECTED','CANCEL'):
                task=self.core.tasks.get(rid)
                if task is None:raise ValueError('Unknown request_id')
                robot=task.robot;supervisor=self.status[robot];state=supervisor.get('state')
                if 'table' in data and data['table']!=task.table:raise ValueError('Confirmation table mismatch')
                fingerprint=json.dumps({'source':data.get('source'),'robot_id':data.get('robot_id') or robot},sort_keys=True)
                with self.database_lock:old=self.db.execute('SELECT fingerprint,reply FROM signals WHERE request_id=? AND action=?',(rid,action)).fetchone()
                if old:
                    if old[0]!=fingerprint:raise ValueError('Confirmation content conflict')
                    reply=json.loads(old[1]);self.publish_msg(self.reply_pub,reply);return reply
                if action in ('LOAD','COMPLETE','COLLECTED') and supervisor.get('request_id')==rid and action in supervisor.get('accepted_signals',[]):
                    if data.get('source')!=('kitchen' if action=='LOAD' else 'table'):raise ValueError('Wrong confirmation origin')
                    reply={'ok':True,'action':action,'request_id':rid,'robot':robot}
                    with self.database_lock:
                        self.db.execute('INSERT INTO signals(request_id,action,fingerprint,reply,applied) VALUES (?,?,?,?,1)',(rid,action,fingerprint,json.dumps(reply)));self.db.commit()
                    self.publish_msg(self.reply_pub,reply);return reply
                if action=='CANCEL':
                    if task.state=='QUEUED':
                        if rid in self.core.queue:self.core.queue.remove(rid)
                        task.state='CANCELED';self.save_task(task)
                    else:self.publish_msg(self.commands[robot],{'command':'CANCEL','request_id':rid})
                else:
                    if supervisor.get('request_id')!=rid or not supervisor.get('confirmation_pose_valid') or not may_confirm(action,state,task.mode):
                        raise ValueError('Robot not in corresponding confirmation state: '+str(state))
                    expected='kitchen' if action=='LOAD' else 'table'
                    if data.get('source')!=expected:raise ValueError('Wrong confirmation origin')
                    self.publish_msg(self.complete[robot],{'command':action,'request_id':rid})
                self.record('TASK_COMMAND',{'request_id':rid,'action':action,'robot':robot})
                reply={'ok':True,'action':action,'request_id':rid,'robot':robot}
                with self.database_lock:
                    self.db.execute('INSERT INTO signals(request_id,action,fingerprint,reply) VALUES (?,?,?,?)',(rid,action,fingerprint,json.dumps(reply)));self.db.commit()
            else:raise ValueError('Unknown action')
            self.last_error=None
        except (ValueError,KeyError) as exc:
            self.last_error=str(exc)
            reply={'ok':False,'action':action,'request_id':rid,'error':self.last_error}
            self.get_logger().warning('Request rejected: '+self.last_error)
        self.publish_msg(self.reply_pub,reply)
        self.record('FLEET_REPLY',reply)
        return reply

    def kitchen_busy(self):
        # Include newly ASSIGNED jobs before a mission-status heartbeat arrives.
        for r,rid in self.core.active.items():
            task=self.core.tasks.get(rid)
            if task and task.mode in ('KITCHEN_PICKUP','GO_KITCHEN') and (task.state=='ASSIGNED' or self.status[r].get('state') in ('GOING_PICKUP','GOING_KITCHEN','WAIT_LOAD')):
                return True
        return False

    def tick(self):
        while True:
            try:body,resq=self.http_queue.get_nowait()
            except queue.Empty:break
            resq.put(self.process(body))
        now=time.monotonic()
        # Persisted confirmations are retried until Supervisor records application.
        # Its own durable command guard makes this safe across reconnect/restart.
        with self.database_lock:pending_signals=self.db.execute('SELECT request_id,action FROM signals WHERE applied=0').fetchall()
        for rid,action in pending_signals:
            if action not in ('LOAD','COMPLETE','COLLECTED'):continue
            task=self.core.tasks.get(rid)
            if not task:continue
            current=self.status[task.robot]
            if current.get('request_id')!=rid or not current.get('confirmation_pose_valid') or not may_confirm(action,current.get('state'),task.mode):continue
            key=(rid,action)
            if now-self.signal_sent_wall.get(key,0.)>=1.:
                self.signal_sent_wall[key]=now;self.publish_msg(self.complete[task.robot],{'command':action,'request_id':rid});self.record('CONFIRM_OUTBOX_RETRY',{'request_id':rid,'robot_id':task.robot,'action':action})
        for r in ROBOT_IDS:
            try:
                t=self.tf.lookup_transform('map',r+'/base_footprint',Time())
                age=(self.get_clock().now().nanoseconds-(t.header.stamp.sec*10**9+t.header.stamp.nanosec))/1e9
                if age<1.:
                    self.poses[r]=(t.transform.translation.x,t.transform.translation.y);self.pose_wall[r]=now
            except Exception:pass
        for r in ROBOT_IDS:
            supervisor_fresh=now-self.status_wall.get(r,-1e9)<2.
            pose_valid=supervisor_fresh and (now-self.pose_wall.get(r,-1e9) < 2.0 and now-self.odom_wall.get(r,-1e9)<1.0)
            scan_valid=(now-self.scan_wall.get(r,-1e9) < 1.5)
            self.traffic[r]=not (pose_valid and scan_valid)
        # Fleet-owned zone arbitration and collision separation.
        current_map_poses={r:p for r,p in self.poses.items() if now-self.pose_wall.get(r,-1e9)<2.0}
        traffic_conflict=set(self.zones.update(self.poses,self.limits['fleet']['zone_approach_margin_m']))
        for r in traffic_conflict:self.traffic[r]=True
        # One robot yields when close. Do not use last scan range as a person proxy.
        if all(r in current_map_poses for r in ROBOT_IDS):
            if distance(current_map_poses['robot_01'],current_map_poses['robot_02'])<self.limits['fleet']['robot_separation_m']:
                self.traffic['robot_02']=True;traffic_conflict.add('robot_02')
        if self.recovery:
            loser=self.recovery['loser'];winner=self.recovery['winner']
            st=self.status[loser].get('state')
            safe=(now-self.pose_wall.get(loser,-1e9)<2. and now-self.scan_wall.get(loser,-1e9)<1.5)
            if st=='YIELDING':
                self.traffic[loser]=not safe;self.traffic[winner]=True
            elif st=='YIELD_WAIT':
                self.traffic[loser]=True
                occupied=any(in_bounds(current_map_poses.get(winner),b,.45) for b in self.zones.zones.values())
                if not occupied and safe:
                    self.publish_msg(self.commands[loser],{'command':'RESUME','robot_id':loser})
                    self.record('YIELD_RECOVERY_COMPLETE',self.recovery);self.recovery=None;self.last_error=None
            elif st in ('ERROR','CANCELED','COMPLETED') or now-self.recovery['started']>self.limits['navigation']['goal_timeout_s']:
                self.record('YIELD_RECOVERY_FAILED',self.recovery);self.recovery=None
        for r in ROBOT_IDS:
            if r in traffic_conflict and self.traffic[r] and self.status[r].get('state') in ('GOING_PICKUP','GOING_TABLE','GOING_DIRECT','GOING_KITCHEN','RETURNING_HOME'):
                self.blocked_since.setdefault(r,now)
                if (r not in self.block_alerted and now-self.blocked_since[r] >= self.limits['fleet']['blocked_alert_s']):
                    self.block_alerted.add(r)
                    detail={'robot':r,'seconds':round(now-self.blocked_since[r],2),
                            'reason':'YIELD_RECOVERY_REQUESTED'}
                    self.record('FLEET_TRAFFIC_BLOCKED',detail)
                    if self.recovery is None and all(bot in current_map_poses for bot in ROBOT_IDS):
                        loser=r;winner=next(bot for bot in ROBOT_IDS if bot!=loser)
                        self.recovery={'loser':loser,'winner':winner,'started':now}
                        self.publish_msg(self.commands[loser],{'command':'YIELD','robot_id':loser})
                        self.record('YIELD_RETREAT_REQUEST',self.recovery)
                    self.last_error='Fleet traffic hold exceeded configured limit: '+r
            else:
                self.blocked_since.pop(r,None);self.block_alerted.discard(r)
            self.holds[r].publish(Bool(data=bool(self.traffic[r])))
        ready={r for r in ROBOT_IDS if self.status[r].get('state') in ('IDLE','COMPLETED','CANCELED') and not self.traffic[r] and self.status[r].get('map_valid') is True and self.status[r].get('nav_ready') is True and self.commands[r].get_subscription_count()>0}
        task=self.core.dispatch_next(ready,kitchen_free=not self.kitchen_busy())
        if task:
            self.save_task(task)
            self.publish_msg(self.commands[task.robot],{'command':'START','request_id':task.request_id,
                'robot_id':task.robot,'table':task.table,'mode':task.mode,'side':task.side})
            self.record('TASK_ASSIGNED',vars(task))
        snapshot=self.core.snapshot()
        snapshot.update({'supervisors':self.status,'robot_poses':self.poses,'traffic_hold':self.traffic,
                         'last_error':self.last_error,'http_port':self.http_port,'safety':self.safety,'zone_owner':dict(self.zones.owners),'recovery':self.recovery,'dynamic':self.dynamic,'dynamic_events':self.dynamic_events,'navigation_events':self.navigation_events})
        self.publish_msg(self.pub,snapshot)
        with self.snapshot_lock:self.latest_status=snapshot

    def start_http(self):
        port=self.http_port; parent=self
        web=Path(__file__).resolve().parent.parent/'web'/'dashboard.html'
        # In symlink or installed builds, locate the web artifact in the package share.
        try:
            from ament_index_python.packages import get_package_share_directory
            web=Path(get_package_share_directory('cafe_autonomy'))/'web'/'dashboard.html'
        except Exception:pass
        class Handler(BaseHTTPRequestHandler):
            def respond(self,obj,code=200):
                data=json.dumps(obj,ensure_ascii=False,default=str).encode()
                self.send_response(code);self.send_header('Content-Type','application/json; charset=utf-8')
                self.send_header('Content-Length',str(len(data)));self.end_headers();self.wfile.write(data)
            def do_GET(self):
                if self.path in ('/','/dashboard'):
                    if not web.exists():return self.respond({'error':'dashboard missing'},404)
                    data=web.read_bytes();self.send_response(200)
                    self.send_header('Content-Type','text/html; charset=utf-8')
                    self.send_header('Content-Length',str(len(data)));self.end_headers();self.wfile.write(data)
                elif self.path=='/api/status':
                    with parent.snapshot_lock:data=dict(parent.latest_status)
                    self.respond(data)
                elif self.path=='/api/history':
                    with parent.database_lock:
                        rows=parent.db.execute('SELECT stamp,event,payload FROM events ORDER BY id DESC LIMIT 100').fetchall()
                    self.respond([{'time':r[0],'event':r[1],'payload':jparse(r[2])} for r in rows])
                elif self.path=='/health':self.respond({'ok':True})
                else:self.respond({'error':'not found'},404)
            def do_POST(self):
                if self.path!='/api/request':return self.respond({'error':'not found'},404)
                try:
                    n=int(self.headers.get('Content-Length','0'))
                    if n<=0 or n>16384:raise ValueError('Invalid request size')
                    payload=json.loads(self.rfile.read(n))
                    q=queue.Queue(maxsize=1);parent.http_queue.put((payload,q))
                    self.respond(q.get(timeout=3))
                except Exception as ex:self.respond({'ok':False,'error':str(ex)},400)
            def log_message(self,*args):pass
        try:
            server=ThreadingHTTPServer(('127.0.0.1',port),Handler)
            threading.Thread(target=server.serve_forever,daemon=True).start()
        except OSError as exc:
            self.get_logger().error('HTTP dashboard disabled: '+str(exc))

def main():
    rclpy.init();node=Fleet()
    try:rclpy.spin(node)
    finally:node.destroy_node();rclpy.shutdown()
