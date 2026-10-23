"""On-demand pedestrians: separate Gazebo models, no writes to robot cmd_vel.

Commands: /person_simulator/command (std_msgs/String containing JSON).
Built-in safeguards gate motion on recent *world-ground-truth* robot + person poses.
"""
import fcntl
import json
import math
import os
from pathlib import Path
import random
import secrets
import time

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from std_msgs.msg import String, Empty
import yaml

from .gazebo_api import spawn, remove
from .pose_cli import PoseStream
from .policy import (PersonRules, Walker, apply_deadlock_overlay, decide_action, draw_speed,
                     next_time, validate, validate_spawn_positions)


def xy(transform):
    return (transform.transform.translation.x, transform.transform.translation.y)


from .policy import can_escape_person_hold

class Manager(Node):
    def __init__(self):
        super().__init__('person_simulator_manager')
        self.declare_parameter('config_path', '')
        config_path = self.get_parameter('config_path').value
        if not config_path:
            raise RuntimeError('config_path is required')
        with open(config_path, encoding='utf-8') as f:
            self.cfg=validate(yaml.safe_load(f))
        # Optional overlay: may ONLY set the two deadlock keys (never speeds/radii/lanes).
        self.declare_parameter('overlay_config_path', '')
        overlay_path = self.get_parameter('overlay_config_path').value
        if overlay_path:
            with open(overlay_path, encoding='utf-8') as f:
                self.cfg = apply_deadlock_overlay(self.cfg, yaml.safe_load(f))
        self.rules = PersonRules(self.cfg)
        self.world=self.cfg['world']
        self.prefix=self.cfg['model_prefix']
        self.robot_names=self.cfg.get('robot_models',[self.cfg['robot_model']])
        if not isinstance(self.robot_names,list) or not self.robot_names or any(
            r not in ('robot_01','robot_02') for r in self.robot_names):
            raise RuntimeError('robot_models must contain robot_01/robot_02')
        if self.prefix != 'test_person_':
            raise RuntimeError('model_prefix must remain test_person_ for ownership safety')
        self.seed = self.cfg['seed'] if self.cfg['seed'] is not None else secrets.randbits(63)
        self.rng = random.Random(self.seed)
        self.people = {}  # id -> Walker
        self.cmd_pubs = {
            i: self.create_publisher(Twist, f'/model/{self.prefix}{i:02d}/cmd_vel', 10)
            for i in range(1,self.cfg['people_max']+1)
        }
        self.poses = {}
        self.pose_received_at = {}
        self.main_paused = False
        self.shutdown_requested = False
        self.last_error = None
        self.last_sim_time = None
        self.last_print_state = None
        # Raw Gazebo model pose names are preserved by the CLI; the Pose_V ->
        # TFMessage bridge in this environment drops child_frame_id names.
        self.pose_stream = PoseStream(self.world, self.prefix, self.robot_names)
        self.create_subscription(String, '/person_simulator/command', self.on_command, 10)
        self.status_pub=self.create_publisher(String, '/person_simulator/status', 10)
        self.heartbeat_pub=self.create_publisher(Empty, '/person_simulator/heartbeat', 10)
        self.create_timer(1.0 / self.cfg['control_rate_hz'],self.tick)
        self.create_timer(1.0,self.publish_status)
        folder=Path.home()/'.ros'/'person_simulator'
        folder.mkdir(parents=True, exist_ok=True)
        self.log_path=folder/f'run_{time.strftime("%Y%m%d_%H%M%S")}_{os.getpid()}.jsonl'
        self.log_file=self.log_path.open('w',encoding='utf-8',buffering=1)
        self.record('startup',seed=self.seed,world=self.world,deadlock_resolution=self.rules.enabled)
        self.get_logger().info(f'Ready: seed={self.seed}. No humans spawned by default. Configured lanes={len(self.cfg["lanes"])}')

    def sim_now(self):
        return self.get_clock().now().nanoseconds/1e9

    def record(self,event,**fields):
        if hasattr(self,'log_file') and not self.log_file.closed:
            self.log_file.write(json.dumps({'sim_s':round(self.sim_now(),3),
              'event':event,**fields},ensure_ascii=False)+'\n')

    def refresh_poses(self):
        # No fallback to anonymous TF transforms or odom coordinates.
        for name, (point, received) in self.pose_stream.snapshot().items():
            if received > self.pose_received_at.get(name, -1):
                self.poses[name] = point
                self.pose_received_at[name] = received

    def fresh_pose(self,name):
        self.refresh_poses()
        received=self.pose_received_at.get(name)
        if received is None or time.monotonic()-received > self.cfg['pose_timeout_s']:
            return None
        p=self.poses.get(name)
        return p if p is not None and all(math.isfinite(v) for v in p) else None

    def command_error(self,reason):
        self.last_error = reason
        self.get_logger().error(reason)
        self.record('command_error',reason=reason)
        self.publish_status(error=reason)

    def on_command(self,msg):
        try:
            data=json.loads(msg.data)
            if not isinstance(data,dict):
                raise ValueError('command must be a JSON object')
            action=data.get('action')
            if action in ('spawn','delete','start','stop','resume','pause','unpause','cleanup','shutdown','recover','status'):
                self.process_command(data)
                self.last_error = None
                self.publish_status()
            else:
                raise ValueError(f'unknown action {action!r}')
        except Exception as exc:
            self.command_error(str(exc))

    def _ids(self,data,allow_empty=False):
        if 'id' not in data:
            return list(self.people)
        ident=data['id']
        if type(ident) is not int or not 1<=ident<=self.cfg['people_max']:
            raise ValueError('id must be an integer 1..8')
        if ident not in self.people and not allow_empty:
            raise ValueError(f'person {ident} is not owned/active')
        return [ident]

    def _spawn_one(self,ident,start,end):
        name=f'{self.prefix}{ident:02d}'
        if self.fresh_pose(name) is not None:
            raise ValueError(f'orphan / duplicate exists: {name}; use recover explicitly')
        spawn(self.world,name,*start,self.cfg['person_radius_m'],self.cfg['person_height_m'])
        # A true service response means the request was accepted, NOT that the
        # world entity exists. Gazebo ground-truth confirmation is mandatory.
        deadline = time.monotonic() + 6.0
        confirmed = None
        while time.monotonic() < deadline:
            self.heartbeat_pub.publish(Empty())
            confirmed = self.fresh_pose(name)
            if confirmed is not None:
                break
            time.sleep(0.1)
        if confirmed is None or math.dist(confirmed, start) > 0.4:
            # Clean up any late / misplaced entity; never register a ghost.
            try:
                remove(self.world, name)
            except Exception as exc:
                self.get_logger().warning(f'Spawn cleanup could not remove {name}: {exc}')
            raise RuntimeError(
                f'Gazebo spawn not confirmed for {name}; '
                f'pose={confirmed!r}, expected={start!r}'
            )
        _, speed=draw_speed(self.rng,self.cfg)
        w=Walker(ident,name,start,end,speed_mps=speed,paused=True,
                 next_decision=next_time(self.rng,self.cfg,self.sim_now()))
        self.people[ident]=w
        self.record('spawn',id=ident,name=name,x=start[0],y=start[1])

    def process_command(self,data):
        action=data['action']
        if action=='spawn':
            count=data.get('count',1)
            if type(count) is not int or count<1 or count>8:
                raise ValueError('count must be 1..8')
            available=[i for i in range(1,9) if i not in self.people]
            if count > len(available):
                raise ValueError('count exceeds available person slots')
            # All positions preflighted before the first spawn: no partial batch
            # if one future slot collides with an existing/other future person.
            if len(self.cfg['lanes']) < max(available[:count]):
                raise ValueError('Not enough reviewed lanes in config/policy.yaml; no automatic spawn')
            plans=[]
            for ident in available[:count]:
                lane=self.cfg['lanes'][ident-1]
                plans.append((tuple(lane['start']),tuple(lane['end'])))
            # Validate all scheduled people against EVERY active robot, not just robot01.
            for robot_name in self.robot_names:
                robot_pose=self.fresh_pose(robot_name)
                if robot_pose is None:
                    raise RuntimeError('Gazebo robot pose unavailable: '+robot_name)
                validate_spawn_positions(
                    plans,robot_pose,
                    [self.fresh_pose(w.name) for w in self.people.values()],
                    self.cfg['robot_release_radius_m'],
                    self.cfg['person_release_radius_m'])
            created=[]
            try:
                for ident,(start,end) in zip(available[:count],plans):
                    self._spawn_one(ident,start,end)
                    created.append(ident)
            except Exception:
                for ident in reversed(created):
                    try:
                        self.delete_person(ident)
                    except Exception as exc:
                        self.get_logger().error(f'Rollback failed for {ident}: {exc}')
                raise
            return
        if action=='recover':
            if self.people:
                raise ValueError('delete owned people before recovering orphans')
            # Never remove arbitrary models; only previously known unique owned prefix.
            for i in range(1,9):
                name=f'{self.prefix}{i:02d}'
                if self.fresh_pose(name) is not None:
                    remove(self.world,name)
                    self.record('recover_orphan',name=name)
            return
        if action in ('delete','cleanup'):
            ids=self._ids(data) if action=='delete' else list(self.people)
            for ident in ids:
                self.delete_person(ident)
            return
        if action=='status':
            self.publish_status()
            return
        if action=='shutdown':
            self.shutdown_requested=True
            return
        if action=='pause':
            self.main_paused=True
            self.stop_all()
            self.record('global_pause')
            return
        if action=='unpause':
            self.main_paused=False
            self.record('global_unpause')
            return
        if action in ('stop','start','resume'):
            for ident in self._ids(data):
                w=self.people[ident]
                w.paused=(action=='stop')
                w.phase='moving'
                w.current_action='continue'
                w.next_decision=next_time(self.rng,self.cfg,self.sim_now())
                if w.paused:
                    self.send_zero(ident)
                self.record(action,id=ident)

    def delete_person(self,ident):
        w=self.people[ident]
        self.send_zero(ident)
        remove(self.world,w.name)
        self.record('delete',id=ident)
        del self.people[ident]

    def stop_all(self):
        for ident in self.people:
            self.send_zero(ident)

    def send_zero(self,ident):
        if rclpy.ok():
            self.cmd_pubs[ident].publish(Twist())
        if ident in self.people:
            self.people[ident].published_v=(0,0)

    def tick(self):
        self.heartbeat_pub.publish(Empty())
        now=self.sim_now()
        if now<=0:
            return
        if self.last_sim_time is None:
            self.last_sim_time=now;return
        dt=now-self.last_sim_time
        self.last_sim_time=now
        if dt<=0 or dt>0.5:
            self.stop_all();return
        robots=[self.fresh_pose(name) for name in self.robot_names]
        actual_positions={i:self.fresh_pose(w.name) for i,w in self.people.items()}
        for ident,w in list(self.people.items()):
            person=actual_positions[ident]
            if (any(p is None for p in robots) or person is None or self.main_paused or w.paused):
                self.send_zero(ident)
                continue
            # Absolute emergency proximity guard is NOT probabilistic.
            robot_dist=min(math.dist(person,r) for r in robots)
            if robot_dist <= self.cfg['robot_stop_radius_m']:
                w.proximity_hold=True
            elif robot_dist >= self.cfg['robot_release_radius_m']:
                w.proximity_hold=False
                w.yield_selected=None
            peers=[(j,q,self.people[j].published_v,self.people[j])
                   for j,q in actual_positions.items() if j!=ident and q is not None]
            if len(peers)!=max(0,len(self.people)-1):
                self.send_zero(ident);continue  # Any unknown person is unsafe.
            self.rules.begin(w,now,person,peers)
            # Yield decision cached per encounter, not rerolled each frame.
            if robot_dist<1.35 and w.yield_selected is None:
                w.yield_selected=self.rng.random()*100<self.cfg['behavior']['give_way_percent']
                self.record('give_way_choice',id=ident,choice='yield' if w.yield_selected else 'non_yield')
            should_yield=robot_dist<1.35 and w.yield_selected is True
            if w.proximity_hold or should_yield:
                self.send_zero(ident);continue
            if w.phase in ('stopped','gradual_pause','sudden_pause','long_block'):
                if now<w.until:
                    self.send_zero(ident);continue
                w.phase='moving';w.current_action='continue'
                _,w.speed_mps=draw_speed(self.rng,self.cfg)
                w.next_decision=next_time(self.rng,self.cfg,now)
                self.record('resume_auto',id=ident)
            if w.phase=='gradual_brake':
                duration=max(0.01,w.until-w.stop_start)
                remaining=max(0,(w.until-now)/duration)
                speed=w.decel_start_speed*remaining
                if now>=w.until:
                    w.phase='gradual_pause'
                    w.until=now+self.rng.uniform(*self.cfg['behavior']['gradual_pause_s'])
                    speed=0.0
                    self.record('gradual_stop_complete',id=ident)
            else:
                speed=w.speed_mps
                if now>=w.next_decision:
                    kind,duration=decide_action(self.rng,self.cfg,now,*person)
                    w.current_action=kind
                    self.record('action',id=ident,kind=kind)
                    w.next_decision=next_time(self.rng,self.cfg,now)
                    if kind=='gradual_stop':
                        w.phase='gradual_brake';w.stop_start=now
                        w.until=now+duration;w.decel_start_speed=speed
                    elif kind in ('sudden_stop','long_block'):
                        w.phase='sudden_pause' if kind=='sudden_stop' else 'long_block'
                        w.until=now+duration
                        speed=0.0
                    else:
                        _,w.speed_mps=draw_speed(self.rng,self.cfg)
                        speed=w.speed_mps
            if speed<=0:
                self.send_zero(ident);continue
            target=w.end if w.target_end else w.start
            dx,dy=target[0]-person[0],target[1]-person[1]
            length=math.hypot(dx,dy)
            if length<0.12:
                w.target_end=not w.target_end
                self.send_zero(ident)
                self.record('turnaround',id=ident)
                continue
            vx=speed*dx/length;vy=speed*dy/length
            if self.rules.hold_blocked(w, person, (vx, vy), peers):
                self._person_block(ident, w, now, person, peers)
                self.send_zero(ident)
                continue
            projected=(person[0]+vx*max(dt,0.1),person[1]+vy*max(dt,0.1))
            if any(math.dist(projected, r) < self.cfg['robot_stop_radius_m'] for r in robots):
                self.send_zero(ident)
                continue

            # Predict the closest distance between moving people (PersonRules).
            if self.rules.future_blocked(w, person, (vx, vy), peers):
                self._person_block(ident, w, now, person, peers)
                self.send_zero(ident)
                continue
            self.rules.mark(ident, w, now, person, peers, False)
            cmd=Twist();cmd.linear.x=vx;cmd.linear.y=vy
            self.cmd_pubs[ident].publish(cmd)
            w.published_v=(vx,vy)
            w.last_position=person

    def _person_block(self,ident,w,now,person,peers):
        event=self.rules.mark(ident,w,now,person,peers,True)
        if event=='grant':
            self.record('deadlock_pass_granted',id=ident,x=round(person[0],3),y=round(person[1],3))
        elif event=='retreat':
            self.record('deadlock_retreat',id=ident,x=round(person[0],3),y=round(person[1],3))

    def publish_status(self,error=None):
        people=[]
        for i,w in sorted(self.people.items()):
            people.append({'id':i,'name':w.name,'position':self.fresh_pose(w.name),
                'paused':w.paused,'phase':w.phase,'action':w.current_action,
                'velocity_cmd':w.published_v,'target_end':w.target_end,'goal':w.end if w.target_end else w.start,'person_hold':w.person_hold,'proximity_hold':w.proximity_hold,'yield_selected':w.yield_selected,
                'person_pass':w.person_pass,'deadlock_retreats':w.person_retreats})
        payload={'seed':self.seed,'world':self.world,'paused':self.main_paused,
                 'robot_pose_valid':all(self.fresh_pose(r) is not None for r in self.robot_names),
                 'configured_lanes':len(self.cfg['lanes']),'deadlock_resolution':self.rules.enabled,'people':people,'error':self.last_error if error is None else error,
                 'gazebo_pose_stream_alive':self.pose_stream.alive()}
        output=String();output.data=json.dumps(payload,ensure_ascii=False)
        self.status_pub.publish(output)

    def cleanup(self):
        self.stop_all()
        # Publish repeated zeros before removing models; no robot command is sent.
        for _ in range(4):
            self.stop_all();time.sleep(0.1)
        for ident in list(self.people):
            try:
                self.delete_person(ident)
            except Exception as exc:
                self.get_logger().error(f'Cleanup failed: person {ident}: {exc}. Run recover after relaunch.')
        self.record('shutdown')
        self.pose_stream.close()
        self.log_file.close()


def main():
    # Prevent two managers controlling /model/test_person_XX/cmd_vel simultaneously.
    domain=os.environ.get('ROS_DOMAIN_ID','0')
    lock_path=f'/tmp/serving_robot_person_simulator_{domain}.lock'
    with open(lock_path,'w') as lock:
        try:
            fcntl.flock(lock, fcntl.LOCK_EX|fcntl.LOCK_NB)
        except BlockingIOError:
            raise SystemExit('Person simulator already running in this ROS domain')
        rclpy.init()
        node=None
        try:
            node=Manager()
            while rclpy.ok() and not node.shutdown_requested:
                rclpy.spin_once(node,timeout_sec=0.2)
        except KeyboardInterrupt:
            pass
        finally:
            if node is not None:
                node.cleanup()
                node.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()

if __name__=='__main__':
    main()
