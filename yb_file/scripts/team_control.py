"""One owner process, one simulator, scoped shutdown; Linux/WSL entry point."""
import argparse,fcntl,json,os,signal,subprocess,sys,time
from pathlib import Path

root=Path(__file__).resolve().parents[1]
local=root/'_local_only';local.mkdir(exist_ok=True)
ap=argparse.ArgumentParser();ap.add_argument('action',choices=['start','stop']);ap.add_argument('--mode',choices=['operation','mapping'],default='operation');ap.add_argument('--headless',action='store_true');ap.add_argument('--robot',choices=['robot_01','robot_02']);args=ap.parse_args()
pidfile=local/'runtime.json'
if args.action=='stop':
    if not pidfile.exists():print('No managed session');sys.exit(0)
    entry=json.loads(pidfile.read_text());pid=int(entry['pid'])
    cmd=Path(f'/proc/{pid}/cmdline')
    if not cmd.exists():pidfile.unlink();sys.exit(0)
    if b'team_control.py' not in cmd.read_bytes():raise RuntimeError('PID reused; refusing to signal unrelated process')
    os.kill(pid,signal.SIGINT)
    deadline=time.monotonic()+25
    while pidfile.exists() and time.monotonic()<deadline:time.sleep(.2)
    if pidfile.exists():raise SystemExit('Managed shutdown did not finish; see runtime log')
    sys.exit(0)
lock=open(local/'runtime.lock','w')
try:fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
except BlockingIOError:raise SystemExit('TEAM session already running')
env=os.environ.copy()
for key in ('OMP_NUM_THREADS','OPENBLAS_NUM_THREADS','MKL_NUM_THREADS','NUMEXPR_NUM_THREADS'):env[key]='1'
env.setdefault('ROS_DOMAIN_ID','101');env.setdefault('GZ_PARTITION','team_integration_'+env['ROS_DOMAIN_ID'])
# Detect actual Gazebo command lines in this partition, including Ruby wrappers.
for proc in Path('/proc').iterdir():
    if not proc.name.isdigit():continue
    try:
        command=(proc/'cmdline').read_bytes().replace(b'\x00',b' ').decode(errors='replace')
        penv=(proc/'environ').read_bytes().split(b'\x00')
        if ('gz sim ' in command or 'gz-sim-server' in command) and ('GZ_PARTITION='+env['GZ_PARTITION']).encode() in penv:
            raise SystemExit('Gazebo already exists in requested partition: '+proc.name)
    except (FileNotFoundError,PermissionError,ProcessLookupError):pass
from configure_runtime import configure
config=configure(root)
if args.mode=='operation':
    import yaml
    c=yaml.safe_load(config.read_text())
    for path in (c['map_yaml'],c['perception']['weights']):
        if not Path(path).is_file() or Path(path).stat().st_size==0:raise SystemExit('Missing runtime asset: '+path)
command='source /opt/ros/jazzy/setup.bash && source '+str(root/'ros_ws/install/setup.bash')+' && exec ros2 launch cafe_autonomy team.launch.py config:='+str(config)+' mode:='+args.mode+' gui:='+('false' if args.headless else 'true')+(' robot:='+args.robot if args.robot else '')
stamp=time.strftime('%Y%m%d_%H%M%S');logpath=local/(args.mode+'_'+stamp+'.log')
with open(logpath,'wb') as log:
    child=subprocess.Popen(['bash','-c',command],env=env,stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
    pidfile.write_text(json.dumps({'pid':os.getpid(),'launch_pid':child.pid,'log':str(logpath),'mode':args.mode}))
    print('TEAM '+args.mode+'; log: '+str(logpath),flush=True)
    def stop(*_):
        signal.signal(signal.SIGINT,signal.SIG_IGN);signal.signal(signal.SIGTERM,signal.SIG_IGN)
        try:os.killpg(child.pid,signal.SIGINT)
        except ProcessLookupError:pass
    signal.signal(signal.SIGINT,stop);signal.signal(signal.SIGTERM,stop)
    try:code=child.wait()
    finally:
        stop()
        deadline=time.monotonic()+10
        while time.monotonic()<deadline:
            try:os.killpg(child.pid,0)
            except ProcessLookupError:break
            time.sleep(.2)
        try:os.killpg(child.pid,signal.SIGTERM)
        except ProcessLookupError:pass
        # Gazebo may fork out of the ros2 launch process group on WSL.
        # Match both this exact checkout and the owned partition before signaling.
        owned=[]
        for proc in Path('/proc').iterdir():
            if not proc.name.isdigit():continue
            try:
                cmd=(proc/'cmdline').read_bytes().replace(b'\x00',b' ')
                penv=(proc/'environ').read_bytes().split(b'\x00')
                if b'gz sim ' in cmd and str(root/'ros_ws/install/serving_robot_gazebo').encode() in cmd and ('GZ_PARTITION='+env['GZ_PARTITION']).encode() in penv:
                    owned.append(int(proc.name));os.kill(int(proc.name),signal.SIGTERM)
            except (FileNotFoundError,PermissionError,ProcessLookupError):pass
        deadline=time.monotonic()+5
        while owned and time.monotonic()<deadline:
            owned=[pid for pid in owned if Path(f'/proc/{pid}/cmdline').exists() and Path(f'/proc/{pid}/cmdline').read_bytes()]
            time.sleep(.2)
        for pid in owned:
            try:os.kill(pid,signal.SIGKILL)
            except ProcessLookupError:pass
        pidfile.unlink(missing_ok=True)
sys.exit(code)
