"""Settings of one backend run (command line -> Settings)."""
import os
from dataclasses import dataclass, field


def parse_zones(text):
    """'robot_01:table_01,table_03;robot_02:table_02' -> {table: robot}"""
    out = {}
    for part in filter(None, (p.strip() for p in text.split(';'))):
        robot, _, tables = part.partition(':')
        for tb in filter(None, (x.strip() for x in tables.split(','))):
            out[tb] = robot.strip()
    return out


def parse_robots(text):
    """'robot_01,robot_02:2' -> [('robot_01', None), ('robot_02', 2)]"""
    out = []
    for part in text.split(','):
        part = part.strip()
        if not part:
            continue
        name, _, dom = part.partition(':')
        out.append((name, int(dom) if dom else None))
    if not out:
        raise ValueError('robots is empty')
    return out


def default_repo_root():
    here = os.path.dirname(os.path.abspath(__file__))      # web/backend/app
    return os.path.dirname(os.path.dirname(os.path.dirname(here)))


@dataclass
class Settings:
    adapter: str = 'fake'                       # 'fake' | 'ros'
    robots: list = field(default_factory=lambda: [('robot_01', None)])
    repo: str = field(default_factory=default_repo_root)
    store_id: str = 'demo_store'
    db_url: str = 'memory'                      # 'memory' or a SQLAlchemy URL
    host: str = '127.0.0.1'
    port: int = 8000
    kitchen: str = 'kitchen'
    mode: str = 'C'                             # A single robot | B simple dispatch | C cooperative
    priorities: dict = field(default_factory=dict)   # table -> int, bigger is served first
    # dispatcher
    stagger_s: float = 10.0
    kitchen_lock: bool = True
    route_check: bool = True
    zones: dict = field(default_factory=dict)   # table -> owner robot
    help_after_s: float = 15.0
    idle_home_s: float = 10.0
    serve_home: bool = False
    # operations
    offline_after_s: float = 5.0       # no message this long -> robot OFFLINE
    offline_fail_s: float = 30.0       # offline this long with an order -> order FAILED
    ack_timeout_s: float = 10.0        # order sent but the robot never reports it -> FAILED
    # map setup (saved Semantic Map, automatic nodes)
    data_dir: str = ''                 # '' = web/backend/data
    require_drive_test: bool = False   # apply needs a passed drive test (the tool is not wired yet)
    # automatic mapping (the ROS mapping stack is started by a person; the backend only runs the session script)
    mapping_enabled: bool = False      # off: Start Mapping answers 503 MAPPING_DISABLED
    mapping_work_dir: str = ''         # '' = <data_dir>/mapping (sessions, drafts, outputs, state.json)
    mapping_runner: str = ''           # '' = ros_ws/src/serving_robot_mapping/tools/run_lidar_mapping_session.py
    mapping_ros_python: str = '/usr/bin/python3'
    mapping_require_ros_env: bool = True   # ROS_DISTRO must be jazzy (a source'd terminal)
    # SLAM map frame starts at the robot's start pose. Simulation: the Gazebo spawn pose
    # (-3.2, -2.4) so the map lines up with the world; a real store: 0, 0 (M-19)
    mapping_shift_origin: list = field(default_factory=lambda: [0.0, 0.0])
    # fake adapter
    fake_speed: float = 1.0
    fake_wait: float = 2.0
    fake_fail: str = ''
    # ros adapter
    use_sim_time: bool = True


def default_data_dir():
    here = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))      # web/backend
    return os.path.join(here, 'data')


def default_db_url():
    data = default_data_dir()
    os.makedirs(data, exist_ok=True)
    return 'sqlite:///' + os.path.join(data, 'backend.db').replace(os.sep, '/')


MODES = {
    'A': 'one robot (the first one), no route check',
    'B': 'all robots, nearest free robot takes the oldest order, kitchen lock only',
    'C': 'all robots, zones + route check + help (cooperative)',
}


def apply_mode(s):
    """Turn the mode name into the switches it stands for. Returns s."""
    s.mode = s.mode.upper()
    if s.mode not in MODES:
        raise ValueError('mode must be one of %s' % ', '.join(MODES))
    if s.mode == 'A':
        s.robots = s.robots[:1]
    if s.mode in ('A', 'B'):
        s.route_check, s.zones = False, {}
    return s


def load_fleet_file(path):
    """Read a fleet YAML file -> {Settings field: value}. See config/fleet.example.yaml."""
    import yaml
    with open(path, encoding='utf-8') as f:
        raw = yaml.safe_load(f) or {}
    known = set(Settings.__dataclass_fields__)
    bad = sorted(set(raw) - known)
    if bad:
        raise ValueError('unknown keys in %s: %s' % (path, ', '.join(bad)))
    out = {k: v for k, v in raw.items() if v is not None}      # an empty key means "default"
    if 'robots' in out:
        rb = out['robots']
        out['robots'] = parse_robots(rb) if isinstance(rb, str) else [
            (x, None) if isinstance(x, str) else (x['name'], x.get('domain')) for x in rb]
    if 'zones' in out and not isinstance(out['zones'], str):
        out['zones'] = {tb: robot for robot, tables in out['zones'].items() for tb in tables}
    elif 'zones' in out:
        out['zones'] = parse_zones(out['zones'])
    if 'priorities' in out:
        out['priorities'] = {k: int(v) for k, v in out['priorities'].items()}
    return out
