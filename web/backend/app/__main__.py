"""python -m app --adapter ros --robots robot_01:0,robot_02:2 --zones "..."   (run in web/backend)

Precedence: built-in defaults < --config file < command line.
"""
import argparse

from .config import (MODES, Settings, apply_mode, default_db_url, default_repo_root,
                     load_fleet_file, parse_robots, parse_zones)


def build_parser():
    ap = argparse.ArgumentParser(description='serving-robot backend')
    a = ap.add_argument
    a('--config', help='fleet YAML file (config/fleet.example.yaml)')
    a('--mode', choices=sorted(MODES), help='; '.join('%s = %s' % kv for kv in MODES.items())
      + ' (default C)')
    a('--adapter', choices=['fake', 'ros'], help='default fake')
    a('--robots', help='name[:ROS_DOMAIN_ID], comma separated, e.g. robot_01:0,robot_02:2')
    a('--repo', help='repository root (config/common and maps/stores are read from here)')
    a('--store-id', help='default demo_store')
    a('--db', help="SQLAlchemy URL, or 'memory'. default: sqlite file web/backend/data/backend.db")
    a('--host', help='default 127.0.0.1; keep it: this accepts commands that move robots')
    a('--port', type=int, help='default 8000')
    a('--stagger-s', type=float, help='min seconds between two orders sent (default 10)')
    a('--no-kitchen-lock', action='store_true',
      help='allow a second robot to head for the kitchen while another is there')
    a('--no-route-check', action='store_true',
      help='send orders without checking which route nodes other robots hold')
    a('--zones', help='table owners, e.g. "robot_01:table_01,table_03;robot_02:table_02,table_04"')
    a('--help-after-s', type=float, help='seconds an order waits for its owner (default 15)')
    a('--idle-home-s', type=float, help='idle seconds away from home before going home (10)')
    a('--serve-home', action='store_true', help='every order ends with the drive home')
    a('--offline-after-s', type=float, help='silence this long -> robot OFFLINE (5)')
    a('--offline-fail-s', type=float, help='offline this long with an order -> FAILED (30)')
    a('--ack-timeout-s', type=float, help='order never reported by the robot -> FAILED (10)')
    a('--fake-speed', type=float, help='fake adapter speed m/s (1)')
    a('--fake-wait', type=float, help='fake adapter load/handover wait s (2)')
    a('--fake-fail', help='fake adapter: this table fails with NO_ROUTE')
    a('--wall-clock', action='store_true', help='ros adapter: no use_sim_time')
    a('--data-dir', help='where saved Semantic Maps and generated nodes go (web/backend/data)')
    a('--require-drive-test', action='store_true',
      help='applying a map setup needs a passed drive test (not wired yet: apply is refused)')
    return ap


def parse_args(argv=None):
    return build_parser().parse_args(argv)


def settings_from_args(a):
    s = Settings(repo=default_repo_root(), db_url=default_db_url())
    if a.config:
        for k, v in load_fleet_file(a.config).items():
            setattr(s, k, v)
    plain = {'mode': a.mode, 'adapter': a.adapter, 'repo': a.repo, 'store_id': a.store_id,
             'db_url': a.db, 'host': a.host, 'port': a.port, 'stagger_s': a.stagger_s,
             'help_after_s': a.help_after_s, 'idle_home_s': a.idle_home_s,
             'offline_after_s': a.offline_after_s, 'offline_fail_s': a.offline_fail_s,
             'ack_timeout_s': a.ack_timeout_s, 'fake_speed': a.fake_speed,
             'fake_wait': a.fake_wait, 'fake_fail': a.fake_fail, 'data_dir': a.data_dir}
    for k, v in plain.items():
        if v is not None:
            setattr(s, k, v)
    if a.robots:
        s.robots = parse_robots(a.robots)
    if a.zones is not None:
        s.zones = parse_zones(a.zones)
    if a.no_kitchen_lock:
        s.kitchen_lock = False
    if a.no_route_check:
        s.route_check = False
    if a.serve_home:
        s.serve_home = True
    if a.wall_clock:
        s.use_sim_time = False
    if a.require_drive_test:
        s.require_drive_test = True
    return apply_mode(s)


def main():
    import uvicorn
    from .main import create_app
    settings = settings_from_args(parse_args())
    print('mode %s: %s; robots %s' % (settings.mode, MODES[settings.mode],
                                       ', '.join(n for n, _ in settings.robots)), flush=True)
    uvicorn.run(create_app(settings), host=settings.host, port=settings.port,
                log_level='info')


if __name__ == '__main__':
    main()
