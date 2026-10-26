"""Drive-test commands that need no ROS.

    python -m site_builder.drive plan --candidate config/generated --out-dir out/drive_demo
    python -m site_builder.drive summary --tour out/drive_demo/tour.json --report out/drive_demo/test_report.json

`plan` writes the drive-test folder (config/common + candidate + probe poses) and tour.json.
The robot side is started on it with  SERVING_ROBOT_CONFIG_DIR=<out-dir>.
`run` drives the tour (ROS 2 needed; logic in drive_ros.py, tested with a fake bus).
"""
import argparse
import json
import os
import sys

from . import probe
from .params import REPO, load_params
from .report import Report


def cmd_plan(a):
    par = load_params()
    tour = probe.build(a.candidate, a.common, a.out_dir, par.speed_mps, par.omega_rps)
    total = sum(s['expected_s'] for s in tour['steps'])
    print('tour %s: %d steps over %d corridors, home %s, about %.1f min at %.2f m/s'
          % (tour['hash'], len(tour['steps']), len(tour['steps']) // 2, tour['start'], total / 60, par.speed_mps))
    for s in tour['steps']:
        print('  %2d  %-4s %s -> %s  %5.2f m  heading %6.1f  turn %5.1f  %s'
              % (s['i'], s['edge'], s['from'], s['to'], s['length_m'], s['heading_deg'], s['turn_deg'], s['probe']))
    print('drive-test folder:', os.path.abspath(a.out_dir))
    print('start the robot side with:  export SERVING_ROBOT_CONFIG_DIR=%s' % os.path.abspath(a.out_dir))
    return 0


def cmd_summary(a):
    with open(a.tour, 'r', encoding='utf-8') as f:
        tour = json.load(f)
    rep = Report(a.report, tour)
    print(rep.summary())
    return 0


def make_proposals(tour_path, report_path, out_path, candidate=None):
    """tour.json + test_report.json -> adjustments.json (proposals only). Decisions already made
    on an identical proposal (same kind, nodes/edge and shift) are carried over."""
    import yaml
    from . import adjust
    with open(tour_path, 'r', encoding='utf-8') as f:
        tour = json.load(f)
    with open(report_path, 'r', encoding='utf-8') as f:
        rep = json.load(f)
    if rep.get('tour_hash') != tour['hash']:
        raise ValueError('the report belongs to another tour')
    cand = candidate or tour['candidate_dir']
    with open(os.path.join(cand, 'waypoints.yaml'), 'r', encoding='utf-8') as f:
        wp = yaml.safe_load(f)
    ap = os.path.join(cand, 'adjustments_applied.json')
    info = None
    if os.path.isfile(ap):
        with open(ap, 'r', encoding='utf-8') as f:
            info = json.load(f)
    out = adjust.propose(wp, rep, tour, info)
    sig = lambda p: json.dumps([p['kind'], p.get('edge'), p.get('nodes'), p.get('d')], sort_keys=True)
    if os.path.isfile(out_path):
        with open(out_path, 'r', encoding='utf-8') as f:
            old = json.load(f)
        if old.get('base_hash') == out['base_hash']:
            prev = {sig(p): p['status'] for p in old.get('proposals', [])}
            for p in out['proposals']:
                if p['kind'] != 'review_edge' and sig(p) in prev:
                    p['status'] = prev[sig(p)]
    tmp = out_path + '.tmp'
    with open(tmp, 'w', encoding='utf-8') as f:
        json.dump(out, f, indent=1, ensure_ascii=False)
    os.replace(tmp, out_path)
    return out


def cmd_adjust(a):
    try:
        out = make_proposals(a.tour, a.report, a.out, a.candidate)
    except ValueError as e:
        print(e)
        return 1
    for p in out['proposals']:
        print('%s  %-11s %s  %s' % (p['id'], p['kind'], p.get('edge') or ','.join(p.get('edges', [])), p['reason']))
    for n in out['notes']:
        print('note:', n)
    print('%d proposals written to %s (status proposed; nothing is applied until approved)' % (len(out['proposals']), a.out))
    return 0


def cmd_run(a):
    import yaml
    from .drive_ros import BusDriver, RosBus
    from .runner import TourRunner
    with open(a.tour, 'r', encoding='utf-8') as f:
        tour = json.load(f)
    cfg = tour['test_config_dir']
    with open(os.path.join(cfg, 'waypoints.yaml'), 'r', encoding='utf-8') as f:
        wp = yaml.safe_load(f)
    par = load_params()
    rep = Report(a.report or os.path.join(os.path.dirname(os.path.abspath(a.tour)), 'test_report.json'), tour)
    bus = RosBus(a.robot, cfg, a.world)
    try:
        drv = BusDriver(bus, tour, wp, par, mode=a.mode)
        res = TourRunner(tour, rep, drv, par, only=a.only, max_consecutive_failures=a.max_fail, time_scale=a.time_scale).run()
    finally:
        bus.close()
    print(rep.summary())
    print('ran %(ran)d, skipped %(skipped)d' % res)
    if res['aborted']:
        print('STOPPED: %s' % res['aborted'])
        return 2
    return 0


def main(argv=None):
    ap = argparse.ArgumentParser(prog='site_builder.drive')
    sub = ap.add_subparsers(dest='cmd', required=True)
    p = sub.add_parser('plan', help='write the drive-test folder and tour.json')
    p.add_argument('--candidate', required=True, help='folder with the generated waypoints/entities/zones')
    p.add_argument('--common', default=os.path.join(REPO, 'config', 'common'))
    p.add_argument('--out-dir', required=True)
    p.set_defaults(func=cmd_plan)
    s = sub.add_parser('summary', help='print the result table of a test_report.json')
    s.add_argument('--tour', required=True)
    s.add_argument('--report', required=True)
    s.set_defaults(func=cmd_summary)
    j = sub.add_parser('adjust', help='measured results -> adjustments.json (proposals only)')
    j.add_argument('--tour', required=True)
    j.add_argument('--report', required=True)
    j.add_argument('--candidate', default=None, help='default: the candidate folder named in tour.json')
    j.add_argument('--out', required=True)
    j.set_defaults(func=cmd_adjust)
    r = sub.add_parser('run', help='drive the tour through the route executor (needs ROS 2 and the robot side running)')
    r.add_argument('--tour', required=True)
    r.add_argument('--report', default=None, help='default: test_report.json next to the tour')
    r.add_argument('--mode', choices=('sim', 'manual'), default='sim',
                   help='sim: teleport in Gazebo; manual: a person places the robot (real robot)')
    r.add_argument('--only', nargs='*', help='edge ids (e01) or step numbers; e.g. --only e01 for a first try')
    r.add_argument('--world', default='cafe')
    r.add_argument('--robot', default='')
    r.add_argument('--max-fail', type=int, default=4)
    r.add_argument('--time-scale', type=float, default=1.0,
                   help='multiplies the expected time of every step (time limit and SLOW check); '
                        'use e.g. 3.5 when the simulation runs slower than real time')
    r.set_defaults(func=cmd_run)
    a = ap.parse_args(argv)
    return a.func(a)


if __name__ == '__main__':
    sys.exit(main())
