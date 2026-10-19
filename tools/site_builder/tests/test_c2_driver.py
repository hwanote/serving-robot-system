"""C-2: BusDriver + StepMonitor against a fake bus that behaves like the route executor."""
import os, sys, json, tempfile, unittest
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from _fixtures import ensure_demo_fixtures   # noqa: E402
ensure_demo_fixtures()
import yaml
from site_builder import monitor, probe, tour as tourmod
from site_builder.drive_ros import BusDriver
from site_builder.params import load_params
from site_builder.report import Report
from site_builder.runner import TourRunner

GEN = os.path.join(os.path.dirname(__file__), '..', 'out', 'drive_demo')


class FakeBus:
    """scenario: ok | block | hardstop | collision | reject | hang | noloc | resetfail"""
    def __init__(self, scenario='ok', pose_of=None):
        self.t = 0.0; self.sink = None; self.scenario = scenario
        self.sent = []; self.placed = []; self.ev = []; self.latch = False
        self.cur = None; self.listen = 1; self.cleared = 0; self.xy = (0.0, 0.0)
        self.step_len = 2.0
        self.where = {}; self.moves = True
    def now(self): return self.t
    def listeners(self): return self.listen
    def pose(self): return (self.xy[0], self.xy[1], 0.0)
    def at(self, dt, f): self.ev.append((self.t + dt, f))
    def spin(self, to):
        self.t += to
        due = [e for e in self.ev if e[0] <= self.t]
        self.ev = [e for e in self.ev if e[0] > self.t]
        for _, f in sorted(due, key=lambda e: e[0]): f()
    def clear_collision(self):
        self.cleared += 1
        self.at(0.3, lambda: self._latch(False)); return True
    def _latch(self, v):
        self.latch = v; self.sink.latched(v, self.t)
    def place(self, pose):
        self.placed.append(pose)
        if self.moves and pose in self.where: self.xy = self.where[pose]
        self.at(0.5, lambda: self.sink.status({'state': 'IDLE', 'localization': None if self.scenario == 'noloc' else 'OK'}, self.t)); return True
    def publish_cmd(self, d):
        self.sent.append(d); c = d['cmd']
        if c == 'reset':
            if self.scenario == 'resetfail':
                self.at(0.2, lambda: self.sink.alarm({'code': 'BAD_COMMAND'}, self.t)); return
            self.at(0.2, lambda: self.sink.alarm({'code': 'ROBOT_RESET'}, self.t))
            self.at(0.4, lambda: self.sink.status({'state': 'IDLE', 'localization': None if self.scenario == 'noloc' else 'OK'}, self.t))
        elif c == 'go':
            mid = d['id']; self.cur = mid; S = self.scenario
            st = lambda **k: self.sink.status(dict({'mission_id': mid, 'state': 'RUNNING'}, **k), self.t)
            self.at(0.1, st)
            if S == 'ok':
                self.at(8, lambda: (self.sink.status({'mission_id': mid, 'state': 'IDLE', 'result': 'SUCCEEDED'}, self.t)))
            elif S == 'person':
                self.at(0.5, lambda: self.sink.person({'state': 'SLOW'}, self.t))
                self.at(3.5, lambda: self.sink.person({'state': 'STOP'}, self.t))
                self.at(5.5, lambda: self.sink.person({'state': 'NORMAL'}, self.t))
                self.at(8, lambda: self.sink.status({'mission_id': mid, 'state': 'IDLE', 'result': 'SUCCEEDED'}, self.t))
            elif S == 'block':
                self.at(1, lambda: st(stopped_s=3.5)); self.at(2, lambda: self.sink.alarm({'code': 'BLOCKED'}, self.t))
            elif S == 'hardstop':
                for k in range(0, 30): self.at(1 + k * 0.1, lambda: self.sink.hardstop(True, self.t))
            elif S == 'collision':
                self.at(1, lambda: self._latch(True))
            elif S == 'reject':
                self.at(0.2, lambda: self.sink.alarm({'code': 'BAD_COMMAND', 'rejected_id': mid}, self.t))
        elif c == 'cancel':
            if d.get('id') and d.get('id') == self.cur:
                mid = self.cur
                self.at(0.5, lambda: self.sink.status({'mission_id': mid, 'state': 'IDLE', 'result': 'CANCELED'}, self.t))


def setup(td):
    wp = yaml.safe_load(open(os.path.join(GEN, 'waypoints.yaml')))
    tour = json.load(open(os.path.join(GEN, 'tour.json')))
    return wp, tour


class T(unittest.TestCase):
    def mk(self, scenario, mode='sim', ask=None):
        self.td = tempfile.mkdtemp()
        wp, tour = setup(self.td)
        bus = FakeBus(scenario)
        for pn, pv in wp['poses'].items():
            nd = wp['nodes'][pv['node']]; bus.where[pn] = (nd['x'], nd['y'])
        par = load_params()
        drv = BusDriver(bus, tour, wp, par, mode=mode, ask=ask or (lambda q: 'y'), log=lambda *a: None)
        return bus, drv, tour, par

    def test_sector_mins(self):
        import math
        r = [5.0] * 360
        r[0] = 1.0; r[90] = 0.7; r[270] = 0.9; r[180] = 0.1
        f, l, rt = monitor.sector_mins(r, -math.pi, 2 * math.pi / 360, 0.05, 10)
        self.assertEqual((f, l, rt), (5.0 if False else min(r[150:211]), 0.7 if False else min(r[240:301]), min(r[60:121])))

    def test_ros_names_exist_in_interfaces_yaml(self):
        """RosBus cannot run here, but every name it asks for must exist in the real interfaces.yaml."""
        import re
        src = open(os.path.join(os.path.dirname(__file__), '..', 'site_builder', 'drive_ros.py')).read()
        raw = yaml.safe_load(open(os.path.join(os.path.dirname(__file__), '..', '..', '..', 'config', 'common', 'interfaces.yaml')))
        used = re.findall(r"n\.(topic|service|nav2)\('([a-z_]+)'\)", src)
        self.assertGreaterEqual(len(used), 7)
        sect = {'topic': 'topics', 'service': 'services', 'nav2': 'nav2'}
        for kind, key in used:
            self.assertIn(key, raw[sect[kind]], '%s %s' % (kind, key))

    def test_ok_step(self):
        bus, drv, tour, par = self.mk('ok')
        self.assertTrue(drv.start(tour['start']))
        self.assertEqual(bus.placed, ['standby_01'])
        o = drv.run_step(tour['steps'][0], 40)
        self.assertEqual(o['result'], 'SUCCEEDED'); self.assertFalse(o['timed_out'])
        self.assertEqual(bus.sent[-1]['cmd'], 'go'); self.assertEqual(bus.sent[-1]['names'], ['probe_01'])

    def test_block_cancels(self):
        bus, drv, tour, par = self.mk('block'); drv.start(tour['start'])
        o = drv.run_step(tour['steps'][0], 40)
        self.assertIn('BLOCKED', o['alarms']); self.assertTrue(any(s['cmd'] == 'cancel' and s.get('id') for s in bus.sent))
        self.assertLess(o['duration_s'], 8)

    def test_hardstop_cancels_after_1s(self):
        bus, drv, tour, par = self.mk('hardstop'); drv.start(tour['start'])
        o = drv.run_step(tour['steps'][0], 40)
        self.assertGreaterEqual(o['hardstop_s'], 1.0); self.assertLess(o['hardstop_s'], 2.0)

    def test_collision_cancels_and_recover_clears(self):
        bus, drv, tour, par = self.mk('collision'); drv.start(tour['start'])
        o = drv.run_step(tour['steps'][0], 40)
        self.assertTrue(o['latched']); self.assertFalse(o['timed_out']); self.assertLess(o['duration_s'], 10)
        self.assertTrue(drv.recover('n02', 'x')); self.assertGreaterEqual(bus.cleared, 1)

    def test_reject(self):
        bus, drv, tour, par = self.mk('reject'); drv.start(tour['start'])
        o = drv.run_step(tour['steps'][0], 40)
        self.assertEqual(o['result'], 'FAILED'); self.assertEqual(o['code'], 'BAD_COMMAND')

    def test_timeout(self):
        bus, drv, tour, par = self.mk('hang'); drv.start(tour['start'])
        o = drv.run_step(tour['steps'][0], 12)
        self.assertTrue(o['timed_out']); self.assertTrue(any(s['cmd'] == 'cancel' for s in bus.sent))

    def test_listener_discovered_late(self):
        bus, drv, tour, par = self.mk('ok'); bus.listen = 0
        bus.at(3.0, lambda: setattr(bus, 'listen', 1))          # discovery takes 3 s
        self.assertTrue(drv.start(tour['start']))
        self.assertGreaterEqual(bus.t, 3.0)

    def test_no_listener(self):
        bus, drv, tour, par = self.mk('ok'); bus.listen = 0
        self.assertFalse(drv.start(tour['start']))
        self.assertGreaterEqual(bus.t, 15.0)                    # it really waited before giving up

    def test_reset_refused_and_no_localization(self):
        bus, drv, tour, par = self.mk('resetfail'); self.assertFalse(drv.start(tour['start']))
        bus, drv, tour, par = self.mk('noloc'); self.assertFalse(drv.start(tour['start']))

    def test_manual_mode(self):
        qs = []
        def ask(q):
            qs.append(q)
            if len(qs) == 2: bus.xy = bus.where['standby_01']       # the person puts the robot on the node
            return 'y' if len(qs) == 1 else ''
        bus, drv, tour, par = self.mk('ok', 'manual', ask=ask)
        bus.at(0.0, lambda: None)
        # manual: no teleport, the person is asked twice (safety, placement); localization comes from the status
        bus.scenario = 'ok'
        orig = bus.publish_cmd
        def pub(d):
            orig(d)
            if d['cmd'] == 'reset':
                bus.at(0.6, lambda: bus.sink.status({'state': 'IDLE', 'localization': 'OK'}, bus.t))
        bus.publish_cmd = pub
        self.assertTrue(drv.start(tour['start']))
        self.assertEqual(len(qs), 2); self.assertEqual(bus.placed, [])
        self.assertIn('emergency', qs[0])
        bus2, drv2, tour2, _ = self.mk('ok', 'manual', ask=lambda q: 'n')
        self.assertFalse(drv2.start(tour2['start']))
        bus3, drv3, tour3, _ = self.mk('ok', 'manual', ask=lambda q: 'y' if 'emergency' in q else 'q')
        self.assertFalse(drv3.start(tour3['start']))

    def test_place_that_did_not_move_the_robot_is_refused(self):
        """Gazebo says true even when the model was not moved; the localization check must catch it."""
        bus, drv, tour, par = self.mk('ok'); bus.moves = False; bus.xy = (3.0, 2.0)
        logs = []; drv.log = logs.append
        self.assertFalse(drv.start(tour['start']))
        self.assertTrue(any('not on n01' in l for l in logs), logs)
        bus2, drv2, tour2, _ = self.mk('ok')
        self.assertTrue(drv2.start(tour2['start']))
        bus2.xy = (bus2.xy[0] + 0.2, bus2.xy[1])                 # 20 cm off: inside the tolerance
        self.assertTrue(drv2.recover('n02', 'x'))

    def test_pose_that_appears_late_is_waited_for(self):
        bus, drv, tour, par = self.mk('ok')
        real = bus.pose
        bus.pose = lambda: None if bus.t < 9.0 else real()      # the TF buffer fills slowly
        self.assertTrue(drv.start(tour['start']))
        self.assertGreaterEqual(bus.t, 9.0)

    def test_pose_that_never_appears_is_refused_with_the_reason(self):
        bus, drv, tour, par = self.mk('ok'); bus.pose = lambda: None; bus.last_error = 'LookupException: no frame'
        logs = []; drv.log = logs.append
        t0 = bus.t
        self.assertFalse(drv.start(tour['start']))
        self.assertTrue(any('LookupException' in l for l in logs), logs)
        self.assertGreaterEqual(bus.t - t0, 12.0)               # it really waited (3 s settle + 10 s)

    def test_no_pose_after_placing_is_refused(self):
        bus, drv, tour, par = self.mk('ok'); bus.pose = lambda: None
        logs = []; drv.log = logs.append
        self.assertFalse(drv.start(tour['start']))
        self.assertTrue(any('no robot pose' in l for l in logs), logs)

    def test_person_limit_time_is_recorded_and_flagged(self):
        from site_builder.verdict import judge
        bus, drv, tour, par = self.mk('person'); drv.start(tour['start'])
        o = drv.run_step(tour['steps'][0], 60)
        self.assertAlmostEqual(o['person_slow_s'], 3.0, delta=0.4)
        self.assertAlmostEqual(o['person_stop_s'], 2.0, delta=0.4)
        o['expected_s'], o['timeout_s'] = 10.0, 60
        v, flags, why = judge(o, par)
        self.assertEqual(v, 'PASS'); self.assertIn('PERSON_LIMIT', flags)
        bus2, drv2, tour2, _ = self.mk('ok'); drv2.start(tour2['start'])
        o2 = drv2.run_step(tour2['steps'][0], 60)
        self.assertEqual((o2['person_slow_s'], o2['person_stop_s']), (0.0, 0.0))

    def test_rosbus_uses_the_default_robot_name_and_listens_to_person_limit(self):
        src = open(os.path.join(os.path.dirname(__file__), '..', 'site_builder', 'drive_ros.py')).read()
        self.assertIn("self.robot = n.robot", src)       # '' must never reach Gazebo as the model name
        self.assertIn("n.topic('person_limit')", src)

    def test_full_tour_with_failures(self):
        bus, drv, tour, par = self.mk('ok')
        rep = Report(os.path.join(self.td, 'r.json'), tour)
        res = TourRunner(tour, rep, drv, par, only=['e01'], log=lambda *a: None).run()
        self.assertIsNone(res['aborted']); self.assertGreaterEqual(res['ran'], 1)
        self.assertEqual(rep.get(1)['verdict'], 'PASS')


if __name__ == '__main__':
    unittest.main()
