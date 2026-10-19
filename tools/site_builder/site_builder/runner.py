"""Runs a tour step by step through a Driver and records every step.

The runner has no ROS in it. A Driver does the real work:

  start(node)               the robot is to stand at the home node; True = ready
  recover(node, reason)     put the robot on `node` (simulation: teleport, real robot: ask a person
                            and wait); True = it is there, False = stop the whole test
  run_step(step, timeout_s) drive one corridor, watch it, cancel at the first problem, and return
                            an Observation dict (see verdict.py)

After a step that did not pass the robot is not where the tour expects it, so the next step first
calls recover(). A failed corridor therefore never stops the test: it is marked and the tour goes on.
"""
import time

from .verdict import judge


class Driver:
    def start(self, node):
        raise NotImplementedError

    def recover(self, node, reason):
        raise NotImplementedError

    def run_step(self, step, timeout_s):
        raise NotImplementedError


class TourRunner:
    def __init__(self, tour, report, driver, par, only=None, max_consecutive_failures=4,
                 timeout_factor=3.0, timeout_extra_s=10.0, time_scale=1.0, log=print):
        if not time_scale > 0:
            raise ValueError('time_scale must be > 0')
        self.tour, self.report, self.driver, self.par = tour, report, driver, par
        self.only = None if not only else set(only)
        self.max_fail = max_consecutive_failures
        self.tf, self.te, self.log = timeout_factor, timeout_extra_s, log
        self.scale = float(time_scale)

    def _wanted(self, step):
        return self.only is None or step['edge'] in self.only or str(step['i']) in self.only

    def run(self):
        steps = self.tour['steps']
        ran = skipped = 0
        aborted = None
        pos = self.report.last_ok_position()
        if not self.report.data['steps'] and self.only is None:
            if not self.driver.start(self.tour['start']):
                return {'ran': 0, 'skipped': 0, 'aborted': 'the robot is not ready at the home node'}
            pos = self.tour['start']
        fails = 0
        try:
            for s in steps:
                if self.report.done(s['i']):
                    rec = self.report.get(s['i'])
                    pos = rec['to'] if rec['verdict'] == 'PASS' else None
                    skipped += 1
                    continue
                if not self._wanted(s):
                    skipped += 1
                    continue
                recovered = None
                if pos != s['from']:
                    recovered = 'robot expected at %s, was %s' % (s['from'], pos or 'unknown/failed step')
                    self.log('step %d: putting the robot on %s (%s)' % (s['i'], s['from'], recovered))
                    if not self.driver.recover(s['from'], recovered):
                        aborted = 'stopped while putting the robot on node %s' % s['from']
                        break
                    pos = s['from']
                expected = s['expected_s'] * self.scale      # this machine's pace (simulation may run slow)
                timeout = expected * self.tf + self.te
                self.log('step %d/%d: %s  %s -> %s  (%.1f m, expect %.0f s)'
                         % (s['i'], len(steps), s['edge'], s['from'], s['to'], s['length_m'], expected))
                t0 = time.monotonic()
                obs = dict(self.driver.run_step(s, timeout) or {})
                obs.setdefault('duration_s', round(time.monotonic() - t0, 1))
                obs['expected_s'] = round(expected, 1)
                obs['time_scale'] = self.scale
                obs['timeout_s'] = timeout
                verdict, flags, reasons = judge(obs, self.par)
                rec = {'i': s['i'], 'edge': s['edge'], 'from': s['from'], 'to': s['to'],
                       'probe': s.get('probe'), 'verdict': verdict, 'flags': flags, 'reasons': reasons,
                       'recovered_before': recovered, 'obs': obs,
                       'finished': time.strftime('%Y-%m-%d %H:%M:%S')}
                self.report.add(rec)
                ran += 1
                self.log('   -> %s%s' % (verdict, (' ' + '+'.join(flags)) if flags else ''))
                pos = s['to'] if verdict == 'PASS' else None
                fails = 0 if verdict == 'PASS' else fails + 1
                if fails >= self.max_fail:
                    aborted = '%d steps failed in a row: stopping (look at the robot first)' % fails
                    break
        except KeyboardInterrupt:
            aborted = 'interrupted by the user (everything done so far is saved)'
        return {'ran': ran, 'skipped': skipped, 'aborted': aborted}
