import itertools
import random
import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
import offline_world as ow
from serving_robot_person_simulator.policy import draw_speed

SPEEDS = (0.08, 0.15, 0.22)   # existing policy values, unchanged


class OfflineDeadlock(unittest.TestCase):
    def test_old_rules_reproduce_6_8_deadlock(self):
        r = ow.run(ow.load_cfg(deadlock=False), (6, 8), {6: .15, 8: .15}, seconds=600)
        self.assertGreaterEqual(r['max_all_stopped_s'], 300, r)   # permanent mutual stop
        self.assertLessEqual(min(r['turns'].values()), 2, r)

    def test_new_rules_resolve_every_pair_and_speed_combo(self):
        cfg = ow.load_cfg()
        worst = 0.0
        for a, b in itertools.combinations(range(1, 9), 2):
            for sa, sb in itertools.product(SPEEDS, SPEEDS):
                r = ow.run(cfg, (a, b), {a: sa, b: sb}, seconds=900)
                ctx = (a, b, sa, sb, r)
                self.assertLessEqual(r['max_all_stopped_s'], 25.0, ctx)
                self.assertGreaterEqual(min(r['turns'].values()), 2, ctx)
                self.assertGreaterEqual(r['min_sep'], 0.50 - 0.03, ctx)  # never closer than hard min
                self.assertGreater(r['min_sep'], 2 * cfg['person_radius_m'], ctx)  # no body overlap
                worst = max(worst, r['max_all_stopped_s'])
        print(f'\n[pairs] 28 pairs x 9 speed combos OK; worst all-stopped = {worst:.1f}s')

    def test_new_rules_all_eight_together(self):
        cfg = ow.load_cfg()
        for seed in (1, 2, 3, 4, 5):
            rng = random.Random(seed)
            speeds = {i: draw_speed(rng, cfg)[1] for i in range(1, 9)}
            r = ow.run(cfg, tuple(range(1, 9)), speeds, seconds=1500)
            ctx = (seed, speeds, r)
            self.assertLessEqual(r['max_all_stopped_s'], 25.0, ctx)
            self.assertGreaterEqual(min(r['turns'].values()), 2, ctx)
            self.assertGreater(r['min_sep'], 2 * cfg['person_radius_m'], ctx)
            print(f'[eight seed={seed}] turns={r["turns"]} min_sep={r["min_sep"]:.3f} '
                  f'worst_stop={r["max_all_stopped_s"]:.1f}s passes={r["passes"]}')


if __name__ == '__main__':
    unittest.main()
