import sys
from pathlib import Path
import unittest
import random
import yaml

sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from serving_robot_person_simulator.policy import choose, draw_speed, decide_action, point_near_polyline, validate, next_time, validate_spawn_positions
from serving_robot_person_simulator.gazebo_api import make_person_sdf

CONFIG=Path(__file__).resolve().parents[1]/'config'/'policy.yaml'

class PolicyTests(unittest.TestCase):
    def setUp(self):
        self.cfg=validate(yaml.safe_load(CONFIG.read_text()))

    def test_probabilities(self):
        self.assertEqual(sum(x['percent'] for x in self.cfg['behavior']['speed'].values()),100)
        self.assertEqual(sum(self.cfg['behavior']['action_percent'].values()),100)
        self.assertEqual(self.cfg['behavior']['action_percent']['sudden_stop'],2)

    def test_range_1_to_8(self):
        self.assertEqual(self.cfg['people_max'],8)
        # Approved lanes may be added after installation; keep tests config-agnostic.
        self.assertLessEqual(len(self.cfg['lanes']),8)

    def test_invalid_weights(self):
        self.cfg['behavior']['action_percent']['sudden_stop']=10
        with self.assertRaises(ValueError): validate(self.cfg)

    def test_invalid_speed(self):
        self.cfg['behavior']['speed']['fast']['mps']=2.0
        with self.assertRaises(ValueError): validate(self.cfg)

    def test_deterministic(self):
        r1,r2=random.Random(20261014),random.Random(20261014)
        a=[draw_speed(r1,self.cfg) for _ in range(200)]
        b=[draw_speed(r2,self.cfg) for _ in range(200)]
        self.assertEqual(a,b)

    def test_percentage_rates(self):
        r=random.Random(31)
        outcomes=[choose(r,self.cfg['behavior']['action_percent']) for _ in range(20000)]
        count=sum(x=='sudden_stop' for x in outcomes)
        self.assertTrue(300<count<500, count)

    def test_block_requires_corridor(self):
        rng=random.Random(4)
        # No corridor: long_block can never be selected.
        self.assertFalse(any(decide_action(rng,self.cfg,0,0,0)[0]=='long_block' for _ in range(3000)))
        self.cfg['blockage_corridor']=[[0,0],[2,0]]
        self.assertTrue(point_near_polyline(1,0.4, self.cfg['blockage_corridor'],0.6))
        self.assertFalse(point_near_polyline(1,1, self.cfg['blockage_corridor'],0.6))
        self.assertTrue(any(decide_action(rng,self.cfg,0,1,0)[0]=='long_block' for _ in range(3000)))

    def test_decision_not_every_frame(self):
        t=next_time(random.Random(33),self.cfg,5)
        self.assertGreaterEqual(t,13)
        self.assertLessEqual(t,21)

    def test_batch_preflight_prevents_overlap(self):
        with self.assertRaises(ValueError):
            validate_spawn_positions([((2,2),(4,2)),((2.1,2),(4,3))],
                                     (0,0),[],1.05,0.75)
        validate_spawn_positions([((2,2),(4,2)),((2,3),(4,3))],
                                 (0,0),[],1.05,0.75)

    def test_person_is_physical_model(self):
        sdf=make_person_sdf('test_person_01',-2,1)
        self.assertIn('person_collision',sdf)
        self.assertIn('gz::sim::systems::VelocityControl',sdf)
        self.assertIn('/model/test_person_01/cmd_vel',sdf)

    def test_reject_unowned_name(self):
        with self.assertRaises(ValueError):
            make_person_sdf('robot_01',0,0)

if __name__=='__main__': unittest.main()
