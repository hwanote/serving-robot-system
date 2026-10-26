import unittest

from cafe_autonomy.yield_policy import candidate_poses, assess_path, choose_yield


class YieldPolicyTests(unittest.TestCase):
    def setUp(self):
        self.zones = {"lane": (2.6, 3.8, -0.2, 4.45)}

    def test_candidate_never_uses_home_or_occupied_lane(self):
        poses = candidate_poses((3.2, 1.0, 1.57), (3.2, 2.2),
                                [(3.2, 0.0), (3.2, -1.0)], self.zones, {},
                                [(3.2, -1.0), (5.6, 0.0)], None, 0.9, 0.45)
        self.assertTrue(poses)
        self.assertTrue(all(not (2.15 <= x <= 4.25 and -0.65 <= y <= 4.9)
                            for x, y, _, _ in poses))
        self.assertTrue(all((x, y) != (3.2, -1.0) for x, y, _, _ in poses))

    def test_path_near_other_robot_rejected(self):
        self.assertIsNone(assess_path([(0, 0), (1, 0), (2, 0)],
                                      (0, 0), (2, 0), (1, 0.5), .9, .45, .1))

    def test_safe_path_accepted(self):
        result = assess_path([(0, 0), (0, 1), (0, 2)],
                             (0, 0), (0, 2), (2, 1), .9, .45, .1)
        self.assertIsNotNone(result)
        self.assertGreaterEqual(result["peer_clearance_m"], .9)

    def test_either_robot_can_yield_and_no_route_is_safe_wait(self):
        one = {"robot": "robot_01", "rear_turn": False, "length_m": 1.1, "turn_rad": .2}
        two = {"robot": "robot_02", "rear_turn": False, "length_m": .9, "turn_rad": .3}
        self.assertEqual(choose_yield([one, two])["robot"], "robot_02")
        self.assertEqual(choose_yield([dict(one, length_m=.7), two])["robot"], "robot_01")
        self.assertIsNone(choose_yield([]))

    def test_rear_turn_only_after_front_side_paths_fail(self):
        rear = {"robot": "robot_01", "rear_turn": True, "length_m": .9, "turn_rad": 3.0}
        side = {"robot": "robot_02", "rear_turn": False, "length_m": 2.0, "turn_rad": 1.0}
        self.assertEqual(choose_yield([rear, side])["robot"], "robot_02")
        self.assertEqual(choose_yield([rear])["robot"], "robot_01")


if __name__ == "__main__":
    unittest.main()
