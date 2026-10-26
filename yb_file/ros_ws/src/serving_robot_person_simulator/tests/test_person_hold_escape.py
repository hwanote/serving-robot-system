import unittest
from serving_robot_person_simulator.policy import can_escape_person_hold


class PersonHoldEscapeTests(unittest.TestCase):
    def test_person08_can_move_away(self):
        p8 = (2.02668, 2.3)
        p6 = (1.47880, 1.7)
        self.assertTrue(can_escape_person_hold(
            p8, (0.15, 0.0), [(p6, (0.0, 0.0))], 0.8, 1.0
        ))

    def test_moving_toward_person_is_blocked(self):
        self.assertFalse(can_escape_person_hold(
            (2.02668, 2.3), (-0.15, 0.0),
            [((1.47880, 1.7), (0.0, 0.0))], 0.8, 1.0
        ))

    def test_inside_hard_stop_is_blocked(self):
        self.assertFalse(can_escape_person_hold(
            (0.75, 0.0), (0.15, 0.0),
            [((0.0, 0.0), (0.0, 0.0))], 0.8, 1.0
        ))

    def test_other_person_approaching_faster_is_blocked(self):
        self.assertFalse(can_escape_person_hold(
            (2.02668, 2.3), (0.15, 0.0),
            [((1.47880, 1.7), (0.4, 0.0))], 0.8, 1.0
        ))

    def test_tangential_motion_is_blocked(self):
        self.assertFalse(can_escape_person_hold(
            (0.0, 0.0), (0.15, 0.0),
            [((0.0, 0.85), (0.0, 0.0))], 0.8, 1.0
        ))
