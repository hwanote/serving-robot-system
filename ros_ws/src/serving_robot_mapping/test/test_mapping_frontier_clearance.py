import math
import unittest

from serving_robot_mapping.mapping_frontier_clearance import (
    FrontierClearanceGuard,
)


class FrontierClearanceTests(unittest.TestCase):
    def setUp(self):
        self.guard = FrontierClearanceGuard()
        self.data = [0] * 400

    def update(self, yaw=0.0):
        self.assertTrue(self.guard.update_map(
            data=self.data,
            width=20,
            height=20,
            resolution=0.1,
            origin_x=0.0,
            origin_y=0.0,
            origin_yaw=yaw,
        ))

    def test_no_map_rejected(self):
        self.assertFalse(
            self.guard.goal_clear(1.0, 1.0)[0]
        )

    def test_clear_goal_accepted(self):
        self.update()
        self.assertTrue(
            self.guard.goal_clear(1.0, 1.0)[0]
        )

    def test_occupied_neighbor_rejected(self):
        self.data[10 * 20 + 13] = 100
        self.update()
        self.assertFalse(
            self.guard.goal_clear(1.0, 1.0)[0]
        )

    def test_other_clear_goal_accepted(self):
        self.data[10 * 20 + 13] = 100
        self.update()
        # Keep the goal inside the 0.52m map-edge guard,
        # and sufficiently far from the occupied cell.
        self.assertTrue(
            self.guard.goal_clear(0.7, 0.7)[0]
        )

    def test_unknown_goal_rejected(self):
        self.data[10 * 20 + 10] = -1
        self.update()
        self.assertFalse(
            self.guard.goal_clear(1.0, 1.0)[0]
        )

    def test_outside_map_rejected(self):
        self.update()
        self.assertFalse(
            self.guard.goal_clear(3.0, 3.0)[0]
        )

    def test_rotated_map_coordinates(self):
        self.update(yaw=math.pi / 2)
        self.assertTrue(
            self.guard.goal_clear(-1.0, 1.0)[0]
        )

    def test_start_close_obstacle_blocked(self):
        self.assertFalse(self.guard.start_clear(
            [2.0] * 100 + [0.46],
            0.05, 10.0, 0.1,
        )[0])

    def test_start_clear_accepted(self):
        self.assertTrue(self.guard.start_clear(
            [0.8] * 100,
            0.05, 10.0, 0.1,
        )[0])

    def test_missing_scan_blocked(self):
        self.assertFalse(self.guard.start_clear(
            [], 0.05, 10.0, 0.1,
        )[0])

    def test_stale_scan_blocked(self):
        self.assertFalse(self.guard.start_clear(
            [1.0] * 100,
            0.05, 10.0, 3.0,
        )[0])

    def test_unsafe_clearance_configuration_rejected(self):
        with self.assertRaises(ValueError):
            FrontierClearanceGuard(
                start_clearance_m=0.3
            )


if __name__ == "__main__":
    unittest.main(verbosity=2)
