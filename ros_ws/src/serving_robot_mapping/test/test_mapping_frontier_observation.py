import math
import unittest

from serving_robot_mapping.mapping_frontier_clearance import (
    FrontierClearanceGuard,
)


class ObservationTests(unittest.TestCase):
    def setUp(self):
        self.guard = FrontierClearanceGuard()
        self.cells = [0] * (60 * 60)

    def update(self, yaw=0.0):
        self.assertTrue(
            self.guard.update_map(
                self.cells, 60, 60, 0.1,
                0.0, 0.0, yaw,
            )
        )

    def test_no_map_fails_closed(self):
        self.assertEqual(
            self.guard.observation_candidates(3.0, 3.0),
            [],
        )

    def test_safe_goal_preserved(self):
        self.update()
        self.assertEqual(
            self.guard.observation_candidates(3.0, 3.0),
            [(3.0, 3.0)],
        )

    def test_unsafe_goal_has_safe_alternative(self):
        self.cells[30 * 60 + 32] = 100
        self.update()

        self.assertFalse(
            self.guard.goal_clear(3.0, 3.0)[0]
        )

        options = self.guard.observation_candidates(
            3.0, 3.0
        )

        self.assertGreater(len(options), 0)

        for x, y in options:
            self.assertTrue(
                self.guard.goal_clear(x, y)[0]
            )
            self.assertTrue(
                self.guard._observation_line_clear(
                    x, y, 3.0, 3.0
                )
            )

    def test_maximum_two_candidates(self):
        self.cells[30 * 60 + 32] = 100
        self.update()

        options = self.guard.observation_candidates(
            3.0, 3.0, max_results=2
        )

        self.assertLessEqual(len(options), 2)

    def test_no_free_space_fails_closed(self):
        self.cells = [100] * (60 * 60)
        self.update()

        self.assertEqual(
            self.guard.observation_candidates(3.0, 3.0),
            [],
        )

    def test_unknown_area_not_used_as_goal(self):
        self.cells[30 * 60 + 30] = -1
        self.update()

        self.assertFalse(
            self.guard.goal_clear(3.05, 3.05)[0]
        )

        for x, y in self.guard.observation_candidates(
            3.05, 3.05
        ):
            self.assertTrue(
                self.guard.goal_clear(x, y)[0]
            )

    def test_occupied_line_of_sight_blocked(self):
        for row in range(60):
            self.cells[row * 60 + 25] = 100

        self.update()

        self.assertFalse(
            self.guard._observation_line_clear(
                2.0, 3.0, 3.0, 3.0
            )
        )

    def test_nonfinite_frontier_rejected(self):
        self.update()

        self.assertEqual(
            self.guard.observation_candidates(
                math.nan, 3.0
            ),
            [],
        )


if __name__ == "__main__":
    unittest.main(verbosity=2)
