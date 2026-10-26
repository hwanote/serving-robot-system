import math
import unittest

from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker

from serving_robot_mapping.frontier_marker_adapter import (
    frontier_representatives,
)
from serving_robot_mapping.exploration_manager import (
    ExplorationManager,
)


def make_points(groups, resolution=0.05):
    result = []

    for cells in groups:
        for col, row in cells:
            result.append(Point(
                x=(col + 0.5) * resolution,
                y=(row + 0.5) * resolution,
                z=0.0,
            ))

    return result


class AdapterTests(unittest.TestCase):

    def test_two_clusters_are_represented(self):
        points = make_points([
            [(x, 2) for x in range(20)],
            [(x, 30) for x in range(30, 50)],
        ])

        result = frontier_representatives(
            points, 0.05, limit=10
        )

        self.assertGreaterEqual(len(result), 2)

        self.assertLess(result[0][1], 0.5)
        self.assertGreater(result[1][1], 1.0)

    def test_limit_respected(self):
        groups = [
            [(x, y) for x in range(10)]
            for y in range(0, 100, 3)
        ]

        result = frontier_representatives(
            make_points(groups), 0.05, limit=10
        )

        self.assertEqual(len(result), 10)

    def test_empty_marker(self):
        self.assertEqual(
            frontier_representatives([], 0.05), []
        )

    def test_nonfinite_points_ignored(self):
        points = [
            Point(x=math.nan, y=0.0),
            Point(x=0.025, y=0.025),
        ]

        self.assertEqual(
            len(frontier_representatives(points, 0.05)),
            1,
        )

    def test_invalid_resolution_fails_closed(self):
        self.assertEqual(
            frontier_representatives(
                make_points([[(1, 1)]]), 0.0
            ),
            [],
        )

    def test_ros_marker_constants(self):
        self.assertEqual(Marker.POINTS, 8)
        self.assertEqual(Marker.SPHERE_LIST, 7)

    def test_explorer_converts_points_for_planner(self):
        class Guard:
            grid = (
                (), 100, 100, 0.05,
                0.0, 0.0, 0.0,
            )

            def observation_candidates(
                self, x, y, max_results=2
            ):
                return [(x, y)]

        class Planner:
            forwarded = None

            def update_candidates(self, marker):
                self.forwarded = marker

        class Logger:
            def info(self, message):
                pass

        class FakeExplorer:
            navigation_enabled = True
            frontier_clearance_guard = Guard()
            planner_client = Planner()
            _last_observation_summary = None

            def get_logger(self):
                return Logger()

        explorer = FakeExplorer()

        marker = Marker()
        marker.header.frame_id = "map"
        marker.action = Marker.ADD
        marker.type = Marker.POINTS
        marker.points = make_points([
            [(x, 2) for x in range(5)],
            [(x, 25) for x in range(5)],
        ])

        ExplorationManager.mapping_update_candidates(
            explorer, marker
        )

        forwarded = explorer.planner_client.forwarded

        self.assertIsNotNone(forwarded)
        self.assertEqual(
            forwarded.type, Marker.SPHERE_LIST
        )
        self.assertEqual(
            forwarded.action, Marker.ADD
        )
        self.assertGreaterEqual(len(forwarded.points), 2)
        self.assertEqual(marker.type, Marker.POINTS)


if __name__ == "__main__":
    unittest.main(verbosity=2)
