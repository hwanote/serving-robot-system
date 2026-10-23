from types import SimpleNamespace
from unittest.mock import Mock

from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker

from serving_robot_mapping.exploration_manager import ExplorationManager


def make_case(points, visited, previous_signature=(), hold=38):
    forwarded = []
    planner = SimpleNamespace(
        signature=previous_signature,
        update_candidates=forwarded.append,
    )

    manager = SimpleNamespace(
        navigation_enabled=True,
        visited_frontiers=visited,
        _visited_only_hold_revision=hold,
        frontier_clearance_guard=SimpleNamespace(
            observation_candidates=lambda x, y, max_results=2: [
                (x, y)
            ]
        ),
        planner_client=planner,
        get_logger=lambda: Mock(),
    )

    marker = Marker()
    marker.header.frame_id = "map"
    marker.type = Marker.SPHERE_LIST
    marker.action = Marker.ADD
    marker.points = [
        Point(x=float(x), y=float(y), z=0.0)
        for x, y in points
    ]

    ExplorationManager.mapping_update_candidates(manager, marker)
    return manager, forwarded[0]


def test_reached_goals_excluded_before_planner():
    manager, marker = make_case(
        points=[
            (5.57, 4.49),
            (5.34, 4.40),
            (6.20, 3.70),
        ],
        visited=[(5.44, 4.55)],
        previous_signature=((5.57, 4.49),),
    )

    assert marker.action == Marker.ADD
    assert len(marker.points) == 1
    assert abs(marker.points[0].x - 6.20) < 1e-6
    assert abs(marker.points[0].y - 3.70) < 1e-6
    assert manager._visited_only_hold_revision is None


def test_only_reached_goals_emit_delete_not_completion():
    manager, marker = make_case(
        points=[
            (5.57, 4.49),
            (5.34, 4.40),
        ],
        visited=[(5.44, 4.55)],
    )

    assert marker.action == Marker.DELETE
    assert len(marker.points) == 0
    assert manager._visited_only_hold_revision == 38


def test_no_visit_history_keeps_candidate():
    manager, marker = make_case(
        points=[(5.57, 4.49)],
        visited=[],
    )

    assert marker.action == Marker.ADD
    assert len(marker.points) == 1
