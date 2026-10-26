"""Focused regression for map coverage and safe map replacement."""
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PoseWithCovarianceStamped
import rclpy

from cafe_autonomy.map_odom_guard import MapOdomGuard, known_fraction


def grid():
    msg = OccupancyGrid()
    msg.info.width = 10
    msg.info.height = 10
    msg.info.resolution = 1.0
    msg.info.origin.orientation.w = 1.0
    msg.data = [-1] * 100
    return msg


def test_coverage_uses_loaded_map_cells():
    msg = grid()
    assert known_fraction(msg, 5.0, 5.0, 2.0) == 0.0
    for row in range(3, 7):
        for col in range(3, 7):
            msg.data[row * 10 + col] = 0
    assert known_fraction(msg, 5.0, 5.0, 2.0) > 0.2
    assert known_fraction(msg, 0.0, 0.0, 1.0) == 0.0


def test_new_map_withholds_tf_until_explicit_initial_pose():
    rclpy.init()
    node = MapOdomGuard()
    try:
        first = grid()
        node.on_map(first)
        assert not node.pending_initial_pose
        changed = grid()
        changed.data[55] = 0
        node.on_map(changed)
        assert node.pending_initial_pose
        node.odom = (0.0, 0.0, 0.0)
        pose = PoseWithCovarianceStamped()
        pose.header.frame_id = "map"
        pose.pose.pose.position.x = 1.0
        pose.pose.pose.position.y = 2.0
        pose.pose.pose.orientation.w = 1.0
        node.on_initial_pose(pose)
        assert not node.pending_initial_pose
        assert (node.tx, node.ty) == (1.0, 2.0)
    finally:
        node.destroy_node()
        rclpy.shutdown()
