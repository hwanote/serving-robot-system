"""Straight rail of one leg as a list of poses (no ROS).

The route executor sends every leg along the route graph as a path for the
Nav2 controller (FollowPath), not as a goal for the planner: the path is the
straight line from where the robot is to the leg's node, one pose every
`step_m` ("sub-nodes"). The controller follows the line; nothing replans it
around an obstacle. The last pose carries the heading the robot turns to at
the node (the rotation shim controller turns in place there).
"""
import math


def rail_poses(x0, y0, x1, y1, end_yaw, step_m=0.05):
    """[(x, y, yaw), ...] from (x0, y0) to (x1, y1), at most step_m apart.
    Every pose faces along the line; the last one has `end_yaw`."""
    length = math.hypot(x1 - x0, y1 - y0)
    heading = math.atan2(y1 - y0, x1 - x0) if length > 1e-6 else end_yaw
    n = max(1, int(math.ceil(length / step_m)))
    poses = [(x0 + (x1 - x0) * i / n, y0 + (y1 - y0) * i / n, heading)
             for i in range(n)]
    poses.append((x1, y1, end_yaw))
    return poses
