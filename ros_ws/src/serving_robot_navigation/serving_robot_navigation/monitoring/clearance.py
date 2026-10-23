"""Facts from the LiDAR and the contact sensor (no ROS dependency).

    forward_clearance(...)   distance from the body front to the nearest thing ahead
    touched_name(...)        name of the outside thing the robot touches

Used by the person detector, the person speed limiter and the soft stop limiter.
"""
import math

import numpy as np


def forward_clearance(ranges, angle_min, angle_increment, range_min, range_max,
                      front, half_width):
    """Distance from the body front to the nearest point straight ahead.

    Looks at the corridor in front of the robot (as wide as the body).
    Returns math.inf when nothing is there. `ranges` should be the scan without
    known obstacles (scan_dynamic). The sensor sits at the base centre.
    """
    r = np.asarray(ranges, dtype=float)
    idx = np.flatnonzero(np.isfinite(r) & (r >= range_min) & (r <= range_max))
    if idx.size == 0:
        return math.inf
    a = angle_min + angle_increment * idx
    x = r[idx] * np.cos(a)
    y = r[idx] * np.sin(a)
    ahead = (x > front) & (np.abs(y) <= half_width)
    if not ahead.any():
        return math.inf
    return float(x[ahead].min() - front)


def touched_name(contacts, self_model):
    """Name of the first outside thing the robot touches, '' when there is none.

    `contacts` is the list of ros_gz_interfaces/Contact (collision1/collision2
    with a scoped `name`, e.g. 'robot_01::link::collision'). A contact between
    two parts of the robot itself is not a collision with the world.
    """
    own = self_model + '::'

    def is_own(name):
        return name.startswith(own) or ('::' + own) in name

    for c in contacts:
        a, b = c.collision1.name, c.collision2.name
        if is_own(a) and is_own(b):
            continue
        return b if is_own(a) else a
    return ''
