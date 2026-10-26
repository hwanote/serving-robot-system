"""Remove laser points that hit KNOWN obstacles (walls, counter, table legs, pots).

The map is assumed to contain fixed objects only. A laser point that lands on
(or within `tolerance_m` of) an occupied map cell is a known obstacle; every
other point belongs to something the map does not know (a chair, a person,
belongings) and is kept. The collision monitor's stop / slow zones look only at
the kept points, so the robot may come close to walls while still reacting to
anything unexpected.

No ROS dependency (numpy only), so it is unit-testable anywhere.
"""
import math

import numpy as np


def dilate(occupied, radius_cells):
    """Grow True cells by `radius_cells` (circular)."""
    r = int(radius_cells)
    if r <= 0:
        return occupied.copy()
    h, w = occupied.shape
    padded = np.pad(occupied, r, mode='constant', constant_values=False)
    out = np.zeros_like(occupied)
    for dy in range(-r, r + 1):
        for dx in range(-r, r + 1):
            if dx * dx + dy * dy > r * r:
                continue
            out |= padded[r + dy:r + dy + h, r + dx:r + dx + w]
    return out


class StaticMask:
    """Occupied cells of the map, grown by the tolerance."""

    def __init__(self, data, width, height, resolution, origin_x, origin_y,
                 tolerance_m=0.10, occupied_threshold=65):
        grid = np.asarray(data, dtype=np.int16).reshape(height, width)
        occupied = grid >= occupied_threshold      # unknown (-1) is not occupied
        self.mask = dilate(occupied, int(math.ceil(tolerance_m / resolution)))
        self.width = int(width)
        self.height = int(height)
        self.resolution = float(resolution)
        self.origin_x = float(origin_x)
        self.origin_y = float(origin_y)

    def is_static(self, xs, ys):
        """Boolean array: does each map-frame point lie on a known obstacle?"""
        xs = np.asarray(xs, dtype=float)
        ys = np.asarray(ys, dtype=float)
        ix = np.floor((xs - self.origin_x) / self.resolution).astype(int)
        iy = np.floor((ys - self.origin_y) / self.resolution).astype(int)
        inside = (ix >= 0) & (ix < self.width) & (iy >= 0) & (iy < self.height)
        out = np.zeros(xs.shape, dtype=bool)
        out[inside] = self.mask[iy[inside], ix[inside]]
        return out


def filter_ranges(ranges, angle_min, angle_increment, range_min, range_max,
                  sensor_pose, static_mask):
    """Return (new_ranges, removed_count).

    `sensor_pose` = (x, y, yaw) of the laser in the map frame. Points on known
    obstacles become `inf` (= no return); all other points are unchanged.
    """
    r = np.asarray(ranges, dtype=float)
    out = r.copy()
    valid = np.flatnonzero(np.isfinite(r) & (r >= range_min) & (r <= range_max))
    if valid.size == 0:
        return out, 0
    tx, ty, yaw = sensor_pose
    a = angle_min + angle_increment * valid + yaw
    xs = tx + r[valid] * np.cos(a)
    ys = ty + r[valid] * np.sin(a)
    static = static_mask.is_static(xs, ys)
    out[valid[static]] = np.inf
    return out, int(static.sum())
