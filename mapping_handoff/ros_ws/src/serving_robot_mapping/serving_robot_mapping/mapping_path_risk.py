"""Mapping-only, store-independent occupancy-grid path screening.

No ROS/Nav2 dependency. Does NOT replace the Nav2 costmaps, collision monitor,
soft-stop limiter, live LiDAR, or a live obstacle-hold check.

The caller MUST pass the actual path geometry returned by the planner.
Checking just (goal.x, goal.y, path_length_m) is insufficient and is rejected.
"""

from dataclasses import dataclass
import math


@dataclass(frozen=True)
class PathRiskResult:
    allowed: bool
    reason: str
    samples: int = 0
    minimum_occupied_distance_m: float | None = None


class MappingPathRiskGuard:
    """Rejects invalid/unknown path centerlines and near-occupied paths.

    Unknown, uncertain and outside-map space block the clearance
    envelope of each actual planner path sample.
    Nav2 and live collision protection remain mandatory.

    All geometry is taken from the current OccupancyGrid; no venue coordinates,
    map dimensions, or cafe landmarks are hard-coded.
    """

    def __init__(self, occupied_clearance_m=0.48, max_sample_spacing_m=0.025):
        if not math.isfinite(occupied_clearance_m) or occupied_clearance_m < 0.0:
            raise ValueError("occupied_clearance_m must be finite and >= 0")
        if not math.isfinite(max_sample_spacing_m) or max_sample_spacing_m <= 0:
            raise ValueError("max_sample_spacing_m must be finite and > 0")
        self.occupied_clearance_m = float(occupied_clearance_m)
        self.max_sample_spacing_m = float(max_sample_spacing_m)
        self._grid = None
        self._occupied = ()

    def update_grid(self, data, width, height, resolution,
                    origin_x, origin_y, origin_yaw=0.0):
        """Accept a ROS OccupancyGrid's numeric fields (not ROS message itself)."""
        if (not isinstance(width, int) or isinstance(width, bool)
                or not isinstance(height, int) or isinstance(height, bool)
                or width < 1 or height < 1 or len(data) != width * height):
            self._grid = None
            self._occupied = ()
            return False
        if not all(math.isfinite(v) for v in
                   (resolution, origin_x, origin_y, origin_yaw)) or resolution <= 0:
            self._grid = None
            self._occupied = ()
            return False
        values = tuple(int(v) for v in data)
        if any((v < -1 or v > 100) for v in values):
            self._grid = None
            self._occupied = ()
            return False
        self._grid = (values, width, height, float(resolution),
                      float(origin_x), float(origin_y), float(origin_yaw))
        self._occupied = tuple((i % width, i // width)
                               for i, v in enumerate(values) if v >= 65)
        return True

    def _world_to_grid(self, x, y):
        _, _, _, res, ox, oy, yaw = self._grid
        dx, dy = x - ox, y - oy
        c, s = math.cos(yaw), math.sin(yaw)
        return (c * dx + s * dy) / res, (-s * dx + c * dy) / res

    def _check_point(self, x, y, cells):
        values, width, height, res, _, _, _ = self._grid
        gx, gy = self._world_to_grid(x, y)
        col, row = math.floor(gx), math.floor(gy)
        if not (0 <= col < width and 0 <= row < height):
            return False, 'PATH_OUTSIDE_MAP', None
        if not 0 <= values[row * width + col] <= 25:
            return False, 'PATH_CENTER_NOT_KNOWN_FREE', None

        # Block path samples whose clearance envelope
        # extends beyond the current map.
        edge_distance = min(
            gx, gy, width - gx, height - gy
        ) * res
        if edge_distance < self.occupied_clearance_m:
            return False, 'PATH_NEAR_MAP_EDGE', None

        # Conservative cell-half-diagonal inflation, independent of map yaw.
        inflated = self.occupied_clearance_m + res * math.sqrt(2.0) / 2.0
        span = int(math.ceil(inflated / res)) + 1
        d_min = None
        for rr in range(max(0, row - span), min(height, row + span + 1)):
            for cc in range(max(0, col - span), min(width, col + span + 1)):
                value = values[rr * width + cc]
                if 0 <= value <= 25:
                    continue

                distance = math.hypot((cc + 0.5 - gx) * res,
                                      (rr + 0.5 - gy) * res)

                # Keep occupied-distance reporting limited
                # to observed occupied cells.
                if value >= 65:
                    d_min = (
                        distance if d_min is None
                        else min(d_min, distance)
                    )

                if distance <= inflated:
                    if value == -1:
                        return False, 'PATH_NEAR_UNKNOWN', d_min
                    if value < 65:
                        return False, 'PATH_NEAR_UNCERTAIN', d_min
                    return False, 'PATH_NEAR_OCCUPIED', d_min
        return True, 'PATH_SAMPLE_CLEAR', d_min

    def check_path(self, points):
        """points: 2+ actual planner path points, each an (x, y) pair.

        Never synthesize a straight line from start+goal as a replacement for
        the planner's path; a valid Nav2 path may bend around obstacles.
        """
        if self._grid is None:
            return PathRiskResult(False, 'MAP_NOT_READY')
        if points is None:
            return PathRiskResult(False, 'PLANNER_PATH_UNAVAILABLE')
        try:
            coords = [(float(x), float(y)) for x, y in points]
        except (TypeError, ValueError):
            return PathRiskResult(False, 'PLANNER_PATH_FORMAT_INVALID')
        if len(coords) < 2:
            return PathRiskResult(False, 'PLANNER_PATH_TOO_SHORT')
        if not all(math.isfinite(v) for xy in coords for v in xy):
            return PathRiskResult(False, 'PLANNER_PATH_NONFINITE')

        res = self._grid[3]
        step = min(self.max_sample_spacing_m, res * 0.5)
        checked = 0
        minimum = None
        for a, b in zip(coords, coords[1:]):
            length = math.hypot(b[0] - a[0], b[1] - a[1])
            n = max(1, math.ceil(length / step))
            for i in range(n + 1):
                t = i / n
                x, y = a[0] + (b[0] - a[0]) * t, a[1] + (b[1] - a[1]) * t
                allowed, reason, d = self._check_point(x, y, None)
                checked += 1
                if d is not None:
                    minimum = d if minimum is None else min(minimum, d)
                if not allowed:
                    return PathRiskResult(False, reason, checked, minimum)
        return PathRiskResult(True, 'PATH_GRID_SCREEN_PASS', checked, minimum)
