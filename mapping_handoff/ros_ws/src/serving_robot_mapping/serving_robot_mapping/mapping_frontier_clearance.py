"""Conservative Mapping-only checks; never bypass Nav2 safety."""

import math


class FrontierClearanceGuard:
    def __init__(
        self,
        goal_clearance_m=0.52,
        start_clearance_m=0.50,
    ):
        # Current Navigation HardStop is +/-0.36 x +/-0.30 m.
        # Its circumscribed radius is approximately 0.469 m.
        # Keep the Mapping start guard outside this envelope.
        if not (
            math.isfinite(goal_clearance_m)
            and goal_clearance_m >= 0.48
            and math.isfinite(start_clearance_m)
            and start_clearance_m >= 0.48
        ):
            raise ValueError("Mapping clearance cannot be below 0.48 m")

        self.goal_clearance = float(goal_clearance_m)
        self.start_clearance = float(start_clearance_m)
        self.grid = None

    def update_map(
        self, data, width, height, resolution,
        origin_x, origin_y, origin_yaw,
    ):
        values = (
            float(resolution),
            float(origin_x),
            float(origin_y),
            float(origin_yaw),
        )

        if (
            width <= 0 or height <= 0
            or not all(math.isfinite(v) for v in values)
            or resolution <= 0
            or len(data) != width * height
        ):
            self.grid = None
            return False

        self.grid = (
            tuple(data),
            int(width),
            int(height),
            float(resolution),
            float(origin_x),
            float(origin_y),
            float(origin_yaw),
        )
        return True

    def goal_clear(self, x, y):
        """Check goal clearance from non-free cells and map edges."""
        if self.grid is None:
            return False, "MAP_NOT_READY"

        if not math.isfinite(x) or not math.isfinite(y):
            return False, "GOAL_NOT_FINITE"

        data, width, height, res, ox, oy, yaw = self.grid

        dx = x - ox
        dy = y - oy
        co = math.cos(yaw)
        si = math.sin(yaw)

        mx = co * dx + si * dy
        my = -si * dx + co * dy

        col = math.floor(mx / res)
        row = math.floor(my / res)

        if not (0 <= col < width and 0 <= row < height):
            return False, "GOAL_OUTSIDE_MAP"

        cell = data[row * width + col]

        if cell < 0 or cell > 25:
            return False, "GOAL_NOT_KNOWN_FREE"

        # Outside-map is blocked. Check all four boundaries
        # in the map's local (possibly rotated) coordinates.
        edge_distance = min(
            mx, my, width * res - mx, height * res - my
        )
        if edge_distance < self.goal_clearance:
            return False, "GOAL_NEAR_MAP_EDGE"

        # Unknown, uncertain and occupied cells are blocked.
        radius = self.goal_clearance
        inflated = radius + res * math.sqrt(2) / 2
        span = math.ceil(inflated / res) + 1

        for r in range(max(0, row - span),
                       min(height, row + span + 1)):
            for c in range(max(0, col - span),
                           min(width, col + span + 1)):
                value = data[r * width + c]
                if 0 <= value <= 25:
                    continue

                cx = (c + 0.5) * res
                cy = (r + 0.5) * res

                if math.hypot(cx - mx, cy - my) <= inflated:
                    if value < 0:
                        return False, "GOAL_NEAR_UNKNOWN_CELL"
                    if value < 65:
                        return False, "GOAL_NEAR_UNCERTAIN_CELL"
                    return False, "GOAL_NEAR_OCCUPIED_CELL"

        return True, "GOAL_CLEAR"

    def start_clear(self, ranges, range_min, range_max, wall_age):
        """Fail closed if raw LiDAR suggests a blocked start."""
        if (
            not math.isfinite(wall_age)
            or wall_age < 0
            or wall_age > 2.0
        ):
            return False, "START_SCAN_STALE"

        valid = [
            float(distance)
            for distance in ranges
            if math.isfinite(distance)
            and range_min <= distance <= range_max
        ]

        if len(valid) < 12:
            return False, "START_SCAN_INSUFFICIENT"

        nearest = min(valid)

        if nearest < self.start_clearance:
            return (
                False,
                f"START_RAW_OBSTACLE_{nearest:.3f}M",
            )

        return True, "START_CLEAR"

    def _observation_line_clear(self, x, y, fx, fy):
        """Require known free cells along the observation approach."""
        if self.grid is None:
            return False

        data, width, height, res, ox, oy, yaw = self.grid

        distance = math.hypot(fx - x, fy - y)
        if distance < 0.15:
            return False

        # Frontier itself borders unknown space. The last 12 cm
        # need not be mapped, but the approach must be known free.
        length = max(0.0, distance - 0.12)
        samples = max(1, math.ceil(length / (res * 0.5)))

        co = math.cos(yaw)
        si = math.sin(yaw)

        for index in range(samples + 1):
            ratio = (length * index / samples) / distance

            px = x + (fx - x) * ratio - ox
            py = y + (fy - y) * ratio - oy

            mx = co * px + si * py
            my = -si * px + co * py

            col = math.floor(mx / res)
            row = math.floor(my / res)

            if not (0 <= col < width and 0 <= row < height):
                return False

            value = data[row * width + col]

            if not 0 <= value <= 25:
                return False

        return True

    def observation_candidates(self, x, y, max_results=2):
        """Find safe known-free viewpoints near a Frontier.

        Outputs are NOT navigation approvals. Each generated
        position must still pass Nav2 ComputePathToPose and
        the existing live navigation safety gates.
        """
        if (
            self.grid is None
            or not math.isfinite(x)
            or not math.isfinite(y)
            or max_results < 1
        ):
            return []

        # Preserve an existing safe Frontier goal.
        if self.goal_clear(x, y)[0]:
            return [(x, y)]

        results = []

        # Explore nearby known-free viewpoints without reducing
        # any safety distance or altering Navigation parameters.
        for radius in (0.35, 0.50, 0.70, 0.90, 1.15):
            for index in range(16):
                angle = 2.0 * math.pi * index / 16.0

                cx = x + radius * math.cos(angle)
                cy = y + radius * math.sin(angle)

                if not self.goal_clear(cx, cy)[0]:
                    continue

                if not self._observation_line_clear(
                    cx, cy, x, y
                ):
                    continue

                if any(
                    math.hypot(cx - px, cy - py) < 0.20
                    for px, py in results
                ):
                    continue

                results.append((cx, cy))

                if len(results) >= max_results:
                    return results

        return results
