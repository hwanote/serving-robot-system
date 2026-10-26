"""Choose reachable, clearance-safe approach cells for map frontiers."""

import heapq
import math
from collections import deque
from dataclasses import dataclass
from typing import Sequence

from serving_robot_mapping.frontier_detector import FrontierCluster


@dataclass(frozen=True)
class FrontierGoal:
    """Candidate approach point, not yet approved by Nav2."""

    cluster_size_cells: int
    cell: tuple[int, int]
    x: float
    y: float
    travel_distance_m: float
    frontier_distance_m: float


class FrontierSelector:
    """Find one safe, grid-reachable goal per frontier cluster."""

    MOVES_8 = (
        (-1, 0), (1, 0),
        (0, -1), (0, 1),
        (-1, -1), (-1, 1),
        (1, -1), (1, 1),
    )

    MOVES_4 = (
        (-1, 0), (1, 0),
        (0, -1), (0, 1),
    )

    def __init__(
        self,
        goal_clearance_m: float = 0.35,
        max_approach_distance_m: float = 1.0,
        min_goal_distance_m: float = 0.35,
        free_threshold: int = 25,
    ) -> None:
        if goal_clearance_m < 0 or max_approach_distance_m <= 0:
            raise ValueError("Invalid clearance or approach distance")

        if min_goal_distance_m < 0 or not 0 <= free_threshold <= 100:
            raise ValueError("Invalid minimum distance or threshold")

        self.goal_clearance_m = goal_clearance_m
        self.max_approach_distance_m = max_approach_distance_m
        self.min_goal_distance_m = min_goal_distance_m
        self.free_threshold = free_threshold

    def select(
        self,
        data: Sequence[int],
        width: int,
        height: int,
        resolution: float,
        origin_x: float,
        origin_y: float,
        origin_yaw: float,
        robot_x: float,
        robot_y: float,
        clusters: Sequence[FrontierCluster],
    ) -> list[FrontierGoal]:
        """Select reachable frontier approach goals."""

        if width <= 0 or height <= 0 or resolution <= 0:
            raise ValueError("Invalid map dimensions or resolution")

        if len(data) != width * height:
            raise ValueError("Occupancy grid size mismatch")

        if not clusters:
            return []

        cos_yaw = math.cos(origin_yaw)
        sin_yaw = math.sin(origin_yaw)

        # Convert robot world position to map grid coordinates.
        dx = robot_x - origin_x
        dy = robot_y - origin_y

        robot_cx = math.floor(
            (cos_yaw * dx + sin_yaw * dy) / resolution
        )
        robot_cy = math.floor(
            (-sin_yaw * dx + cos_yaw * dy) / resolution
        )

        if not (
            0 <= robot_cx < width
            and 0 <= robot_cy < height
        ):
            raise ValueError("Robot pose lies outside the map")

        # Known-free cells only.
        free = [
            0 <= value <= self.free_threshold
            for value in data
        ]

        # Remove cells too close to obstacles or unknown space.
        safe = self._clearance_mask(
            free, width, height, resolution
        )

        start = robot_cy * width + robot_cx

        # The current robot pose may be close to unexplored
        # space during initial mapping.
        if not free[start]:
            raise ValueError(
                "Robot start cell is not known-free"
            )

        # Coarse connectivity through known-free cells.
        # This does NOT certify a collision-free path.
        # Nav2 must validate the actual navigation path.
        distances = self._travel_distances(
            free, width, height, resolution, start
        )

        max_steps = math.floor(
            self.max_approach_distance_m / resolution
        )

        goals = []

        for cluster in clusters:
            queue = deque()
            seen = set()

            # Start searching inward from all cells in a frontier.
            for x, y in cluster.cells:
                if 0 <= x < width and 0 <= y < height:
                    index = y * width + x

                    if free[index] and index not in seen:
                        queue.append((index, 0))
                        seen.add(index)

            best = None

            while queue:
                index, steps = queue.popleft()
                travel = distances[index]

                # Candidate must be safe and robot-reachable.
                if (
                    safe[index]
                    and math.isfinite(travel)
                    and travel >= self.min_goal_distance_m
                ):
                    rank = (steps, travel, index)

                    if best is None or rank < best[0]:
                        best = (rank, index)

                if steps >= max_steps:
                    continue

                x = index % width
                y = index // width

                for ox, oy in self.MOVES_4:
                    nx = x + ox
                    ny = y + oy

                    if 0 <= nx < width and 0 <= ny < height:
                        nxt = ny * width + nx

                        if free[nxt] and nxt not in seen:
                            seen.add(nxt)
                            queue.append((nxt, steps + 1))

            if best is None:
                continue

            (approach_steps, travel, _), index = best

            cx = index % width
            cy = index // width

            # Convert selected grid cell center into world position.
            local_x = (cx + 0.5) * resolution
            local_y = (cy + 0.5) * resolution

            world_x = (
                origin_x
                + cos_yaw * local_x
                - sin_yaw * local_y
            )

            world_y = (
                origin_y
                + sin_yaw * local_x
                + cos_yaw * local_y
            )

            goals.append(
                FrontierGoal(
                    cluster_size_cells=len(cluster.cells),
                    cell=(cx, cy),
                    x=world_x,
                    y=world_y,
                    travel_distance_m=travel,
                    frontier_distance_m=(
                        approach_steps * resolution
                    ),
                )
            )

        # Prefer nearby reachable candidates for now.
        sorted_goals = sorted(
            goals,
            key=lambda goal: (
                goal.travel_distance_m,
                -goal.cluster_size_cells,
            ),
        )

        # Remove duplicate goal cells.
        unique_goals = []
        seen_cells = set()

        for goal in sorted_goals:
            if goal.cell in seen_cells:
                continue

            seen_cells.add(goal.cell)
            unique_goals.append(goal)

        return unique_goals

    def _clearance_mask(
        self,
        free: list[bool],
        width: int,
        height: int,
        resolution: float,
    ) -> list[bool]:
        """Conservatively erode free space using a square footprint."""

        stride = width + 1
        prefix = [0] * ((height + 1) * stride)

        # Build summed-area table of blocked cells.
        for y in range(height):
            row_blocked = 0

            for x in range(width):
                if not free[y * width + x]:
                    row_blocked += 1

                prefix[(y + 1) * stride + x + 1] = (
                    prefix[y * stride + x + 1]
                    + row_blocked
                )

        radius_cells = math.ceil(
            self.goal_clearance_m / resolution
        )

        safe = [False] * (width * height)

        for y in range(radius_cells, height - radius_cells):
            y0 = y - radius_cells
            y1 = y + radius_cells + 1

            for x in range(radius_cells, width - radius_cells):
                index = y * width + x

                if not free[index]:
                    continue

                x0 = x - radius_cells
                x1 = x + radius_cells + 1

                blocked = (
                    prefix[y1 * stride + x1]
                    - prefix[y0 * stride + x1]
                    - prefix[y1 * stride + x0]
                    + prefix[y0 * stride + x0]
                )

                safe[index] = blocked == 0

        return safe

    def _travel_distances(
        self,
        safe: list[bool],
        width: int,
        height: int,
        resolution: float,
        start: int,
    ) -> list[float]:
        """Dijkstra through known-free cells without corner cutting."""

        distances = [math.inf] * (width * height)
        distances[start] = 0.0

        queue = [(0.0, start)]

        while queue:
            current_dist, index = heapq.heappop(queue)

            if current_dist > distances[index]:
                continue

            x = index % width
            y = index // width

            for ox, oy in self.MOVES_8:
                nx = x + ox
                ny = y + oy

                if not (
                    0 <= nx < width
                    and 0 <= ny < height
                ):
                    continue

                nxt = ny * width + nx

                if not safe[nxt]:
                    continue

                # Do not cut diagonally through blocked corners.
                if ox and oy:
                    if (
                        not safe[y * width + nx]
                        or not safe[ny * width + x]
                    ):
                        continue

                step = resolution * (
                    math.sqrt(2)
                    if ox and oy
                    else 1.0
                )

                trial = current_dist + step

                if trial < distances[nxt]:
                    distances[nxt] = trial
                    heapq.heappush(queue, (trial, nxt))

        return distances
