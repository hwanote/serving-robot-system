"""Detect frontiers between free and unknown occupancy-grid cells."""

import math
from collections import deque
from dataclasses import dataclass
from typing import Sequence


@dataclass(frozen=True)
class FrontierCluster:
    """A connected group of frontier cells."""

    cells: tuple[tuple[int, int], ...]
    estimated_length_m: float


class FrontierDetector:
    """Detect frontier clusters in a 2D occupancy grid."""

    # Used to identify free cells bordering unknown space.
    NEIGHBORS_4 = (
        (1, 0),
        (-1, 0),
        (0, 1),
        (0, -1),
    )

    # Used to connect neighboring frontier cells into clusters.
    NEIGHBORS_8 = (
        (-1, -1), (0, -1), (1, -1),
        (-1, 0),           (1, 0),
        (-1, 1),  (0, 1),  (1, 1),
    )

    def __init__(
        self,
        min_size_m: float = 0.40,
        free_threshold: int = 25,
    ) -> None:
        if min_size_m < 0:
            raise ValueError("min_size_m must be non-negative")

        if not 0 <= free_threshold <= 100:
            raise ValueError("free_threshold must be between 0 and 100")

        self.min_size_m = min_size_m
        self.free_threshold = free_threshold

    def detect(
        self,
        data: Sequence[int],
        width: int,
        height: int,
        resolution: float,
    ) -> list[FrontierCluster]:
        """Return frontier clusters sorted by descending size."""

        if width <= 0 or height <= 0 or resolution <= 0:
            raise ValueError("Invalid occupancy-grid dimensions")

        if len(data) != width * height:
            raise ValueError("Occupancy-grid data length mismatch")

        def inside(x: int, y: int) -> bool:
            return 0 <= x < width and 0 <= y < height

        def value_at(x: int, y: int) -> int:
            return data[y * width + x]

        # Step 1: Find free cells next to unknown cells.
        frontier_cells = set()

        for y in range(height):
            for x in range(width):
                value = value_at(x, y)

                if not 0 <= value <= self.free_threshold:
                    continue

                for dx, dy in self.NEIGHBORS_4:
                    nx = x + dx
                    ny = y + dy

                    if inside(nx, ny) and value_at(nx, ny) == -1:
                        frontier_cells.add((x, y))
                        break

        # Step 2: Group connected frontier cells.
        visited = set()
        clusters = []

        min_cells = max(
            1,
            math.ceil(self.min_size_m / resolution),
        )

        for start in sorted(frontier_cells):
            if start in visited:
                continue

            queue = deque([start])
            visited.add(start)
            cluster_cells = []

            while queue:
                x, y = queue.popleft()
                cluster_cells.append((x, y))

                for dx, dy in self.NEIGHBORS_8:
                    neighbor = (x + dx, y + dy)

                    if (
                        neighbor in frontier_cells
                        and neighbor not in visited
                    ):
                        visited.add(neighbor)
                        queue.append(neighbor)

            # Step 3: Discard small frontier clusters.
            if len(cluster_cells) < min_cells:
                continue

            clusters.append(
                FrontierCluster(
                    cells=tuple(cluster_cells),
                    estimated_length_m=(
                        len(cluster_cells) * resolution
                    ),
                )
            )

        # Step 4: Process larger frontiers first.
        clusters.sort(
            key=lambda cluster: len(cluster.cells),
            reverse=True,
        )

        return clusters
