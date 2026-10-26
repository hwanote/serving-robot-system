"""Independent conservative occupancy-grid completion check."""

import math


def verify_closed_free_region(
    data,
    width,
    height,
    resolution,
    free_threshold=25,
    min_free_area_m2=10.0,
):
    """Reject exposed unknown boundaries and insufficient map coverage."""

    if (
        width < 3 or height < 3
        or len(data) != width * height
        or not math.isfinite(resolution)
        or resolution <= 0
        or not math.isfinite(min_free_area_m2)
        or min_free_area_m2 <= 0
        or not 0 <= free_threshold <= 100
    ):
        return False, "INVALID_MAP_GEOMETRY"

    free_count = 0

    for y in range(height):
        for x in range(width):
            index = y * width + x
            value = data[index]

            if not 0 <= value <= free_threshold:
                continue

            free_count += 1

            # A free cell touching the grid boundary may hide
            # unexplored space beyond the current map extent.
            if (
                x == 0 or y == 0
                or x == width - 1
                or y == height - 1
            ):
                return False, "FREE_SPACE_REACHES_MAP_EDGE"

            neighbors = (
                index - 1,
                index + 1,
                index - width,
                index + width,
            )

            if any(data[n] == -1 for n in neighbors):
                return False, "FREE_SPACE_TOUCHES_UNKNOWN"

    known_free_area = free_count * resolution * resolution

    if known_free_area < min_free_area_m2:
        return False, "INSUFFICIENT_KNOWN_FREE_AREA"

    return True, "CLOSED_FREE_REGION_VERIFIED"


def verify_reachable_closed_free_region(
    data, width, height, resolution, seed_cell,
    free_threshold=25, min_free_area_m2=10.0,
):
    """Verify the known-free component connected to the robot.

    Unknown cells adjacent to the reachable component remain
    unresolved. Disconnected free-space artifacts do not count.
    This function never proves that unknown space is unreachable.
    """
    from collections import deque

    if (
        width < 3 or height < 3
        or len(data) != width * height
        or not math.isfinite(resolution)
        or resolution <= 0.0
        or not math.isfinite(min_free_area_m2)
        or min_free_area_m2 <= 0.0
        or not 0 <= free_threshold <= 100
    ):
        return False, "INVALID_MAP_GEOMETRY", {}

    if not isinstance(seed_cell, int):
        return False, "INVALID_ROBOT_SEED", {}

    if not 0 <= seed_cell < width * height:
        return False, "ROBOT_OUTSIDE_MAP", {}

    def free(index):
        return 0 <= data[index] <= free_threshold

    if not free(seed_cell):
        return False, "ROBOT_NOT_IN_KNOWN_FREE", {}

    visited = {seed_cell}
    queue = deque([seed_cell])
    unknown_neighbors = set()
    touches_edge = False

    while queue:
        index = queue.popleft()
        x, y = index % width, index // width

        if (
            x == 0 or x == width - 1
            or y == 0 or y == height - 1
        ):
            touches_edge = True

        neighbors = []

        if x > 0:
            neighbors.append(index - 1)
        if x + 1 < width:
            neighbors.append(index + 1)
        if y > 0:
            neighbors.append(index - width)
        if y + 1 < height:
            neighbors.append(index + width)

        for neighbor in neighbors:
            if data[neighbor] == -1:
                unknown_neighbors.add(neighbor)
            elif free(neighbor) and neighbor not in visited:
                visited.add(neighbor)
                queue.append(neighbor)

    area = len(visited) * resolution * resolution

    metrics = {
        "reachable_free_cells": len(visited),
        "reachable_free_area_m2": round(area, 4),
        "reachable_unknown_boundary_cells": len(unknown_neighbors),
        "reachable_free_touches_map_edge": touches_edge,
        "all_known_free_cells": sum(
            free(i) for i in range(width * height)
        ),
    }

    if touches_edge:
        return False, "REACHABLE_FREE_REACHES_MAP_EDGE", metrics

    if area < min_free_area_m2:
        return False, "INSUFFICIENT_REACHABLE_FREE_AREA", metrics

    if unknown_neighbors:
        return False, "REACHABLE_FREE_TOUCHES_UNKNOWN", metrics

    return True, "REACHABLE_CLOSED_FREE_REGION_VERIFIED", metrics
