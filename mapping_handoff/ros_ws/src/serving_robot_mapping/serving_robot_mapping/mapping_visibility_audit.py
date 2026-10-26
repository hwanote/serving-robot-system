"""Independent geometric audit for unexplored map boundaries.

Candidate locations are not navigation approvals. Nav2 and
existing Mapping safety gates must still validate movement.
"""

import math
from collections import deque


def boundary_visibility_audit(
    data, width, height, resolution, seed_cell,
    sensor_range_m=5.0, clearance_m=0.30, stride_cells=4,
    max_candidates=8,
    viewpoint_filter=None,
):
    size = width * height

    if (
        width < 3 or height < 3
        or len(data) != size
        or not isinstance(seed_cell, int)
        or not 0 <= seed_cell < size
        or not math.isfinite(resolution)
        or resolution <= 0
        or not math.isfinite(sensor_range_m)
        or sensor_range_m <= 0
        or not math.isfinite(clearance_m)
        or clearance_m <= 0
        or stride_cells < 1
        or not isinstance(max_candidates, int)
        or max_candidates < 1
        or (
            viewpoint_filter is not None
            and not callable(viewpoint_filter)
        )
    ):
        raise ValueError("Invalid visibility audit input")

    free = [0 <= value <= 25 for value in data]

    if not free[seed_cell]:
        raise ValueError("Robot seed is not known free")

    def neighbors(i):
        x, y = i % width, i // width

        if x > 0:
            yield i - 1
        if x + 1 < width:
            yield i + 1
        if y > 0:
            yield i - width
        if y + 1 < height:
            yield i + width

    # First identify the connected known-free region.
    reachable = {seed_cell}
    queue = deque([seed_cell])

    while queue:
        index = queue.popleft()

        for other in neighbors(index):
            if free[other] and other not in reachable:
                reachable.add(other)
                queue.append(other)

    unknown_boundary = {
        other
        for index in reachable
        for other in neighbors(index)
        if data[other] == -1
    }

    if not unknown_boundary:
        return {
            "reachable_free_cells": len(reachable),
            "unknown_boundary_cells": 0,
            "clearance_safe_cells": 0,
            "visible_boundary_cells": 0,
            "sampled_viewpoints": 0,
            "candidates": [],
            "status": "NO_REACHABLE_UNKNOWN_BOUNDARY",
        }

    # Erode free space by a conservative 0.30m radius.
    # This is geometry screening, NOT a Nav2 collision check.
    radius_cells = math.ceil(clearance_m / resolution)

    offsets = [
        (dx, dy)
        for dy in range(-radius_cells, radius_cells + 1)
        for dx in range(-radius_cells, radius_cells + 1)
        if math.hypot(dx, dy) * resolution
        <= clearance_m + 1e-9
    ]

    safe = set()

    for index in reachable:
        x, y = index % width, index // width

        if all(
            0 <= x + dx < width
            and 0 <= y + dy < height
            and free[(y + dy) * width + x + dx]
            for dx, dy in offsets
        ):
            safe.add(index)

    sx, sy = seed_cell % width, seed_cell // width

    possible_seeds = [
        index for index in safe
        if math.hypot(
            index % width - sx,
            index // width - sy,
        ) * resolution <= 0.80
    ]

    if not possible_seeds:
        return {
            "reachable_free_cells": len(reachable),
            "unknown_boundary_cells": len(unknown_boundary),
            "clearance_safe_cells": len(safe),
            "visible_boundary_cells": 0,
            "sampled_viewpoints": 0,
            "candidates": [],
            "status": "NO_SAFE_SEED_NEAR_ROBOT",
        }

    start = min(
        possible_seeds,
        key=lambda i: (
            (i % width - sx) ** 2
            + (i // width - sy) ** 2
        ),
    )

    # Only sample the safe component connected to the robot.
    safe_reachable = {start}
    queue = deque([start])

    while queue:
        index = queue.popleft()

        for other in neighbors(index):
            if other in safe and other not in safe_reachable:
                safe_reachable.add(other)
                queue.append(other)

    def visible(x0, y0, x1, y1):
        """Raster LOS through known-free cells to unknown endpoint."""
        dx = abs(x1 - x0)
        dy = -abs(y1 - y0)
        step_x = 1 if x0 < x1 else -1
        step_y = 1 if y0 < y1 else -1
        error = dx + dy

        x, y = x0, y0

        while True:
            if x == x1 and y == y1:
                return True

            if not free[y * width + x]:
                return False

            twice = 2 * error

            if twice >= dy:
                error += dy
                x += step_x

            if twice <= dx:
                error += dx
                y += step_y

    max_range_sq = (sensor_range_m / resolution) ** 2
    targets = [
        (i % width, i // width)
        for i in sorted(unknown_boundary)
    ]

    scored = []
    seen_targets = set()
    sampled = 0

    for index in sorted(safe_reachable):
        x = index % width
        y = index // width

        if x % stride_cells or y % stride_cells:
            continue

        sampled += 1
        visible_here = []

        for tx, ty in targets:
            if (x - tx) ** 2 + (y - ty) ** 2 > max_range_sq:
                continue

            if visible(x, y, tx, ty):
                visible_here.append((tx, ty))

        if visible_here:
            scored.append((len(visible_here), x, y))
            seen_targets.update(visible_here)

    scored.sort(key=lambda item: (-item[0], item[1], item[2]))

    # Keep spatially distinct candidate observation centers.
    selected = []
    separation_sq = (0.75 / resolution) ** 2

    for gain, x, y in scored:
        if (x - sx) ** 2 + (y - sy) ** 2 < (
            0.40 / resolution
        ) ** 2:
            continue

        if any(
            (x - px) ** 2 + (y - py) ** 2 < separation_sq
            for _, px, py in selected
        ):
            continue

        # Reject unsuitable viewpoints BEFORE filling
        # the candidate budget.
        if (
            viewpoint_filter is not None
            and not viewpoint_filter(x, y)
        ):
            continue

        selected.append((gain, x, y))

        if len(selected) >= max_candidates:
            break

    return {
        "reachable_free_cells": len(reachable),
        "unknown_boundary_cells": len(unknown_boundary),
        "clearance_safe_cells": len(safe_reachable),
        "visible_boundary_cells": len(seen_targets),
        "sampled_viewpoints": sampled,
        "scored_viewpoints": len(scored),
        "candidates": selected,
        "status": (
            "VISIBLE_OBSERVATION_LOCATIONS_EXIST"
            if selected
            else "NO_VISIBLE_SAFE_OBSERVATION_LOCATION"
        ),
    }
