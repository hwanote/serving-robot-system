"""R16-177: fail-closed geometric map release preflight.

This module cannot authorize a navigation map.
"""

import math
from collections import deque


CLEARANCE_M = 0.48


class MapReleaseError(ValueError):
    """Invalid, unsafe, or inconsistent reviewed map."""


def line_cells(x1, y1, x2, y2):
    """Bresenham pixels in PGM top-left coordinates."""
    dx = abs(x2 - x1)
    dy = abs(y2 - y1)
    sx = 1 if x1 < x2 else -1
    sy = 1 if y1 < y2 else -1
    err = dx - dy

    while True:
        yield x1, y1
        if x1 == x2 and y1 == y2:
            break
        twice = 2 * err
        if twice > -dy:
            err -= dy
            x1 += sx
        if twice < dx:
            err += dx
            y1 += sy


def geometric_validation(
    width, height, raw, edited, resolution, segments
):
    """Check edit provenance and estimate safe robot centers."""
    if (
        width < 3 or height < 3
        or len(raw) != width * height
        or len(edited) != width * height
        or not math.isfinite(resolution)
        or resolution <= 0 or resolution > 0.3
    ):
        raise MapReleaseError("Invalid map geometry")

    if not isinstance(segments, list) or not (
        1 <= len(segments) <= 20
    ):
        raise MapReleaseError("Invalid segment count")

    allowed_cells = set()

    for segment in segments:
        coords = [
            segment.get(name)
            for name in ("x1", "y1", "x2", "y2")
        ]

        if any(type(v) is not int for v in coords):
            raise MapReleaseError("Non-integer segment")

        x1, y1, x2, y2 = coords

        if not (
            0 <= x1 < width and 0 <= x2 < width
            and 0 <= y1 < height and 0 <= y2 < height
        ):
            raise MapReleaseError("Segment outside map")

        for x, y in ((x1, y1), (x2, y2)):
            value = raw[y * width + x]
            if not 65 <= value <= 100:
                raise MapReleaseError(
                    "Segment endpoint is not observed wall"
                )

        for x, y in line_cells(*coords):
            index = y * width + x
            value = raw[index]

            if not (
                65 <= value <= 100 or value >= 101
            ):
                raise MapReleaseError(
                    "Segment crosses free or uncertain space"
                )
            allowed_cells.add(index)

    changed = {
        i for i, (before, after) in enumerate(
            zip(raw, edited)
        )
        if before != after
    }

    expected = {
        i for i in allowed_cells
        if raw[i] >= 101
    }

    if changed != expected:
        raise MapReleaseError(
            "Map edits differ from approved segment pixels"
        )

    if not all(
        raw[i] >= 101 and edited[i] == 100
        for i in changed
    ):
        raise MapReleaseError(
            "Changes other than UNKNOWN to WALL detected"
        )

    def free(x, y):
        return edited[y * width + x] <= 25

    free_cells = {
        y * width + x
        for y in range(height)
        for x in range(width)
        if free(x, y)
    }

    free_edge = {
        i for i in free_cells
        if i // width in (0, height - 1)
        or i % width in (0, width - 1)
    }

    unknown_boundary = set()

    for i in free_cells:
        x = i % width
        y = i // width

        for dx, dy in (
            (-1, 0), (1, 0),
            (0, -1), (0, 1)
        ):
            nx, ny = x + dx, y + dy
            if (
                0 <= nx < width
                and 0 <= ny < height
                and edited[ny * width + nx] >= 101
            ):
                unknown_boundary.add(i)
                break

    border_edits = sum(
        1 for i in changed
        if i // width in (0, height - 1)
        or i % width in (0, width - 1)
    )

    # Conservative clearance:
    # Unknown, uncertain, occupied, and outside-map
    # all block robot-center positions.
    # Account for half a grid cell's diagonal.
    radius_m = (
        CLEARANCE_M
        + resolution * math.sqrt(2) / 2
    )
    radius_cells = math.ceil(radius_m / resolution)

    offsets = [
        (dx, dy)
        for dy in range(-radius_cells, radius_cells + 1)
        for dx in range(-radius_cells, radius_cells + 1)
        if math.hypot(dx, dy) * resolution
        <= radius_m + 1e-9
    ]

    safe = set()

    for i in free_cells:
        x = i % width
        y = i // width

        if (
            x < radius_cells
            or y < radius_cells
            or x >= width - radius_cells
            or y >= height - radius_cells
        ):
            continue

        if all(
            0 <= x + dx < width
            and 0 <= y + dy < height
            and free(x + dx, y + dy)
            for dx, dy in offsets
        ):
            safe.add(i)

    # Connectivity is a diagnostic, not proof
    # that a Nav2 route exists.
    unvisited = set(safe)
    component_sizes = []

    while unvisited:
        seed = unvisited.pop()
        queue = deque([seed])
        count = 0

        while queue:
            i = queue.popleft()
            count += 1
            x = i % width
            y = i // width

            for dx, dy in (
                (-1, 0), (1, 0),
                (0, -1), (0, 1)
            ):
                nx, ny = x + dx, y + dy
                if not (
                    0 <= nx < width
                    and 0 <= ny < height
                ):
                    continue

                next_i = ny * width + nx
                if next_i in unvisited:
                    unvisited.remove(next_i)
                    queue.append(next_i)

        component_sizes.append(count)

    component_sizes.sort(reverse=True)

    return {
        "modified_unknown_to_wall_cells": len(changed),
        "known_free_cells_modified": 0,
        "edited_wall_cells_on_map_edge": border_edits,
        "free_map_edge_cells": len(free_edge),
        "free_unknown_boundary_cells": len(
            unknown_boundary
        ),
        "observed_free_cells": len(free_cells),
        "clearance_threshold_m": CLEARANCE_M,
        "unknown_treated_as_blocked": True,
        "clearance_safe_center_cells": len(safe),
        "clearance_safe_components": len(
            component_sizes
        ),
        "largest_safe_component_cells": (
            component_sizes[0]
            if component_sizes else 0
        ),
        "safe_center_count_is_not_nav2_path_proof": True,
    }


def release_preflight_decision(metrics):
    """No offline geometric check may authorize Nav2."""
    blockers = [
        "LIVE_NAV2_PATH_VALIDATION_MISSING",
        "TF_AND_COLLISION_SAFETY_EVIDENCE_MISSING",
        "OPERATOR_TOPOLOGY_APPROVAL_MISSING",
    ]

    if metrics["free_map_edge_cells"] > 0:
        blockers.append(
            "MAP_EXTENT_FREE_EDGE_REVIEW_REQUIRED"
        )

    if metrics["edited_wall_cells_on_map_edge"] > 0:
        blockers.append(
            "INFERRED_WALL_ON_MAP_BORDER_REVIEW_REQUIRED"
        )

    if metrics["clearance_safe_center_cells"] == 0:
        blockers.append(
            "NO_CLEARANCE_SAFE_ROBOT_CENTER_FOUND"
        )

    return {
        "format": "R16-177-map-release-preflight-v1",
        "geometry_integrity": "PASS",
        "map_release_state": "HOLD_FOR_VALIDATION",
        "map_metrics": metrics,
        "blocking_requirements": blockers,
        "unknown_boundary_requires_review": (
            metrics["free_unknown_boundary_cells"] > 0
        ),
        "final_map_approved": False,
        "navigation_map_authorized": False,
    }
