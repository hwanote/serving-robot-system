"""Safe viewpoint fallback for exhausted Frontier exploration.

This only supplies geometric candidates. Existing Mapping
clearance, Nav2 path verification and live safety checks remain
responsible for navigation approval.
"""

import math

from .mapping_visibility_audit import boundary_visibility_audit


def select_visibility_fallback(
    grid,
    robot_xy,
    visited,
    goal_clear,
    max_results=8,
):
    if grid is None or len(grid) != 7:
        raise ValueError("Missing or invalid map grid")

    data, width, height, res, ox, oy, yaw = grid

    if (
        data is None
        or not isinstance(width, int)
        or not isinstance(height, int)
        or width < 3
        or height < 3
        or len(data) != width * height
        or not all(
            math.isfinite(value)
            for value in (res, ox, oy, yaw, *robot_xy)
        )
        or res <= 0
        or not isinstance(max_results, int)
        or max_results < 1
    ):
        raise ValueError("Invalid visibility fallback input")

    rx, ry = robot_xy

    c = math.cos(yaw)
    s = math.sin(yaw)

    dx = rx - ox
    dy = ry - oy

    gx = math.floor((c * dx + s * dy) / res)
    gy = math.floor((-s * dx + c * dy) / res)

    if not (0 <= gx < width and 0 <= gy < height):
        raise ValueError("Robot outside current map")

    seed = gy * width + gx

    selected = []
    guard_rejections = 0
    visited_rejections = 0
    guard_reasons = {}

    def to_world(cell_x, cell_y):
        return (
            ox + res * (
                c * (cell_x + 0.5)
                - s * (cell_y + 0.5)
            ),
            oy + res * (
                s * (cell_x + 0.5)
                + c * (cell_y + 0.5)
            ),
        )

    def allowed_viewpoint(cell_x, cell_y):
        nonlocal guard_rejections, visited_rejections

        wx, wy = to_world(cell_x, cell_y)

        if any(
            math.hypot(wx - vx, wy - vy) <= 0.25
            for vx, vy in visited
        ):
            visited_rejections += 1
            return False

        approved, reason = goal_clear(wx, wy)

        if not approved:
            guard_rejections += 1
            guard_reasons[reason] = (
                guard_reasons.get(reason, 0) + 1
            )
            return False

        return True

    analysis = boundary_visibility_audit(
        data,
        width,
        height,
        res,
        seed,
        sensor_range_m=5.0,
        clearance_m=0.30,
        stride_cells=4,
        max_candidates=32,
        viewpoint_filter=allowed_viewpoint,
    )

    # One final dense pass when the normal 4-cell sampling
    # cannot find a safe observation center.
    #
    # The same 0.52m Mapping guard is applied BEFORE
    # accepting candidates. Nav2 and live safety still gate
    # actual navigation. No clearance parameter is relaxed.
    viewpoint_stride_cells = 4

    if (
        not analysis["candidates"]
        and analysis["unknown_boundary_cells"] > 0
    ):
        viewpoint_stride_cells = 1

        # Report rejection counts for the decisive dense pass,
        # rather than double counting the sparse pass.
        guard_rejections = 0
        visited_rejections = 0
        guard_reasons.clear()

        analysis = boundary_visibility_audit(
            data,
            width,
            height,
            res,
            seed,
            sensor_range_m=5.0,
            clearance_m=0.30,
            stride_cells=1,
            max_candidates=32,
            viewpoint_filter=allowed_viewpoint,
        )

    # Recheck the selected candidates before forwarding.
    # This preserves fail-closed behavior even if an audit
    # implementation supplies an unchecked candidate.
    for gain, cell_x, cell_y in analysis["candidates"]:
        wx, wy = to_world(cell_x, cell_y)

        if any(
            math.hypot(wx - vx, wy - vy) <= 0.25
            for vx, vy in visited
        ):
            visited_rejections += 1
            continue

        approved, reason = goal_clear(wx, wy)

        if not approved:
            guard_rejections += 1
            guard_reasons[reason] = (
                guard_reasons.get(reason, 0) + 1
            )
            continue

        selected.append((wx, wy))

        if len(selected) >= max_results:
            break

    metrics = {
        "status": analysis["status"],
        "viewpoint_stride_cells": viewpoint_stride_cells,
        "unknown_boundary_cells": (
            analysis["unknown_boundary_cells"]
        ),
        "visible_boundary_cells": (
            analysis["visible_boundary_cells"]
        ),
        "sampled_viewpoints": analysis["sampled_viewpoints"],
        "geometry_candidates": len(analysis["candidates"]),
        "scored_viewpoints": analysis.get(
            "scored_viewpoints", 0
        ),
        "guard_rejection_reasons": dict(guard_reasons),
        "guard_rejections": guard_rejections,
        "visited_rejections": visited_rejections,
    }

    return selected, metrics
