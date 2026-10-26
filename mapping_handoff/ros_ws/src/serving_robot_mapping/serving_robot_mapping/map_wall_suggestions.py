"""Conservative geometric wall-gap suggestions.

Suggestions never modify an occupancy map or authorize navigation.
"""

import math


DIRECTIONS = (
    (1, 0),
    (0, 1),
    (1, 1),
    (1, -1),
)


def find_wall_gaps(
    pixels,
    width,
    height,
    resolution,
    max_gap_m=0.30,
    max_suggestions=12,
):
    """Suggest short UNKNOWN gaps between aligned measured walls."""
    if (
        len(pixels) != width * height
        or width < 3
        or height < 3
        or not math.isfinite(resolution)
        or resolution <= 0
    ):
        raise ValueError("Invalid occupancy grid")

    if (
        not math.isfinite(max_gap_m)
        or max_gap_m <= 0
        or max_suggestions < 1
    ):
        raise ValueError("Invalid suggestion limits")

    def cell(x, y):
        if 0 <= x < width and 0 <= y < height:
            return pixels[y * width + x]
        return None

    def wall(x, y):
        value = cell(x, y)
        return value is not None and 65 <= value <= 100

    def unknown(x, y):
        value = cell(x, y)
        return value is not None and value >= 101

    def free(x, y):
        value = cell(x, y)
        return value is not None and 0 <= value <= 25

    proposed = []

    for y in range(height):
        for x in range(width):
            if not wall(x, y):
                continue

            for dx, dy in DIRECTIONS:
                step_m = resolution * math.hypot(dx, dy)
                maximum = int(
                    (max_gap_m + 1e-9) / step_m
                )

                # Require at least two measured wall cells
                # behind the first endpoint.
                if not all(
                    wall(x - k * dx, y - k * dy)
                    for k in (1, 2)
                ):
                    continue

                for count in range(1, maximum + 1):
                    ex = x + (count + 1) * dx
                    ey = y + (count + 1) * dy

                    # Both ends must continue as measured walls.
                    if not all(
                        wall(ex + k * dx, ey + k * dy)
                        for k in (0, 1, 2)
                    ):
                        continue

                    interior = [
                        (x + k * dx, y + k * dy)
                        for k in range(1, count + 1)
                    ]

                    # Never infer a wall across known free space
                    # or ambiguous non-UNKNOWN cells.
                    if not all(
                        unknown(px, py)
                        for px, py in interior
                    ):
                        continue

                    # A visible passage with free space on
                    # both transverse sides is not a safe
                    # wall-connection suggestion.
                    nx, ny = -dy, dx

                    if any(
                        free(px + nx, py + ny)
                        and free(px - nx, py - ny)
                        for px, py in interior
                    ):
                        continue

                    indices = frozenset(
                        py * width + px
                        for px, py in interior
                    )

                    proposed.append((
                        len(interior),
                        y,
                        x,
                        (x, y, ex, ey),
                        indices,
                        round(count * step_m, 3),
                    ))

                    break

    # Prefer the smallest gaps, and avoid duplicate or
    # overlapping proposals on the same unknown cells.
    proposed.sort(key=lambda item: item[:3])
    used_cells = set()
    result = []

    for _, _, _, coordinates, indices, gap_m in proposed:
        if not used_cells.isdisjoint(indices):
            continue

        used_cells.update(indices)
        x1, y1, x2, y2 = coordinates

        result.append({
            "id": f"wall-gap-{len(result) + 1:03d}",
            "type": "wall_segment",
            "x1": x1,
            "y1": y1,
            "x2": x2,
            "y2": y2,
            "gap_m": gap_m,
            "basis": "measured_wall_anchors",
            "status": "PROPOSED",
            "review_required": True,
        })

        if len(result) >= max_suggestions:
            break

    return result
