"""Convert RViz POINTS frontiers to spatially distributed goals."""

import math
from collections import deque


def frontier_representatives(
    points,
    resolution,
    origin_x=0.0,
    origin_y=0.0,
    origin_yaw=0.0,
    limit=10,
):
    """Cluster adjacent frontier cells and sample each cluster.

    Results are candidate observation targets only.
    They still require Mapping clearance and Nav2 path checks.
    """
    if not (
        math.isfinite(resolution)
        and resolution > 0.0
        and limit > 0
    ):
        return []

    co = math.cos(origin_yaw)
    si = math.sin(origin_yaw)
    cells = {}

    for p in points:
        x = float(p.x)
        y = float(p.y)

        if not (math.isfinite(x) and math.isfinite(y)):
            continue

        dx = x - origin_x
        dy = y - origin_y

        mx = co * dx + si * dy
        my = -si * dx + co * dy

        col = math.floor(mx / resolution)
        row = math.floor(my / resolution)

        cells[(col, row)] = (x, y)

    if not cells:
        return []

    remaining = set(cells)
    clusters = []

    neighbors = [
        (dx, dy)
        for dy in (-1, 0, 1)
        for dx in (-1, 0, 1)
        if dx or dy
    ]

    while remaining:
        start = min(remaining)
        remaining.remove(start)
        queue = deque([start])
        cluster = []

        while queue:
            key = queue.popleft()
            cluster.append(key)

            for dx, dy in neighbors:
                neighbor = (key[0] + dx, key[1] + dy)

                if neighbor in remaining:
                    remaining.remove(neighbor)
                    queue.append(neighbor)

        clusters.append(cluster)

    clusters.sort(
        key=lambda cluster: (
            -len(cluster),
            min(cluster),
        )
    )

    groups = []

    for cluster in clusters:
        coordinates = [cells[key] for key in cluster]

        center_x = sum(x for x, _ in coordinates) / len(coordinates)
        center_y = sum(y for _, y in coordinates) / len(coordinates)

        central = min(
            coordinates,
            key=lambda xy: (
                (xy[0] - center_x) ** 2
                + (xy[1] - center_y) ** 2
            ),
        )

        farthest = max(
            coordinates,
            key=lambda xy: (
                (xy[0] - central[0]) ** 2
                + (xy[1] - central[1]) ** 2
            ),
        )

        representatives = [central]

        if math.hypot(
            farthest[0] - central[0],
            farthest[1] - central[1],
        ) >= resolution:
            representatives.append(farthest)

        groups.append(representatives)

    # First sample each cluster once; then sample alternatives.
    result = []

    for index in range(2):
        for group in groups:
            if index < len(group):
                result.append(group[index])

                if len(result) >= limit:
                    return result

    # Preserve existing fast sampling behavior.
    if limit <= 10:
        return result

    # Extended sampling explores additional cells WITHIN
    # each Frontier cluster, not just more clusters.
    extra_groups = []

    for cluster, representatives in zip(clusters, groups):
        chosen = {
            key for key in cluster
            if cells[key] in representatives
        }

        remaining = set(cluster) - chosen
        nearest_distance = {}

        for key in remaining:
            nearest_distance[key] = min(
                (key[0] - sample[0]) ** 2
                + (key[1] - sample[1]) ** 2
                for sample in chosen
            )

        extra_groups.append((remaining, nearest_distance))

    # Alternate between clusters to preserve spatial coverage.
    while len(result) < limit:
        progress = False

        for remaining, nearest_distance in extra_groups:
            if not remaining:
                continue

            best = max(
                remaining,
                key=lambda key: (
                    nearest_distance[key],
                    -key[1],
                    -key[0],
                ),
            )

            remaining.remove(best)
            result.append(cells[best])
            progress = True

            if len(result) >= limit:
                return result

            for key in remaining:
                distance = (
                    (key[0] - best[0]) ** 2
                    + (key[1] - best[1]) ** 2
                )

                if distance < nearest_distance[key]:
                    nearest_distance[key] = distance

        if not progress:
            break

    return result
