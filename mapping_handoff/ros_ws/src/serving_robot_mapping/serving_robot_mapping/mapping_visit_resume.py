"""Restore successful visits within a compatible live SLAM map."""

import json
import math
import re
from pathlib import Path

SENT = re.compile(
    r"MAPPING NAV2 GOAL SENT:\s*"
    r"\(\s*([-+]?\d+(?:\.\d+)?)\s*,"
    r"\s*([-+]?\d+(?:\.\d+)?)\s*\)"
)


def successful_visits(text):
    """Match each dispatched goal to its terminal result."""
    pending = None
    successful = []

    for line in text.splitlines():
        match = SENT.search(line)

        if match:
            pending = (
                float(match.group(1)),
                float(match.group(2)),
            )
            continue

        if "Mapping navigation result:" not in line:
            continue

        if (
            pending is not None
            and "Mapping navigation result: SUCCEEDED:"
            in line
        ):
            successful.append(pending)

        pending = None

    return successful[-200:]


def restore_visits(
    report_file,
    data,
    width,
    height,
    resolution,
    origin_x,
    origin_y,
    origin_yaw,
):
    """Fail closed if historical goals do not fit the live map.

    This is not proof of a common SLAM epoch. The caller must
    use it only when explicitly resuming the same live session.
    """
    path = Path(report_file).resolve()
    report = json.loads(path.read_text(encoding="utf-8"))

    if (
        report.get("raw_map_saved") is not True
        or report.get("navigation_map_authorized") is True
    ):
        raise ValueError("Invalid prior Mapping report")

    old_size = report.get("map_dimensions")
    old_origin = report.get("map_origin")
    old_resolution = report.get("resolution_m")

    if (
        old_size != [width, height]
        or not isinstance(old_origin, list)
        or len(old_origin) != 2
        or not isinstance(old_resolution, (int, float))
        or abs(float(old_resolution) - resolution) > 0.001
        or abs(float(old_origin[0]) - origin_x) > 0.02
        or abs(float(old_origin[1]) - origin_y) > 0.02
        or abs(origin_yaw) > 0.01
        or len(data) != width * height
    ):
        raise ValueError(
            "Live map geometry differs from previous Mapping"
        )

    log = path.parent / "R16-147_exploration.log"
    if not log.is_file():
        raise ValueError("Previous Explorer log unavailable")

    visits = successful_visits(
        log.read_text(encoding="utf-8", errors="replace")
    )

    if not visits:
        raise ValueError("No independently logged successful goals")

    validated = []

    for x, y in visits:
        if not (math.isfinite(x) and math.isfinite(y)):
            raise ValueError("Non-finite historical goal")

        col = math.floor((x - origin_x) / resolution)
        row = math.floor((y - origin_y) / resolution)

        if not (0 <= col < width and 0 <= row < height):
            raise ValueError("Historical goal outside current map")

        cell = data[row * width + col]

        if not 0 <= cell <= 25:
            raise ValueError(
                "Historical goal not known-free in current map"
            )

        validated.append((x, y))

    return validated
