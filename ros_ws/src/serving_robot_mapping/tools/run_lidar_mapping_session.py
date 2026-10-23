"""Repeatable LiDAR-only mapping session and honest map finalizer."""

import argparse
import json
import os
import re
import signal
import subprocess
import time
from pathlib import Path


def edge_audit(data, width, height):
    """Count exposed map edges and free-to-unknown contacts."""
    if (
        width < 3 or height < 3
        or len(data) != width * height
    ):
        raise ValueError("INVALID_MAP_GEOMETRY")

    def free(value):
        return 0 <= value <= 25

    sides = {
        "bottom": [(x, 0) for x in range(width)],
        "top": [(x, height - 1) for x in range(width)],
        "left": [(0, y) for y in range(height)],
        "right": [(width - 1, y) for y in range(height)],
    }

    edge_free = {}
    unique_edge = set()

    for name, points in sides.items():
        cells = [
            (x, y) for x, y in points
            if free(data[y * width + x])
        ]
        edge_free[name] = {
            "count": len(cells),
            "sample_cells": cells[:10],
        }
        unique_edge.update(cells)

    touching_unknown = set()
    free_count = 0

    for index, value in enumerate(data):
        if not free(value):
            continue

        free_count += 1
        x = index % width
        y = index // width

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
                touching_unknown.add(neighbor)

    return {
        "free_cells": free_count,
        "unknown_cells": sum(v == -1 for v in data),
        "occupied_cells": sum(v >= 65 for v in data),
        "free_edge": edge_free,
        "free_edge_total_unique": len(unique_edge),
        "free_unknown_boundary_cells": len(touching_unknown),
    }


def terminal_event(line):
    """Recognize only explicit Explorer terminal events."""
    # R16_171A_CONFIRMED_TRANSIENT_CANCEL
    # Preserve the Explorer's recovery policy for an explicitly
    # confirmed transient safety cancellation only.
    # All other unsafe cancellations remain terminal.
    if (
        "Mapping navigation result: CANCELED:" in line
        and "Safety check invalid:" in line
    ):
        detail = line.split(
            "Mapping navigation result: CANCELED:", 1
        )[1].strip()

        prefix = "Safety check invalid: "
        safety_code = (
            detail[len(prefix):].split(";", 1)[0].strip()
            if detail.startswith(prefix)
            else ""
        )

        recoverable_safety_codes = {
            "HEALTH_STALE",
            "HEALTH_SIM_STALE",
            "PERSON_LIMIT_STALE",
            "PERSON_LIMIT_NOT_NORMAL",
            "TF_NOT_STABLE",
            "OBSTACLE_HOLD_NOT_CLEAR",
            "MAP_ODOM_TF_STALE",
            "ODOM_BASE_TF_STALE",
            "SLAM_NOT_ACTIVE",
            "SCAN_STALE",
            "ODOM_STALE",
            "MAP_STALE",
        }

        confirmed_recoverable = (
            safety_code in recoverable_safety_codes
            and "terminal status=5" in detail
            and "cancel response=ACCEPTED" in detail
        )

        if confirmed_recoverable:
            # Keep the session alive. The Explorer still requires
            # continuous live safety evidence before replanning.
            return None

        return "SAFETY_HALTED_UNFINISHED"

    if (
        "Mapping navigation halted after " in line
        and "Manual restart required" in line
    ):
        return "NAVIGATION_HALTED_UNFINISHED"

    if (
        "MAP_COMPLETE:" in line
        and "not asserted" not in line.lower()
    ):
        return "EXPLORER_REPORTED_MAP_COMPLETE"

    if (
        "EXPLORATION_BLOCKED:" in line
        and "no Nav2 goal dispatched" in line
    ):
        return "COVERAGE_LIMITED"

    return None


def check_navigation():
    nodes = subprocess.check_output(
        ["ros2", "node", "list"], text=True, timeout=12
    ).splitlines()

    required = (
        "/collision_monitor",
        "/soft_stop_limiter",
        "/planner_server",
        "/controller_server",
        "/bt_navigator",
        "/robot_health",
    )

    forbidden = (
        "/person_detector",
        "/person_state",
        "/person_speed_limiter",
        "/exploration_manager",
    )

    for name in required:
        if name not in nodes:
            raise SystemExit(f"STOP: Missing {name}")

    for name in forbidden:
        if name in nodes:
            raise SystemExit(f"STOP: Unexpected {name}")

    result = subprocess.check_output(
        ["ros2", "lifecycle", "get", "/collision_monitor"],
        text=True,
        timeout=12,
    )

    if "active [3]" not in result:
        raise SystemExit("STOP: Collision monitor not active")


def run_explorer(out, max_seconds):
    """Run bounded autonomous exploration with existing safety."""
    check_navigation()

    args = [
        "timeout", "--signal=INT", "--kill-after=15s",
        f"{max_seconds}s",
        "ros2", "run",
        "serving_robot_mapping", "exploration_manager",
        "--ros-args",
        "-p", "use_sim_time:=true",
        "-p", "mapping.lidar_only:=true",
        "-p", "initial_scan.skip:=true",
        "-p", "planning.enabled:=true",
        "-p", "navigation.enabled:=true",
        "-p", "topics.frontier_goals:=/mapping/frontiers",
        "-p", "navigation.max_goals_per_run:=0",
        "-p", "navigation.timeout_s:=240.0",
        "-p", "navigation.recovery_stable_s:=3.0",
    ]

    events = re.compile(
        r"goal sent|navigation result|fallback|"
        r"EXPLORATION_BLOCKED|MAP_COMPLETE:|"
        r"state:|safety|error",
        re.I,
    )

    process = subprocess.Popen(
        args,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        text=True,
        bufsize=1,
        start_new_session=True,
        env={**os.environ, "PYTHONUNBUFFERED": "1"},
    )

    outcome = None

    try:
        with (
            out / "R16-147_exploration.log"
        ).open("w", buffering=1) as file:
            for line in process.stdout:
                file.write(line)

                if events.search(line):
                    print(line.rstrip(), flush=True)

                event = terminal_event(line)

                if event:
                    outcome = event
                    os.killpg(process.pid, signal.SIGINT)
                    break

            for line in process.stdout:
                file.write(line)

        code = process.wait(timeout=20)

        if outcome is None:
            outcome = (
                "TIME_LIMIT_REACHED"
                if code in (124, 137)
                else "EXPLORER_EXITED"
            )

    except KeyboardInterrupt:
        outcome = "OPERATOR_INTERRUPTED"

    finally:
        if process.poll() is None:
            os.killpg(process.pid, signal.SIGINT)
            try:
                process.wait(timeout=8)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid, signal.SIGKILL)

    return outcome


def get_map():
    """Read the actual /map OccupancyGrid, not a screen image."""
    import rclpy
    from nav_msgs.msg import OccupancyGrid
    from rclpy.qos import (
        QoSProfile,
        ReliabilityPolicy,
        DurabilityPolicy,
    )

    rclpy.init()
    node = rclpy.create_node("r16_147_map_finalizer")
    received = {"map": None}

    subscription = node.create_subscription(
        OccupancyGrid,
        "/map",
        lambda msg: received.__setitem__("map", msg),
        QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        ),
    )

    deadline = time.monotonic() + 25

    try:
        while (
            received["map"] is None
            and time.monotonic() < deadline
        ):
            rclpy.spin_once(node, timeout_sec=0.3)

        return received["map"]

    finally:
        node.destroy_node()
        rclpy.shutdown()


def official_save(prefix):
    """Save using the ROS2 Jazzy Nav2 MapSaver."""
    args = [
        "ros2", "run", "nav2_map_server", "map_saver_cli",
        "-t", "/map",
        "-f", str(prefix),
        "--fmt", "pgm",
        "--mode", "raw",
        "--free", "0.25",
        "--occ", "0.65",
        "--ros-args",
        "-p", "use_sim_time:=true",
        "-p", "save_map_timeout:=8.0",
    ]

    try:
        result = subprocess.run(
            args,
            text=True,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            timeout=35,
        )

        print("MapSaver output:")
        print(result.stdout[-1200:])

        return (
            result.returncode == 0
            and all(
                file.exists() and file.stat().st_size > 0
                for file in (
                    prefix.with_suffix(".yaml"),
                    prefix.with_suffix(".pgm"),
                )
            )
        )

    except (OSError, subprocess.TimeoutExpired) as error:
        print(f"MapSaver error: {error}")
        return False


def save_report(destination, report):
    """Persist a truthful, atomic session checkpoint for interrupted runs."""
    # R16_172_RELEASE_STABILITY
    temporary = destination.with_suffix(destination.suffix + ".tmp")
    temporary.write_text(
        json.dumps(report, indent=2, ensure_ascii=False) + "\n"
    )
    os.replace(temporary, destination)


def main():
    parser = argparse.ArgumentParser()

    modes = parser.add_mutually_exclusive_group(
        required=True
    )

    modes.add_argument(
        "--finalize-existing",
        metavar="STATUS_JSON",
    )
    modes.add_argument("--run", action="store_true")

    parser.add_argument(
        "--max-seconds", type=int, default=1200
    )
    parser.add_argument(
        "--log-dir",
        default="/mnt/c/dev/finalpj/mapping-run-logs",
    )

    options = parser.parse_args()
    output = Path(options.log_dir)
    output.mkdir(parents=True, exist_ok=True)

    if options.run:
        outcome = run_explorer(
            output, options.max_seconds
        )
    else:
        previous = json.loads(
            Path(options.finalize_existing).read_text()
        )
        outcome = previous["explorer_terminal_status"]

    report = {
        "explorer_terminal_status": outcome,
        "map_complete": False,
        "raw_map_saved": False,
        "reason": "FINALIZATION_PENDING",
    }
    destination = output / "R16-147_session_result.json"
    save_report(destination, report)

    message = get_map()

    if message is None:
        report["reason"] = "LIVE_MAP_UNAVAILABLE"

    else:
        report.pop("reason", None)
        width = int(message.info.width)
        height = int(message.info.height)
        resolution = float(message.info.resolution)
        data = tuple(message.data)

        statistics = edge_audit(
            data, width, height
        )

        from serving_robot_mapping.mapping_completion_verifier import (
            verify_closed_free_region,
        )

        verified, reason = verify_closed_free_region(
            data, width, height, resolution
        )

        report.update(statistics)
        report.update({
            "resolution_m": resolution,
            "known_free_m2": round(
                statistics["free_cells"]
                * resolution * resolution,
                4,
            ),
            "strict_verified": verified,
            "strict_check": reason,
            "map_dimensions": [width, height],
            "map_origin": [
                message.info.origin.position.x,
                message.info.origin.position.y,
            ],
        })

        prefix = output / "R16-147_official_raw_map"

        # If map saving is interrupted, keep an explicit unfinished
        # report instead of silently leaving no JSON at all.
        report["reason"] = "OFFICIAL_MAP_SAVE_PENDING"
        save_report(destination, report)
        saved = official_save(prefix)
        report.pop("reason", None)

        report["raw_map_saved"] = saved

        if saved:
            report["map_files"] = [
                str(prefix.with_suffix(".yaml")),
                str(prefix.with_suffix(".pgm")),
            ]

        report["map_complete"] = bool(
            saved
            and verified
            and outcome
            == "EXPLORER_REPORTED_MAP_COMPLETE"
        )

        print(
            "Free map-edge cells:",
            {
                side: value["count"]
                for side, value
                in statistics["free_edge"].items()
            },
        )
        print(
            "Unique free map-edge cells:",
            statistics["free_edge_total_unique"],
        )
        print(
            "Free-to-unknown boundary cells:",
            statistics["free_unknown_boundary_cells"],
        )
        print("Strict verification:", reason)

    save_report(destination, report)

    print("\n========== FINAL SESSION RESULT ==========")
    print("OUTCOME:", outcome)
    print("MAP_COMPLETE:", report["map_complete"])
    print("OFFICIAL_MAP_SAVED:", report["raw_map_saved"])
    print("REPORT:", destination)

    if not report["raw_map_saved"]:
        raise SystemExit(
            "STOP: Official map save failed. "
            "R16-146 checkpoints remain preserved."
        )


if __name__ == "__main__":
    main()
