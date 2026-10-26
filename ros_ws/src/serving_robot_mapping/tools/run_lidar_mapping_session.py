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



def _r16_212_stream_lines(stream):
    """Yield stdout lines and periodic idle ticks.

    This keeps the session supervisor responsive even when
    Explorer emits no additional log lines.
    """
    import codecs
    import os
    import select

    decoder = codecs.getincrementaldecoder("utf-8")(
        errors="replace"
    )
    fd = stream.fileno()
    pending = ""

    while True:
        ready, _, _ = select.select([fd], [], [], 0.5)

        if not ready:
            yield None
            continue

        chunk = os.read(fd, 32768)

        if not chunk:
            pending += decoder.decode(b"", final=True)
            if pending:
                yield pending
            return

        pending += decoder.decode(chunk)

        while "\n" in pending:
            line, pending = pending.split("\n", 1)
            yield line + "\n"


def run_explorer(out, max_seconds, resume_report=None):
    """Run bounded autonomous exploration with existing safety."""
    from time import monotonic as _hold_monotonic
    from serving_robot_mapping.mapping_obstacle_hold_watchdog import (
        ProlongedObstacleHoldWatch,
    )

    hold_watch = ProlongedObstacleHoldWatch(
        limit_s=120.0,
        min_samples=3,
    )
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

    if resume_report:
        args.extend([
            "-p",
            "mapping.resume_report_path:="
            + str(Path(resume_report).resolve()),
        ])

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
    _r16_212_blocked_since = None

    try:
        _r16_212_stream = _r16_212_stream_lines(process.stdout)
        with (
            out / "R16-147_exploration.log"
        ).open("w", buffering=1) as file:
            for line in _r16_212_stream:
                if line is None:
                    if (
                        _r16_212_blocked_since is not None
                        and _hold_monotonic()
                        - _r16_212_blocked_since >= 120.0
                    ):
                        outcome = "COVERAGE_LIMITED"
                        print(
                            "R16-212 COVERAGE_LIMITED: "
                            "no new safe viewpoint dispatched "
                            "without a new navigation goal; "
                            "preserving partial map for review. "
                            "MAP_COMPLETE not asserted.",
                            flush=True,
                        )
                        os.killpg(
                            process.pid, signal.SIGINT
                        )
                        break
                    continue

                file.write(line)

                if (
                    "MAPPING NAV2 GOAL SENT:" in line
                    or "New reachable Frontier candidate detected;"
                    in line
                    or "EXPLORATION_BLOCKED -> EXPLORING"
                    in line
                ):
                    _r16_212_blocked_since = None

                elif (
                    "EXPLORATION_BLOCKED:" in line
                    and "no Nav2 goal dispatched" in line
                    and (
                        "VISITED_VIEWPOINTS_ONLY" in line
                        or "NO_CLEARANCE_QUALIFIED_VIEWPOINT"
                        in line
                    )
                ):
                    if _r16_212_blocked_since is None:
                        _r16_212_blocked_since = (
                            _hold_monotonic()
                        )

                if events.search(line):
                    print(line.rstrip(), flush=True)

                event = terminal_event(line)

                # R16-183: terminate only the exploration
                # session after repeated, continuous safety
                # stop evidence. The robot stays stopped.
                if hold_watch.observe(
                    line, _hold_monotonic()
                ):
                    outcome = "SAFETY_HOLD_PROLONGED"
                    print(
                        "R16-183 SAFETY_HOLD_PROLONGED: "
                        f"obstacle hold observed for "
                        f"{hold_watch.elapsed_s:.1f}s; "
                        "stopping Explorer and saving raw map. "
                        "Safety policy unchanged.",
                        flush=True,
                    )
                    os.killpg(process.pid, signal.SIGINT)
                    break

                # R16-179: EXPLORATION_BLOCKED is a
                # reversible Explorer state. Do not stop
                # the process before map/frontier updates
                # have a chance to recover exploration.
                #
                # This does not relax any navigation or
                # collision safety requirement.
                if event == "COVERAGE_LIMITED":
                    print(
                        "R16-179: EXPLORATION_BLOCKED is "
                        "provisional; Explorer remains active.",
                        flush=True,
                    )
                    continue

                if event:
                    outcome = event
                    os.killpg(process.pid, signal.SIGINT)
                    break

            for line in _r16_212_stream:
                if line is None:
                    continue
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


R16_206_FAILURE_OUTCOMES = frozenset((
    "NAV2_CANCEL_UNCONFIRMED",
    "EXPLORER_SHUTDOWN_UNVERIFIED",
    "EXPLORER_INNER_FAILURE",
    "EXPLORER_LOG_UNAVAILABLE",
))


def reconcile_explorer_shutdown(outcome, log_path):
    """Preserve the original outcome and detect explicit failures."""
    import re
    from pathlib import Path

    evidence = {
        "explorer_runner_outcome": outcome,
        "explorer_log_evidence_available": False,
        "nav2_cancel_unconfirmed_observed": False,
        "explorer_shutdown_unverified_observed": False,
        "explorer_internal_failure_observed": False,
        "nav2_shutdown_status_observed": None,
        "timeout_may_mask_inner_exit_code": (
            outcome == "TIME_LIMIT_REACHED"
        ),
        "nav2_action_result_independently_verified": False,
    }

    try:
        with Path(log_path).open(
            encoding="utf-8",
            errors="replace",
        ) as stream:
            evidence["explorer_log_evidence_available"] = True

            for line in stream:
                if "Nav2 cancellation NOT confirmed" in line:
                    evidence[
                        "nav2_cancel_unconfirmed_observed"
                    ] = True

                match = re.search(
                    r"R16-203 NAVIGATION_SHUTDOWN_STATUS=\s*"
                    r"([A-Z][A-Z0-9_]*)\b",
                    line,
                )

                if match:
                    state = match.group(1)
                    evidence[
                        "nav2_shutdown_status_observed"
                    ] = state

                    if state in (
                        "CANCEL_UNCONFIRMED",
                        "CONTEXT_INVALID_CANCEL_UNCONFIRMED",
                        "CANCEL_REQUEST_EXCEPTION",
                    ):
                        evidence[
                            "nav2_cancel_unconfirmed_observed"
                        ] = True

                if "R16-203 EXPLORER_SHUTDOWN_UNVERIFIED" in line:
                    evidence[
                        "explorer_shutdown_unverified_observed"
                    ] = True

                if any(marker in line for marker in (
                    "[ros2run]: Process exited with failure",
                    "R16-203 EXPLORER_SPIN_EXCEPTION",
                    "R16-203 SHUTDOWN_EXCEPTION",
                    "TypeError: Unable to convert function return value",
                )):
                    evidence[
                        "explorer_internal_failure_observed"
                    ] = True

    except OSError:
        return "EXPLORER_LOG_UNAVAILABLE", evidence

    if evidence["nav2_cancel_unconfirmed_observed"]:
        return "NAV2_CANCEL_UNCONFIRMED", evidence

    if evidence["explorer_shutdown_unverified_observed"]:
        return "EXPLORER_SHUTDOWN_UNVERIFIED", evidence

    if evidence["explorer_internal_failure_observed"]:
        return "EXPLORER_INNER_FAILURE", evidence

    return outcome, evidence


def _r16_206_fail_close_report(report, outcome):
    """Never authorize a map after observed shutdown failure."""
    if outcome not in R16_206_FAILURE_OUTCOMES:
        return

    report.update({
        "map_complete": False,
        "auto_map_verified": False,
        "navigation_map_authorized": False,
        "final_map_approved": False,
        "map_draft_ready": False,
        "manual_review_required": True,
        "reason": "EXPLORER_SHUTDOWN_FAILURE",
    })


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
    modes.add_argument(
        "--handoff-existing", metavar="STATUS_JSON"
    )

    parser.add_argument(
        "--max-seconds", type=int, default=0
    )
    parser.add_argument("--resume-report", default="")
    parser.add_argument(
        "--log-dir",
        default="/mnt/c/dev/finalpj/mapping-run-logs",
    )

    options = parser.parse_args()

    if options.handoff_existing:
        from serving_robot_mapping.map_draft_policy import (
            write_map_handoff,
        )

        source = Path(options.handoff_existing).resolve()
        previous = json.loads(source.read_text(
            encoding="utf-8"
        ))

        if not isinstance(previous, dict):
            raise ValueError("Invalid session report")

        handoff = write_map_handoff(previous, source)

        print("HANDOFF_STATE:", handoff["handoff_state"])
        print("MAP_DRAFT_READY:", handoff["map_draft_ready"])
        print("FINAL_MAP_APPROVED:", False)
        print("NAVIGATION_MAP_AUTHORIZED:", False)
        print("SOURCE_SESSION_UNCHANGED:", source)
        return

    output = Path(options.log_dir)
    output.mkdir(parents=True, exist_ok=True)

    if options.run:
        outcome = run_explorer(
            output, options.max_seconds,
            options.resume_report,
        )
    else:
        previous = json.loads(
            Path(options.finalize_existing).read_text()
        )
        outcome = previous["explorer_terminal_status"]

    outcome, shutdown_evidence = reconcile_explorer_shutdown(
        outcome,
        output / "R16-147_exploration.log",
    )

    report = {
        "explorer_terminal_status": outcome,
        "map_complete": False,
        "raw_map_saved": False,
        "reason": "FINALIZATION_PENDING",
    }
    report.update(shutdown_evidence)

    if outcome == "COVERAGE_LIMITED":
        report["exploration_termination_class"] = (
            "NO_NEW_SAFE_GOAL_PENDING_REVIEW"
        )
        report["unknown_space_proven_unreachable"] = False

    # R16-182: retain the actual session outcome and
    # separately describe any prolonged safety hold
    # visible in the Explorer log.
    from serving_robot_mapping.mapping_session_safety_evidence import (
        summarize_session_safety,
    )
    report.update(summarize_session_safety(
        outcome,
        output / "R16-147_exploration.log",
    ))

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

    # R16-176: automatic exploration is separate from
    # final operator-reviewed map approval.
    from serving_robot_mapping.map_draft_policy import (
        write_map_handoff,
    )

    _r16_206_fail_close_report(report, outcome)
    handoff = write_map_handoff(report, destination)
    report.update(handoff)
    _r16_206_fail_close_report(report, outcome)
    save_report(destination, report)

    print("HANDOFF_STATE:", report["handoff_state"])
    print("MAP_DRAFT_READY:", report["map_draft_ready"])
    print("FINAL_MAP_APPROVED:", False)

    print("\\n========== FINAL SESSION RESULT ==========")
    print("OUTCOME:", outcome)
    print("MAP_COMPLETE:", report["map_complete"])
    print("OFFICIAL_MAP_SAVED:", report["raw_map_saved"])
    print("REPORT:", destination)

    if outcome in R16_206_FAILURE_OUTCOMES:
        print(
            "R16-206 SESSION_SHUTDOWN_FAILURE:",
            outcome,
            flush=True,
        )
        raise SystemExit(2)

    if not report["raw_map_saved"]:
        raise SystemExit(
            "STOP: Official map save failed. "
            "R16-146 checkpoints remain preserved."
        )


if __name__ == "__main__":
    main()
