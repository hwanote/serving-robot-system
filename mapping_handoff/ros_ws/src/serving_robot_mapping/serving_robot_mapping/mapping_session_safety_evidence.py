"""Report observed safety holds without changing safety policy."""

import re
from pathlib import Path


STAMP = re.compile(r"\[(\d{10}\.\d+)\]")
HOLD = re.compile(r"\bobstacle_hold=(True|False)\b")


def summarize_session_safety(outcome, log_path):
    """Return evidence only, never a movement authorization."""
    result = {
        "last_observed_obstacle_hold": None,
        "obstacle_hold_diagnostic_samples": 0,
        "obstacle_hold_observed_span_s": 0.0,
        "obstacle_hold_unresolved_in_log": False,
        "timeout_with_unresolved_hold_evidence": False,
        "safety_evidence_is_not_live_clearance": True,
    }

    path = Path(log_path)
    if not path.is_file():
        return result

    # Do not allow an unexpectedly enormous log to stall
    # session finalization.
    if path.stat().st_size > 30_000_000:
        return result

    lines = path.read_text(
        encoding="utf-8", errors="replace"
    ).splitlines()

    # Only inspect the latest dispatched goal, not an
    # earlier safety incident followed by successful travel.
    start = 0
    for index, line in enumerate(lines):
        if "MAPPING NAV2 GOAL SENT:" in line:
            start = index

    sample_times = []
    last_hold = None
    canceled_for_hold = False

    for line in lines[start:]:
        if (
            "Mapping navigation result: CANCELED:"
            in line
            and "OBSTACLE_HOLD_NOT_CLEAR" in line
        ):
            canceled_for_hold = True

        if "Mapping safety diagnostic:" not in line:
            continue

        match = HOLD.search(line)
        if match is None:
            continue

        last_hold = match.group(1) == "True"

        if last_hold:
            stamp = STAMP.search(line)
            if stamp:
                sample_times.append(float(stamp.group(1)))

    span = (
        max(0.0, sample_times[-1] - sample_times[0])
        if len(sample_times) >= 2 else 0.0
    )

    # "Unresolved in log" deliberately means that no
    # later clear observation was recorded. It does NOT
    # assert what the sensor measured at process exit.
    unresolved = (
        canceled_for_hold
        and last_hold is True
        and span >= 30.0
    )

    result.update({
        "last_observed_obstacle_hold": last_hold,
        "obstacle_hold_diagnostic_samples": len(
            sample_times
        ),
        "obstacle_hold_observed_span_s": round(
            span, 1
        ),
        "obstacle_hold_unresolved_in_log": unresolved,
        "timeout_with_unresolved_hold_evidence": (
            outcome == "TIME_LIMIT_REACHED"
            and unresolved
        ),
    })

    return result
