"""Read-only Mapping safety diagnostic formatting."""

import math


def _fmt(value):
    if isinstance(value, bool):
        return "UNKNOWN"
    try:
        number = float(value)
    except (TypeError, ValueError):
        return "UNKNOWN"
    return (
        f"{number:.3f}"
        if math.isfinite(number)
        else "UNKNOWN"
    )


def format_mapping_safety_diagnostic(monitor, now, phase):
    """Report evidence; never authorize or change motion."""
    snap = getattr(monitor, "last_snapshot", None)
    result = getattr(monitor, "last_result", None)

    reason = (
        str(result[1])
        if isinstance(result, tuple) and len(result) > 1
        else "UNKNOWN"
    )

    def get(name):
        return getattr(snap, name, None)

    map_age = get("map_odom_tf_age_s")
    odom_age = get("odom_base_tf_age_s")

    last_progress = getattr(
        monitor, "_last_sim_progress", None
    )
    clock_age = (
        now - last_progress
        if last_progress is not None else None
    )

    tf_since = getattr(monitor, "tf_valid_since", None)
    stable_for = (
        now - tf_since
        if tf_since is not None else None
    )

    if clock_age is None or clock_age > 2.0:
        hint = "CLOCK_NOT_PROGRESSING"
    elif (
        isinstance(map_age, (int, float))
        and not isinstance(map_age, bool)
        and math.isfinite(map_age)
        and map_age > 0.50
    ):
        hint = "MAP_ODOM_TOO_OLD"
    elif (
        isinstance(odom_age, (int, float))
        and not isinstance(odom_age, bool)
        and math.isfinite(odom_age)
        and odom_age > 0.50
    ):
        hint = "ODOM_BASE_TOO_OLD"
    elif stable_for is not None and stable_for < 1.0:
        hint = "TF_STABILITY_WINDOW"
    else:
        hint = "CHECK_SAFETY_EVIDENCE"

    health = get("health")
    health = health if isinstance(health, dict) else {}
    checks = health.get("checks")
    checks = checks if isinstance(checks, dict) else {}

    fields = {
        "phase": phase,
        "reason": reason,
        "hint": hint,
        "map_odom_age_s": _fmt(map_age),
        "odom_base_age_s": _fmt(odom_age),
        "clock_progress_age_s": _fmt(clock_age),
        "sim_time_s": _fmt(getattr(
            monitor, "_last_sim_value", None
        )),
        "tf_valid_for_s": _fmt(stable_for),
        "tf_stable": repr(get("tf_stable")),
        "scan_age_s": _fmt(get("scan_age_s")),
        "odom_age_s": _fmt(get("odom_age_s")),
        "map_age_s": _fmt(get("map_age_s")),
        "slam_active": repr(get("slam_active")),
        "collision_latched": repr(
            get("collision_latched")
        ),
        "obstacle_hold": repr(get("obstacle_hold")),
        "health_state": str(
            health.get("safety_state", "UNKNOWN")
        ),
        "health_obstacle_hold": repr(
            checks.get("obstacle_hold")
        ),
        "health_collision_latched": repr(
            checks.get("collision_latched")
        ),
    }

    return "; ".join(
        f"{key}={value}" for key, value in fields.items()
    )
