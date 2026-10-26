"""R16-175 diagnostic tests; no robot movement."""

from types import SimpleNamespace

from serving_robot_mapping.mapping_safety_diagnostics import (
    format_mapping_safety_diagnostic,
)


def monitor():
    return SimpleNamespace(
        last_result=(False, "TF_NOT_STABLE"),
        last_snapshot=SimpleNamespace(
            map_odom_tf_age_s=0.62,
            odom_base_tf_age_s=0.04,
            tf_stable=False,
            slam_active=True,
            obstacle_hold=False,
            collision_latched=False,
            scan_age_s=0.1,
            odom_age_s=0.1,
            map_age_s=0.5,
            health={
                "safety_state": "NORMAL",
                "checks": {"obstacle_hold": False},
            },
        ),
        _last_sim_progress=99.9,
        _last_sim_value=12.0,
        tf_valid_since=None,
    )


def test_stale_tf_diagnostic():
    text = format_mapping_safety_diagnostic(
        monitor(), 100.0, "NAVIGATING"
    )
    assert "hint=MAP_ODOM_TOO_OLD" in text
    assert "map_odom_age_s=0.620" in text


def test_obstacle_recovery_diagnostic():
    obj = monitor()
    obj.last_result = (
        False, "OBSTACLE_HOLD_NOT_CLEAR"
    )
    obj.last_snapshot.obstacle_hold = True
    text = format_mapping_safety_diagnostic(
        obj, 100.0, "RECOVERY"
    )
    assert "reason=OBSTACLE_HOLD_NOT_CLEAR" in text
    assert "obstacle_hold=True" in text


def test_missing_clock_and_tf():
    obj = SimpleNamespace(
        last_result=(False, "TF_NOT_STABLE"),
        last_snapshot=None,
        _last_sim_progress=None,
        tf_valid_since=None,
    )
    text = format_mapping_safety_diagnostic(
        obj, 100.0, "RECOVERY"
    )
    assert "hint=CLOCK_NOT_PROGRESSING" in text
    assert "map_odom_age_s=UNKNOWN" in text
