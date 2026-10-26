"""R16-183 prolonged Mapping obstacle-hold tests."""

from pathlib import Path

from serving_robot_mapping.mapping_obstacle_hold_watchdog import (
    ProlongedObstacleHoldWatch,
)


def diagnostic(hold=True, reason=None):
    if reason is None:
        reason = (
            "OBSTACLE_HOLD_NOT_CLEAR"
            if hold else "TF_NOT_STABLE"
        )
    return (
        "Mapping safety diagnostic: phase=RECOVERY; "
        f"reason={reason}; "
        f"obstacle_hold={hold}; "
        "tf_stable=True"
    )


def test_repeated_hold_requires_full_timeout():
    watcher = ProlongedObstacleHoldWatch()
    assert not watcher.observe(diagnostic(), 10.0)
    assert not watcher.observe(diagnostic(), 20.0)
    assert not watcher.observe(diagnostic(), 30.0)
    assert not watcher.observe(diagnostic(), 125.0)


def test_continuous_hold_triggers_after_120_seconds():
    watcher = ProlongedObstacleHoldWatch()

    for now in (10, 20, 40, 60, 80, 100, 120):
        assert not watcher.observe(diagnostic(), now)

    assert watcher.observe(diagnostic(), 130.0)
    assert watcher.elapsed_s == 120.0


def test_clear_observation_resets_timer():
    watcher = ProlongedObstacleHoldWatch()
    assert not watcher.observe(diagnostic(), 0.0)
    assert not watcher.observe(diagnostic(), 20.0)
    assert not watcher.observe(
        diagnostic(hold=False), 21.0
    )
    assert not watcher.observe(diagnostic(), 130.0)
    assert watcher.elapsed_s == 0.0


def test_new_goal_resets_old_hold():
    watcher = ProlongedObstacleHoldWatch()

    for now in (10, 20, 30):
        assert not watcher.observe(diagnostic(), now)

    assert not watcher.observe(
        "MAPPING NAV2 GOAL SENT: (1.0, 2.0)",
        100.0,
    )
    assert not watcher.observe(diagnostic(), 140.0)
    assert watcher.elapsed_s == 0.0


def test_missing_diagnostics_does_not_trigger():
    watcher = ProlongedObstacleHoldWatch()

    assert not watcher.observe(diagnostic(), 10.0)
    assert not watcher.observe(diagnostic(), 20.0)
    assert not watcher.observe(diagnostic(), 200.0)
    assert watcher.elapsed_s == 0.0


def test_unrelated_recovery_reason_resets():
    watcher = ProlongedObstacleHoldWatch()

    assert not watcher.observe(diagnostic(), 10.0)
    assert not watcher.observe(diagnostic(), 20.0)

    other = diagnostic(
        hold=True,
        reason="TF_NOT_STABLE",
    )
    assert not watcher.observe(other, 25.0)
    assert not watcher.observe(diagnostic(), 30.0)
    assert watcher.elapsed_s == 0.0


def test_navigation_phase_does_not_count():
    watcher = ProlongedObstacleHoldWatch()

    line = diagnostic().replace(
        "phase=RECOVERY",
        "phase=NAVIGATING",
    )

    assert not watcher.observe(line, 10.0)
    assert not watcher.observe(line, 150.0)


def test_runner_contains_guard_and_status():
    path = (
        Path(__file__).resolve().parents[1]
        / "tools/run_lidar_mapping_session.py"
    )
    source = path.read_text(encoding="utf-8")

    assert "ProlongedObstacleHoldWatch" in source
    assert 'outcome = "SAFETY_HOLD_PROLONGED"' in source
    assert "hold_watch.observe(" in source
    assert 'if event == "COVERAGE_LIMITED":' in source
