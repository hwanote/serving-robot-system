"""R16-206C Mapping runner regressions without ROS graph."""

import ast
from pathlib import Path

import pytest


RUNNER = (
    Path(__file__).resolve().parents[1]
    / "tools"
    / "run_lidar_mapping_session.py"
)


def load_policy():
    tree = ast.parse(RUNNER.read_text())

    selected = []

    for item in tree.body:
        if isinstance(item, ast.Assign):
            names = [
                target.id
                for target in item.targets
                if isinstance(target, ast.Name)
            ]
            if "R16_206_FAILURE_OUTCOMES" in names:
                selected.append(item)

        if isinstance(item, ast.FunctionDef) and item.name in (
            "reconcile_explorer_shutdown",
            "_r16_206_fail_close_report",
        ):
            selected.append(item)

    assert len(selected) == 3

    namespace = {}

    exec(
        compile(
            ast.Module(body=selected, type_ignores=[]),
            str(RUNNER),
            "exec",
        ),
        namespace,
    )

    return namespace


@pytest.mark.parametrize(
    ("content", "expected"),
    [
        (
            "Nav2 cancellation NOT confirmed.\n"
            "[ros2run]: Process exited with failure 1\n",
            "NAV2_CANCEL_UNCONFIRMED",
        ),
        (
            "R16-203 NAVIGATION_SHUTDOWN_STATUS= "
            "CANCEL_UNCONFIRMED\n",
            "NAV2_CANCEL_UNCONFIRMED",
        ),
        (
            "R16-203 NAVIGATION_SHUTDOWN_STATUS= "
            "CONTEXT_INVALID_CANCEL_UNCONFIRMED\n",
            "NAV2_CANCEL_UNCONFIRMED",
        ),
        (
            "R16-203 NAVIGATION_SHUTDOWN_STATUS= "
            "CANCEL_REQUEST_EXCEPTION\n",
            "NAV2_CANCEL_UNCONFIRMED",
        ),
        (
            "R16-203 EXPLORER_SHUTDOWN_UNVERIFIED\n",
            "EXPLORER_SHUTDOWN_UNVERIFIED",
        ),
        (
            "[ros2run]: Process exited with failure 1\n",
            "EXPLORER_INNER_FAILURE",
        ),
        (
            "R16-203 NAVIGATION_SHUTDOWN_STATUS= "
            "CLIENT_IDLE_TERMINAL_STATUS_NOT_INDEPENDENTLY_VERIFIED\n",
            "TIME_LIMIT_REACHED",
        ),
        (
            "State: EXPLORING\n",
            "TIME_LIMIT_REACHED",
        ),
    ],
)
def test_r16_206c_classification(tmp_path, content, expected):
    log = tmp_path / "explorer.log"
    log.write_text(content)

    result, evidence = load_policy()[
        "reconcile_explorer_shutdown"
    ]("TIME_LIMIT_REACHED", log)

    assert result == expected
    assert evidence["explorer_log_evidence_available"]
    assert evidence["explorer_runner_outcome"] == (
        "TIME_LIMIT_REACHED"
    )
    assert not evidence[
        "nav2_action_result_independently_verified"
    ]


def test_r16_206c_exact_status_parsing(tmp_path):
    log = tmp_path / "explorer.log"

    log.write_text(
        "R16-203 NAVIGATION_SHUTDOWN_STATUS= "
        "CANCEL_UNCONFIRMED\n"
        "R16-203 EXPLORER_SHUTDOWN_UNVERIFIED\n"
    )

    result, evidence = load_policy()[
        "reconcile_explorer_shutdown"
    ]("TIME_LIMIT_REACHED", log)

    assert result == "NAV2_CANCEL_UNCONFIRMED"
    assert evidence["nav2_shutdown_status_observed"] == (
        "CANCEL_UNCONFIRMED"
    )


def test_r16_206c_missing_log_fails_closed(tmp_path):
    result, evidence = load_policy()[
        "reconcile_explorer_shutdown"
    ]("TIME_LIMIT_REACHED", tmp_path / "missing.log")

    assert result == "EXPLORER_LOG_UNAVAILABLE"
    assert not evidence["explorer_log_evidence_available"]


def test_r16_206c_fail_closed_fields():
    policy = load_policy()
    report = {
        "map_complete": True,
        "auto_map_verified": True,
        "navigation_map_authorized": True,
        "final_map_approved": True,
        "map_draft_ready": True,
    }

    policy["_r16_206_fail_close_report"](
        report, "NAV2_CANCEL_UNCONFIRMED"
    )

    for key in (
        "map_complete",
        "auto_map_verified",
        "navigation_map_authorized",
        "final_map_approved",
        "map_draft_ready",
    ):
        assert report[key] is False

    assert report["manual_review_required"] is True


def test_r16_206c_actual_call_count():
    tree = ast.parse(RUNNER.read_text())

    main = next(
        item for item in tree.body
        if isinstance(item, ast.FunctionDef)
        and item.name == "main"
    )

    calls = [
        item
        for item in ast.walk(main)
        if isinstance(item, ast.Call)
        and isinstance(item.func, ast.Name)
        and item.func.id == "_r16_206_fail_close_report"
    ]

    # Count only actual calls inside main().
    # Function definition must not be counted.
    assert len(calls) == 2

    text = RUNNER.read_text()

    assert (
        "    report.update(handoff)\n"
        "    _r16_206_fail_close_report(report, outcome)\n"
        "    save_report(destination, report)"
    ) in text

    assert "raise SystemExit(2)" in text
