"""R16-179: blocked exploration must not kill Explorer."""

import ast
from pathlib import Path


RUNNER = (
    Path(__file__).resolve().parents[1]
    / "tools"
    / "run_lidar_mapping_session.py"
)


def functions():
    tree = ast.parse(
        RUNNER.read_text(encoding="utf-8")
    )
    return {
        node.name: node
        for node in tree.body
        if isinstance(node, ast.FunctionDef)
    }


def terminal_event():
    fn = functions()["terminal_event"]
    module = ast.Module(
        body=[fn],
        type_ignores=[],
    )
    namespace = {}
    exec(compile(
        module, str(RUNNER), "exec"
    ), namespace)
    return namespace["terminal_event"]


def test_blocked_is_still_recognized():
    event = terminal_event()

    line = (
        "EXPLORATION_BLOCKED: VISITED_VIEWPOINTS_ONLY; "
        "no Nav2 goal dispatched. "
        "Waiting for new reachable Frontier. "
        "MAP_COMPLETE not asserted."
    )

    assert event(line) == "COVERAGE_LIMITED"


def test_blocked_without_viewpoint_is_recognized():
    event = terminal_event()

    line = (
        "EXPLORATION_BLOCKED: "
        "NO_CLEARANCE_QUALIFIED_VIEWPOINT; "
        "no Nav2 goal dispatched."
    )

    assert event(line) == "COVERAGE_LIMITED"


def test_map_complete_unchanged():
    event = terminal_event()
    assert event(
        "MAP_COMPLETE: strict verification succeeded"
    ) == "EXPLORER_REPORTED_MAP_COMPLETE"


def test_blocked_is_nonterminal_in_run_loop():
    runner = functions()["run_explorer"]

    guarded = []

    for node in ast.walk(runner):
        if not isinstance(node, ast.If):
            continue

        comparison = node.test

        if not (
            isinstance(comparison, ast.Compare)
            and isinstance(comparison.left, ast.Name)
            and comparison.left.id == "event"
            and len(comparison.ops) == 1
            and isinstance(comparison.ops[0], ast.Eq)
            and len(comparison.comparators) == 1
            and isinstance(
                comparison.comparators[0],
                ast.Constant,
            )
            and comparison.comparators[0].value
            == "COVERAGE_LIMITED"
        ):
            continue

        guarded.append(node)

    assert len(guarded) == 1

    # It must continue reading Explorer stdout,
    # not send SIGINT on a temporary blocked state.
    assert any(
        isinstance(node, ast.Continue)
        for node in ast.walk(guarded[0])
    )

    assert not any(
        isinstance(node, ast.Break)
        for node in ast.walk(guarded[0])
    )


def test_navigation_safety_stop_still_terminal():
    event = terminal_event()

    line = (
        "Mapping navigation result: CANCELED: "
        "Safety check invalid: "
        "OBSTACLE_HOLD_NOT_CLEAR"
    )

    assert event(line) is not None
    assert event(line) != "COVERAGE_LIMITED"


def test_explorer_recovery_code_retained():
    source = (
        RUNNER.parents[1]
        / "serving_robot_mapping"
        / "exploration_manager.py"
    ).read_text(encoding="utf-8")

    assert (
        "EXPLORATION_BLOCKED -> EXPLORING"
        in source
    )
    assert (
        "mapping_navigation_safety_check"
        in source
    )
