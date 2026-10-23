"""Regression tests for acknowledged mapping safety cancellations."""

import ast
from pathlib import Path

import pytest

import serving_robot_mapping.exploration_manager as manager_module


def classify_using_production_branch(outcome, reason):
    """Execute the actual Explorer recovery classification statements."""

    path = Path(manager_module.__file__)
    tree = ast.parse(path.read_text())

    manager_class = next(
        node for node in tree.body
        if isinstance(node, ast.ClassDef)
        and node.name == "ExplorationManager"
    )

    def assigned(statement, name):
        return (
            isinstance(statement, ast.Assign)
            and any(
                isinstance(target, ast.Name)
                and target.id == name
                for target in statement.targets
            )
        )

    selected = None

    for method in manager_class.body:
        if not isinstance(method, ast.FunctionDef):
            continue

        starts = [
            i for i, statement in enumerate(method.body)
            if assigned(statement, "transient_reasons")
        ]

        ends = [
            i for i, statement in enumerate(method.body)
            if assigned(statement, "recoverable_cancel")
        ]

        if len(starts) == 1 and len(ends) == 1:
            start, end = starts[0], ends[0]
            assert start < end
            selected = method.body[start:end + 1]
            break

    assert selected is not None, (
        "Production recovery classification branch not found"
    )

    block = ast.fix_missing_locations(
        ast.Module(body=selected, type_ignores=[])
    )

    namespace = {
        "outcome": outcome,
        "reason": reason,
    }

    exec(compile(block, str(path), "exec"), namespace)

    return namespace["recoverable_cancel"]


@pytest.mark.parametrize(
    "outcome,reason,expected",
    [
        (
            "CANCELED",
            "Safety check invalid: PERSON_LIMIT_NOT_NORMAL; "
            "terminal status=5; cancel response=ACCEPTED",
            True,
        ),
        (
            "CANCELED",
            "Safety check invalid: PERSON_LIMIT_STALE; "
            "terminal status=5; cancel response=ACCEPTED",
            True,
        ),
        (
            "CANCELED",
            "Safety check invalid: PERSON_LIMIT_NOT_NORMAL; "
            "terminal status=5; cancel response=REJECTED",
            False,
        ),
        (
            "CANCELED",
            "Safety check invalid: PERSON_LIMIT_NOT_NORMAL; "
            "terminal status=6; cancel response=ACCEPTED",
            False,
        ),
        (
            "FAILED",
            "Safety check invalid: PERSON_LIMIT_NOT_NORMAL; "
            "terminal status=5; cancel response=ACCEPTED",
            False,
        ),
        (
            "CANCELED",
            "Safety check invalid: COLLISION_STATE_NOT_CLEAR; "
            "terminal status=5; cancel response=ACCEPTED",
            False,
        ),
        (
            "CANCELED",
            "Safety check invalid: PERSON_DETECTOR_SILENT; "
            "terminal status=5; cancel response=ACCEPTED",
            False,
        ),
    ],
)
def test_safety_cancel_classification(outcome, reason, expected):
    assert classify_using_production_branch(outcome, reason) is expected


def test_recovery_loop_still_requires_safe_monitor():
    """The recovery loop must retain its independent safety gate."""

    path = Path(manager_module.__file__)
    tree = ast.parse(path.read_text())

    manager_class = next(
        node for node in tree.body
        if isinstance(node, ast.ClassDef)
        and node.name == "ExplorationManager"
    )

    navigation_loop = next(
        node for node in manager_class.body
        if isinstance(node, ast.FunctionDef)
        and node.name == "navigation_loop"
    )

    safety_calls = [
        node for node in ast.walk(navigation_loop)
        if isinstance(node, ast.Call)
        and isinstance(node.func, ast.Attribute)
        and node.func.attr == "is_safe"
    ]

    assert safety_calls, (
        "Recovery loop must independently check current safety"
    )
