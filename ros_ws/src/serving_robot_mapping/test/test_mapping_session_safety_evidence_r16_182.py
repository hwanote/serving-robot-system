"""R16-182 session-safety evidence regression tests."""

from serving_robot_mapping.mapping_session_safety_evidence import (
    summarize_session_safety,
)


def sample(time, hold):
    return (
        f"[WARN] [{time:.6f}] [exploration_manager]: "
        "Mapping safety diagnostic: phase=RECOVERY; "
        f"obstacle_hold={hold}; tf_stable=True"
    )


def test_long_hold_at_timeout_is_visible(tmp_path):
    path = tmp_path / "exploration.log"
    path.write_text("\n".join([
        "MAPPING NAV2 GOAL SENT: (6.72, 3.72)",
        sample(1792814200.0, True),
        "Mapping navigation result: CANCELED: "
        "Safety check invalid: OBSTACLE_HOLD_NOT_CLEAR",
        sample(1792814770.0, True),
    ]))

    result = summarize_session_safety(
        "TIME_LIMIT_REACHED", path
    )
    assert result[
        "timeout_with_unresolved_hold_evidence"
    ] is True
    assert result[
        "obstacle_hold_observed_span_s"
    ] == 570.0


def test_later_clear_observation_not_unresolved(tmp_path):
    path = tmp_path / "exploration.log"
    path.write_text("\n".join([
        "MAPPING NAV2 GOAL SENT:",
        sample(1792814200.0, True),
        "Mapping navigation result: CANCELED: "
        "Safety check invalid: OBSTACLE_HOLD_NOT_CLEAR",
        sample(1792814770.0, True),
        sample(1792814775.0, False),
    ]))

    result = summarize_session_safety(
        "TIME_LIMIT_REACHED", path
    )
    assert result["last_observed_obstacle_hold"] is False
    assert not result[
        "timeout_with_unresolved_hold_evidence"
    ]


def test_old_hold_does_not_contaminate_new_goal(tmp_path):
    path = tmp_path / "exploration.log"
    path.write_text("\n".join([
        "MAPPING NAV2 GOAL SENT: (1, 2)",
        sample(1792814200.0, True),
        "Mapping navigation result: CANCELED: "
        "Safety check invalid: OBSTACLE_HOLD_NOT_CLEAR",
        sample(1792814770.0, True),
        "MAPPING NAV2 GOAL SENT: (3, 4)",
        "Mapping navigation result: SUCCEEDED",
    ]))

    result = summarize_session_safety(
        "TIME_LIMIT_REACHED", path
    )
    assert not result[
        "timeout_with_unresolved_hold_evidence"
    ]


def test_missing_log_fails_closed(tmp_path):
    result = summarize_session_safety(
        "TIME_LIMIT_REACHED",
        tmp_path / "nonexistent.log",
    )
    assert result["last_observed_obstacle_hold"] is None
    assert not result[
        "timeout_with_unresolved_hold_evidence"
    ]


def test_safety_annotation_does_not_reclassify(
    tmp_path
):
    path = tmp_path / "exploration.log"
    path.write_text("\n".join([
        "MAPPING NAV2 GOAL SENT:",
        sample(1792814200.0, True),
        "Mapping navigation result: CANCELED: "
        "Safety check invalid: OBSTACLE_HOLD_NOT_CLEAR",
        sample(1792814770.0, True),
    ]))

    result = summarize_session_safety(
        "SAFETY_HALTED_UNFINISHED", path
    )
    assert result[
        "obstacle_hold_unresolved_in_log"
    ] is True
    assert not result[
        "timeout_with_unresolved_hold_evidence"
    ]
