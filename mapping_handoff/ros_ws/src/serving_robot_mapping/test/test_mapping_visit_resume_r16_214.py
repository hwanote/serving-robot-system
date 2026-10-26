import json

import pytest

from serving_robot_mapping.mapping_visit_resume import (
    restore_visits,
    successful_visits,
)


LOG = """
MAPPING NAV2 GOAL SENT: (0.80, 0.90), path=1.0 m
Mapping navigation result: SUCCEEDED: Nav2 reported arrival
MAPPING NAV2 GOAL SENT: (1.20, 1.40), path=1.0 m
Mapping navigation result: CANCELED: Safety check invalid
MAPPING NAV2 GOAL SENT: (1.50, 1.60), path=1.0 m
Mapping navigation result: SUCCEEDED: Nav2 reported arrival
"""


def test_only_successful_visits_restored():
    assert successful_visits(LOG) == [
        (0.8, 0.9),
        (1.5, 1.6),
    ]


def make_report(tmp_path):
    report = tmp_path / "R16-147_session_result.json"

    report.write_text(json.dumps({
        "raw_map_saved": True,
        "navigation_map_authorized": False,
        "map_dimensions": [30, 30],
        "map_origin": [0.0, 0.0],
        "resolution_m": 0.1,
    }))

    (tmp_path / "R16-147_exploration.log").write_text(LOG)

    return report


def test_compatible_live_map_restores_history(tmp_path):
    report = make_report(tmp_path)

    result = restore_visits(
        report, [0] * 900,
        30, 30, 0.1, 0.0, 0.0, 0.0,
    )

    assert result == [(0.8, 0.9), (1.5, 1.6)]


def test_changed_map_origin_blocks_resume(tmp_path):
    report = make_report(tmp_path)

    with pytest.raises(ValueError):
        restore_visits(
            report, [0] * 900,
            30, 30, 0.1, 0.5, 0.0, 0.0,
        )


def test_unknown_at_prior_goal_blocks_resume(tmp_path):
    report = make_report(tmp_path)
    data = [0] * 900
    data[9 * 30 + 8] = -1

    with pytest.raises(ValueError):
        restore_visits(
            report, data,
            30, 30, 0.1, 0.0, 0.0, 0.0,
        )
