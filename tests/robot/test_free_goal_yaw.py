"""Mapping mode must not turn to the (meaningless) goal direction."""

from pathlib import Path

ROOT = Path(__file__).resolve().parents[2] / 'ros_ws' / 'src'
NAV = ROOT / 'serving_robot_navigation' / 'launch' / 'navigation.launch.py'
MAPPING = ROOT / 'serving_robot_bringup' / 'launch' / 'mapping_mode.launch.py'


def test_navigation_defaults_to_turning_to_the_goal_direction():
    text = NAV.read_text(encoding='utf-8')
    start = text.index("'free_goal_yaw', default_value=")
    assert "default_value='false'" in text[start:start + 60]


def test_navigation_overrides_both_parameters_together():
    text = NAV.read_text(encoding='utf-8')
    assert "'FollowPath.rotate_to_goal_heading'] = False" in text
    assert "'general_goal_checker.yaw_goal_tolerance'] = 3.14" in text


def test_mapping_mode_turns_it_on():
    text = MAPPING.read_text(encoding='utf-8')
    assert "'free_goal_yaw': 'true'" in text
