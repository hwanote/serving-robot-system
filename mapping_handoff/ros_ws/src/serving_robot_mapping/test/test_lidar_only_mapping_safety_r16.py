"""Mapping profile must retain LiDAR and collision safety."""

from dataclasses import replace

from serving_robot_mapping.mapping_navigation_safety import (
    MappingNavigationSafety,
    MappingSafetySnapshot,
)


def healthy_snapshot():
    health = {
        "robot_id": "robot_01",
        "safety_state": "NORMAL",
        "checks": {
            "scan": "ok",
            "odom": "ok",
            "collision_latched": False,
            "obstacle_hold": False,
        },
        "mission": {"state": "IDLE"},
        "pose": {
            "frame_id": "map",
            "x_m": 3.0,
            "y_m": 2.0,
            "yaw_rad": 0.0,
        },
        "robot_state": "ERROR",
        "localization_state": "UNINITIALIZED",
        "reasons": ["LOCALIZATION_UNINITIALIZED"],
    }

    return MappingSafetySnapshot(
        slam_active=True,
        tf_stable=True,
        scan_age_s=0.1,
        odom_age_s=0.1,
        map_age_s=0.1,
        map_odom_tf_age_s=0.02,
        odom_base_tf_age_s=0.02,
        health_age_s=0.1,
        health_sim_age_s=0.1,
        health=health,
        person_limit_age_s=None,
        person_limit=None,
        collision_latched=False,
        obstacle_hold=False,
    )


def check(snapshot, *, lidar_only=True):
    return MappingNavigationSafety.evaluate(
        snapshot, lidar_only=lidar_only
    )


def test_lidar_mode_accepts_valid_no_person_environment():
    allowed, reason = check(healthy_snapshot())
    assert allowed, reason


def test_default_profile_still_requires_person_messages():
    allowed, reason = MappingNavigationSafety.evaluate(
        healthy_snapshot()
    )
    assert not allowed
    assert reason == "PERSON_LIMIT_STALE"


def test_lidar_mode_rejects_stale_lidar():
    snapshot = replace(
        healthy_snapshot(), scan_age_s=3.0
    )
    assert check(snapshot) == (False, "SCAN_STALE")


def test_lidar_mode_rejects_unstable_tf():
    snapshot = replace(
        healthy_snapshot(), tf_stable=False
    )
    assert check(snapshot) == (False, "TF_NOT_STABLE")


def test_lidar_mode_rejects_collision_latch():
    snapshot = replace(
        healthy_snapshot(), collision_latched=True
    )
    assert check(snapshot) == (
        False, "COLLISION_STATE_NOT_CLEAR"
    )


def test_lidar_mode_rejects_obstacle_hold():
    snapshot = replace(
        healthy_snapshot(), obstacle_hold=True
    )
    assert check(snapshot) == (
        False, "OBSTACLE_HOLD_NOT_CLEAR"
    )


def test_lidar_mode_rejects_stale_health():
    snapshot = replace(
        healthy_snapshot(), health_age_s=20.0
    )
    assert check(snapshot) == (False, "HEALTH_STALE")


def test_lidar_mode_rejects_unsafe_health():
    snapshot = healthy_snapshot()
    health = dict(snapshot.health)
    health["safety_state"] = "STOP"

    result = check(replace(snapshot, health=health))
    assert result == (False, "HEALTH_SAFETY_NOT_NORMAL")


def test_lidar_mode_rejects_serving_mission():
    snapshot = healthy_snapshot()
    health = dict(snapshot.health)
    health["mission"] = {"state": "RUNNING"}

    assert check(replace(snapshot, health=health)) == (
        False, "SERVING_MISSION_NOT_IDLE"
    )


def test_lidar_mode_rejects_uninitialized_slam():
    snapshot = replace(
        healthy_snapshot(), slam_active=False
    )
    assert check(snapshot) == (False, "SLAM_NOT_ACTIVE")


def test_invalid_profile_type_fails_closed():
    assert check(healthy_snapshot(), lidar_only=1) == (
        False, "INVALID_MAPPING_PROFILE"
    )
