"""Conservative safety policy for SLAM-based frontier navigation.

Pure decision logic: no ROS publisher and no navigation command.

All runtime evidence must be supplied by a separate ROS adapter.
Unknown, missing or stale evidence blocks navigation.
"""

import math
from dataclasses import dataclass
from typing import Optional


@dataclass(frozen=True)
class MappingSafetySnapshot:
    """Latest independently observed safety evidence."""

    slam_active: bool = False
    tf_stable: bool = False

    scan_age_s: Optional[float] = None
    odom_age_s: Optional[float] = None
    map_age_s: Optional[float] = None

    map_odom_tf_age_s: Optional[float] = None
    odom_base_tf_age_s: Optional[float] = None

    # Wall-clock receipt age, independent of Gazebo speed.
    health_age_s: Optional[float] = None
    # Simulation-time age from Health JSON's 'time' field.
    health_sim_age_s: Optional[float] = None
    health: Optional[dict] = None

    person_limit_age_s: Optional[float] = None
    person_limit: Optional[dict] = None

    collision_latched: Optional[bool] = None
    obstacle_hold: Optional[bool] = None


class MappingNavigationSafety:
    """Fail-closed safety decision for Mapping mode."""

    @staticmethod
    def _fresh(age, maximum, minimum=0.0):
        if age is None:
            return False

        try:
            value = float(age)
        except (TypeError, ValueError):
            return False

        return (
            math.isfinite(value)
            and minimum <= value <= maximum
        )

    @classmethod
    def evaluate(cls, snapshot, *, lidar_only=False):
        """Return (allowed, reason).

        Only the known AMCL-absent localization error is
        eligible for a tightly limited Mapping exception.
        """

        if type(lidar_only) is not bool:
            return False, "INVALID_MAPPING_PROFILE"

        if not snapshot.slam_active:
            return False, "SLAM_NOT_ACTIVE"

        if not snapshot.tf_stable:
            return False, "TF_NOT_STABLE"

        freshness = (
            ("SCAN_STALE", snapshot.scan_age_s, 2.0),
            ("ODOM_STALE", snapshot.odom_age_s, 2.0),
            ("MAP_STALE", snapshot.map_age_s, 5.0),
            ("HEALTH_STALE", snapshot.health_age_s, 7.0),
            (
                "HEALTH_SIM_STALE",
                snapshot.health_sim_age_s,
                3.0,
            ),
        )

        if not lidar_only:
            freshness += (
                (
                    "PERSON_LIMIT_STALE",
                    snapshot.person_limit_age_s,
                    2.0,
                ),
            )

        for reason, age, maximum in freshness:
            if not cls._fresh(age, maximum):
                return False, reason

        # Match the existing Explorer TF timestamp policy.
        # Only TF is allowed to be slightly future-dated.
        tf_checks = (
            (
                "MAP_ODOM_TF_STALE",
                snapshot.map_odom_tf_age_s,
            ),
            (
                "ODOM_BASE_TF_STALE",
                snapshot.odom_base_tf_age_s,
            ),
        )

        for reason, age in tf_checks:
            if not cls._fresh(
                age,
                maximum=0.50,
                minimum=-0.30,
            ):
                return False, reason

        # Explicit negative observations are required.
        # No message is different from a received False.
        if snapshot.collision_latched is not False:
            return False, "COLLISION_STATE_NOT_CLEAR"

        if snapshot.obstacle_hold is not False:
            return False, "OBSTACLE_HOLD_NOT_CLEAR"

        if not lidar_only:
            limit = snapshot.person_limit

            if not isinstance(limit, dict):
                return False, "PERSON_LIMIT_UNKNOWN"

            if limit.get("silent") is not False:
                return False, "PERSON_DETECTOR_SILENT"

            # Conservative first-motion policy:
            # SLOW or STOP requires a new safety decision.
            if limit.get("state") != "NORMAL":
                return False, "PERSON_LIMIT_NOT_NORMAL"

        health = snapshot.health

        if not isinstance(health, dict):
            return False, "HEALTH_UNKNOWN"

        if health.get("robot_id") != "robot_01":
            return False, "WRONG_ROBOT_HEALTH"

        if health.get("safety_state") != "NORMAL":
            return False, "HEALTH_SAFETY_NOT_NORMAL"

        checks = health.get("checks")

        if not isinstance(checks, dict):
            return False, "HEALTH_CHECKS_MISSING"

        required_checks = (
            ("scan", "odom")
            if lidar_only
            else ("scan", "odom", "detector")
        )

        for name in required_checks:
            if checks.get(name) != "ok":
                return False, f"HEALTH_{name.upper()}_INVALID"

        if checks.get("collision_latched") is not False:
            return False, "HEALTH_COLLISION_LATCHED"

        if checks.get("obstacle_hold") is not False:
            return False, "HEALTH_OBSTACLE_HOLD"

        mission = health.get("mission")

        if not isinstance(mission, dict):
            return False, "MISSION_STATUS_UNKNOWN"

        if mission.get("state") != "IDLE":
            return False, "SERVING_MISSION_NOT_IDLE"

        pose = health.get("pose")

        if not isinstance(pose, dict):
            return False, "MAP_POSE_MISSING"

        if pose.get("frame_id") != "map":
            return False, "WRONG_POSE_FRAME"

        try:
            x = float(pose["x_m"])
            y = float(pose["y_m"])
            yaw = float(pose["yaw_rad"])
        except (KeyError, TypeError, ValueError):
            return False, "MAP_POSE_INVALID"

        if not all(math.isfinite(v) for v in (x, y, yaw)):
            return False, "MAP_POSE_NONFINITE"

        # Deliberately narrow exception for SLAM Mapping:
        # robot_health currently assumes AMCL covariance.
        # Do not allow any additional health error.
        if (
            health.get("robot_state") != "ERROR"
            or health.get("localization_state")
            != "UNINITIALIZED"
            or health.get("reasons")
            != ["LOCALIZATION_UNINITIALIZED"]
        ):
            return False, "HEALTH_NOT_MAPPING_COMPATIBLE"

        return True, "SLAM_MAPPING_CONDITIONS_SATISFIED"
