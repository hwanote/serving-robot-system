"""Offline tests for the Mapping navigation safety policy."""

import unittest
from dataclasses import replace

from serving_robot_mapping.mapping_navigation_safety import (
    MappingNavigationSafety,
    MappingSafetySnapshot,
)


def valid_snapshot():
    return MappingSafetySnapshot(
        slam_active=True,
        tf_stable=True,
        scan_age_s=0.1,
        odom_age_s=0.1,
        map_age_s=0.5,
        map_odom_tf_age_s=0.1,
        odom_base_tf_age_s=0.1,
        health_age_s=0.2,
        health_sim_age_s=0.2,
        person_limit_age_s=0.2,
        collision_latched=False,
        obstacle_hold=False,
        person_limit={
            "state": "NORMAL",
            "silent": False,
        },
        health={
            "robot_id": "robot_01",
            "robot_state": "ERROR",
            "safety_state": "NORMAL",
            "localization_state": "UNINITIALIZED",
            "reasons": ["LOCALIZATION_UNINITIALIZED"],
            "pose": {
                "frame_id": "map",
                "x_m": 0.0,
                "y_m": 0.0,
                "yaw_rad": 0.0,
            },
            "mission": {"state": "IDLE"},
            "checks": {
                "scan": "ok",
                "odom": "ok",
                "detector": "ok",
                "collision_latched": False,
                "obstacle_hold": False,
            },
        },
    )


class MappingSafetyTests(unittest.TestCase):
    def check_blocked(self, snapshot):
        allowed, reason = MappingNavigationSafety.evaluate(
            snapshot
        )
        self.assertFalse(allowed)
        self.assertTrue(reason)

    def test_mapping_only_exception(self):
        allowed, reason = MappingNavigationSafety.evaluate(
            valid_snapshot()
        )
        self.assertTrue(allowed)
        self.assertEqual(
            reason,
            "SLAM_MAPPING_CONDITIONS_SATISFIED",
        )

    def test_missing_slam(self):
        self.check_blocked(
            replace(valid_snapshot(), slam_active=False)
        )

    def test_unstable_tf(self):
        self.check_blocked(
            replace(valid_snapshot(), tf_stable=False)
        )

    def test_stale_scan(self):
        self.check_blocked(
            replace(valid_snapshot(), scan_age_s=3.0)
        )

    def test_missing_tf_age(self):
        self.check_blocked(
            replace(
                valid_snapshot(),
                map_odom_tf_age_s=None,
            )
        )

    def test_collision_latched(self):
        self.check_blocked(
            replace(
                valid_snapshot(),
                collision_latched=True,
            )
        )

    def test_unknown_collision_state(self):
        self.check_blocked(
            replace(
                valid_snapshot(),
                collision_latched=None,
            )
        )

    def test_person_detector_silent(self):
        sample = valid_snapshot()
        limit = dict(sample.person_limit)
        limit["silent"] = True
        self.check_blocked(
            replace(sample, person_limit=limit)
        )

    def test_additional_health_error(self):
        sample = valid_snapshot()
        health = dict(sample.health)
        health["reasons"] = [
            "LOCALIZATION_UNINITIALIZED",
            "SCAN_STALE",
        ]
        self.check_blocked(
            replace(sample, health=health)
        )

    def test_serving_mission_active(self):
        sample = valid_snapshot()
        health = dict(sample.health)
        health["mission"] = {"state": "DRIVING"}
        self.check_blocked(
            replace(sample, health=health)
        )

    def test_bad_robot_health(self):
        sample = valid_snapshot()
        health = dict(sample.health)
        health["robot_id"] = "robot_02"
        self.check_blocked(
            replace(sample, health=health)
        )

    def test_missing_health(self):
        self.check_blocked(
            replace(valid_snapshot(), health=None)
        )


    def test_future_tf_within_tolerance(self):
        sample = replace(
            valid_snapshot(),
            map_odom_tf_age_s=-0.155,
            odom_base_tf_age_s=-0.02,
        )

        allowed, reason = MappingNavigationSafety.evaluate(
            sample
        )

        self.assertTrue(allowed)
        self.assertEqual(
            reason,
            "SLAM_MAPPING_CONDITIONS_SATISFIED",
        )

    def test_future_map_tf_beyond_limit(self):
        sample = replace(
            valid_snapshot(),
            map_odom_tf_age_s=-0.31,
        )

        allowed, reason = MappingNavigationSafety.evaluate(
            sample
        )

        self.assertFalse(allowed)
        self.assertEqual(reason, "MAP_ODOM_TF_STALE")

    def test_future_odom_tf_beyond_limit(self):
        sample = replace(
            valid_snapshot(),
            odom_base_tf_age_s=-0.31,
        )

        allowed, reason = MappingNavigationSafety.evaluate(
            sample
        )

        self.assertFalse(allowed)
        self.assertEqual(reason, "ODOM_BASE_TF_STALE")

    def test_tf_too_old(self):
        sample = replace(
            valid_snapshot(),
            map_odom_tf_age_s=0.51,
        )

        allowed, reason = MappingNavigationSafety.evaluate(
            sample
        )

        self.assertFalse(allowed)
        self.assertEqual(reason, "MAP_ODOM_TF_STALE")

    def test_negative_scan_age_still_rejected(self):
        sample = replace(
            valid_snapshot(),
            scan_age_s=-0.01,
        )

        allowed, reason = MappingNavigationSafety.evaluate(
            sample
        )

        self.assertFalse(allowed)
        self.assertEqual(reason, "SCAN_STALE")



    def test_slow_gazebo_health_within_both_limits(self):
        sample = replace(
            valid_snapshot(),
            health_age_s=5.1,
            health_sim_age_s=1.2,
        )

        allowed, reason = MappingNavigationSafety.evaluate(
            sample
        )

        self.assertTrue(allowed, reason)

    def test_health_wall_limit(self):
        sample = replace(
            valid_snapshot(),
            health_age_s=7.1,
            health_sim_age_s=1.2,
        )

        allowed, reason = MappingNavigationSafety.evaluate(
            sample
        )

        self.assertFalse(allowed)
        self.assertEqual(reason, "HEALTH_STALE")

    def test_health_sim_limit(self):
        sample = replace(
            valid_snapshot(),
            health_age_s=1.0,
            health_sim_age_s=3.1,
        )

        allowed, reason = MappingNavigationSafety.evaluate(
            sample
        )

        self.assertFalse(allowed)
        self.assertEqual(reason, "HEALTH_SIM_STALE")

    def test_missing_health_sim_stamp(self):
        sample = replace(
            valid_snapshot(),
            health_sim_age_s=None,
        )

        allowed, reason = MappingNavigationSafety.evaluate(
            sample
        )

        self.assertFalse(allowed)
        self.assertEqual(reason, "HEALTH_SIM_STALE")

    def test_future_health_sim_stamp(self):
        sample = replace(
            valid_snapshot(),
            health_sim_age_s=-0.01,
        )

        allowed, reason = MappingNavigationSafety.evaluate(
            sample
        )

        self.assertFalse(allowed)
        self.assertEqual(reason, "HEALTH_SIM_STALE")


if __name__ == "__main__":
    unittest.main(verbosity=2)
