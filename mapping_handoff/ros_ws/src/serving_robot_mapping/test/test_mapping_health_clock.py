"""Offline checks for the simulation clock watchdog."""

import unittest
from types import SimpleNamespace

from serving_robot_mapping.mapping_navigation_safety_monitor import (
    MappingNavigationSafetyMonitor,
)


class ClockWatchdogTests(unittest.TestCase):
    def setUp(self):
        self.sim = {"time": 10.0}

        def clock():
            return SimpleNamespace(
                now=lambda: SimpleNamespace(
                    nanoseconds=int(
                        self.sim["time"] * 1e9
                    )
                )
            )

        self.monitor = object.__new__(
            MappingNavigationSafetyMonitor
        )
        self.monitor.node = SimpleNamespace(
            get_clock=clock
        )
        self.monitor._last_sim_value = None
        self.monitor._last_sim_progress = None
        self.monitor.tf_valid_since = None

    def test_clock_start_then_progress(self):
        fresh, _ = self.monitor._clock_progress(0.0)
        self.assertFalse(fresh)

        self.sim["time"] = 10.1

        fresh, _ = self.monitor._clock_progress(0.2)
        self.assertTrue(fresh)

    def test_paused_clock_is_rejected(self):
        self.monitor._clock_progress(0.0)
        self.sim["time"] = 10.1
        self.monitor._clock_progress(0.2)

        self.monitor.tf_valid_since = 0.2

        fresh, _ = self.monitor._clock_progress(2.3)

        self.assertFalse(fresh)
        self.assertIsNone(
            self.monitor.tf_valid_since
        )

    def test_clock_recovers_after_pause(self):
        self.monitor._clock_progress(0.0)
        self.sim["time"] = 10.1
        self.monitor._clock_progress(0.2)
        self.monitor._clock_progress(2.3)

        self.sim["time"] = 10.2

        fresh, _ = self.monitor._clock_progress(2.4)
        self.assertTrue(fresh)

    def test_backward_clock_is_rejected(self):
        self.monitor._clock_progress(0.0)
        self.sim["time"] = 11.0
        self.monitor._clock_progress(0.2)

        self.sim["time"] = 9.0

        fresh, _ = self.monitor._clock_progress(0.3)
        self.assertFalse(fresh)

    def test_unavailable_clock_is_rejected(self):
        self.sim["time"] = 0.0

        fresh, _ = self.monitor._clock_progress(0.0)
        self.assertFalse(fresh)


if __name__ == "__main__":
    unittest.main(verbosity=2)
