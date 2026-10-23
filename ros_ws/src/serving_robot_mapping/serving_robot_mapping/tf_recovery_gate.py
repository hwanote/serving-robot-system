"""Require a stable recovery interval after a TF failure.

This module is independent of ROS.
Time values must come from a monotonic clock.

It does not publish commands or navigation goals.
"""

import math


class TfRecoveryGate:
    """Block replanning until TF recovery remains stable."""

    def __init__(self, stable_s=1.0):
        if (
            not math.isfinite(stable_s)
            or stable_s <= 0.0
        ):
            raise ValueError(
                "stable_s must be finite and positive"
            )

        self.stable_s = float(stable_s)

        # Initial healthy TF does not require recovery delay.
        self._recovering = False
        self._valid_since = None
        self._last_now = None

    @property
    def in_recovery(self):
        return self._recovering

    def _enter_recovery(self):
        self._recovering = True
        self._valid_since = None

    def observe(self, tf_valid, now_s):
        """Return True only when planning may resume.

        Every failed TF observation resets recovery.
        A backwards or invalid monotonic timestamp
        also fails closed.
        """

        if (
            not math.isfinite(now_s)
            or now_s < 0.0
        ):
            self._enter_recovery()
            self._last_now = None
            return False

        now_s = float(now_s)

        if (
            self._last_now is not None
            and now_s < self._last_now
        ):
            self._enter_recovery()
            self._last_now = now_s
            return False

        self._last_now = now_s

        if not tf_valid:
            self._enter_recovery()
            return False

        # No prior TF failure: initial healthy startup.
        if not self._recovering:
            return True

        # First healthy sample after failure.
        if self._valid_since is None:
            self._valid_since = now_s
            return False

        elapsed = now_s - self._valid_since

        if elapsed < self.stable_s:
            return False

        # Recovery is complete.
        self._recovering = False
        self._valid_since = None

        return True
