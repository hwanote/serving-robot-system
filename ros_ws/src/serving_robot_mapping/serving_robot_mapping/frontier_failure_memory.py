"""Spatial failure memory for frontier planning candidates.

Pure Python. Records planning failures per approximate frontier
location. Does not command motion or send ROS actions.
"""

import math
from dataclasses import dataclass
from typing import Iterable


@dataclass
class _FailureRecord:
    anchor_x: float
    anchor_y: float
    failures: int = 0


class FrontierFailureMemory:
    """Remember failures despite small shifts in frontier coordinates.

    Each record uses its original fixed anchor.
    """

    def __init__(self, match_radius_m=0.25, max_failures=3):
        if (
            not math.isfinite(match_radius_m)
            or match_radius_m <= 0
        ):
            raise ValueError(
                "match_radius_m must be finite and positive"
            )

        if type(max_failures) is not int or max_failures < 1:
            raise ValueError(
                "max_failures must be a positive integer"
            )

        self.match_radius_m = float(match_radius_m)
        self.max_failures = max_failures
        self._records = []

    @staticmethod
    def _check_xy(x, y):
        if not (
            math.isfinite(x)
            and math.isfinite(y)
        ):
            raise ValueError(
                "Frontier coordinates must be finite"
            )

    def _matching_record(self, x, y):
        self._check_xy(x, y)

        closest = None
        best_distance = self.match_radius_m

        for record in self._records:
            distance = math.hypot(
                x - record.anchor_x,
                y - record.anchor_y,
            )

            if distance <= best_distance:
                closest = record
                best_distance = distance

        return closest

    def failure_count(self, x, y):
        record = self._matching_record(x, y)

        if record is None:
            return 0

        return record.failures

    def is_blocked(self, x, y):
        return (
            self.failure_count(x, y)
            >= self.max_failures
        )

    def record_failure(self, x, y):
        """Record a genuine planning failure."""

        record = self._matching_record(x, y)

        if record is None:
            record = _FailureRecord(
                anchor_x=float(x),
                anchor_y=float(y),
            )
            self._records.append(record)

        record.failures = min(
            self.max_failures,
            record.failures + 1,
        )

        return record.failures

    def record_success(self, x, y):
        """Clear only the matching goal's failure history."""

        record = self._matching_record(x, y)

        if record is None:
            return False

        self._records.remove(record)
        return True

    def available_candidates(
        self,
        candidates: Iterable[tuple[float, float]],
    ):
        """Filter blocked candidates while preserving order."""

        available = []

        for x, y in candidates:
            if not self.is_blocked(x, y):
                available.append((x, y))

        return tuple(available)

    def clear_on_map_change(self):
        """Forget old planning failures only after a confirmed map revision."""
        # R16_172_RELEASE_STABILITY
        self._records.clear()

    @property
    def record_count(self):
        return len(self._records)
