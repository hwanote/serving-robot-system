"""Bound long Mapping sessions blocked by persistent safety holds.

This module never clears a hold, changes a safety threshold,
sends velocity, or authorizes navigation.
"""


class ProlongedObstacleHoldWatch:
    """Recognize repeated evidence of one continuous hold."""

    def __init__(
        self,
        limit_s=120.0,
        min_samples=3,
        max_sample_gap_s=30.0,
    ):
        self.limit_s = float(limit_s)
        self.min_samples = int(min_samples)
        self.max_gap_s = float(max_sample_gap_s)

        if (
            self.limit_s < 30.0
            or self.min_samples < 2
            or self.max_gap_s <= 0.0
        ):
            raise ValueError("Invalid hold watch settings")

        self.reset()

    def reset(self):
        self.started = None
        self.last_sample = None
        self.samples = 0
        self.elapsed_s = 0.0

    def observe(self, line, now):
        """Return True only on prolonged repeated hold evidence."""

        if (
            "MAPPING NAV2 GOAL SENT:" in line
            or "Mapping navigation result: SUCCEEDED:"
            in line
            or "EXPLORATION_BLOCKED -> EXPLORING"
            in line
        ):
            self.reset()
            return False

        if "Mapping safety diagnostic:" not in line:
            return False

        if "phase=RECOVERY" not in line:
            return False

        still_held = (
            "reason=OBSTACLE_HOLD_NOT_CLEAR" in line
            and "obstacle_hold=True" in line
        )

        if not still_held:
            self.reset()
            return False

        if (
            self.last_sample is not None
            and (
                now < self.last_sample
                or now - self.last_sample > self.max_gap_s
            )
        ):
            self.reset()

        if self.started is None:
            self.started = now

        self.last_sample = now
        self.samples += 1
        self.elapsed_s = max(0.0, now - self.started)

        return (
            self.samples >= self.min_samples
            and self.elapsed_s >= self.limit_s
        )
