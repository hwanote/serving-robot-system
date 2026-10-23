"""Pure-Python rate limit and retry policy for frontier planning.

This module never sends ROS actions or robot motion commands.
The caller must explicitly discard/cancel stale Nav2 requests.
"""

import math
from dataclasses import dataclass
from typing import Hashable, Optional


@dataclass(frozen=True)
class ReplanDecision:
    allowed: bool
    reason: str
    wait_s: float = 0.0


class ReplanningPolicy:
    def __init__(
        self,
        settle_s: float = 2.0,
        min_interval_s: float = 3.0,
        retry_base_s: float = 2.0,
        retry_max_s: float = 16.0,
        max_failures: int = 3,
    ) -> None:
        values = (
            settle_s,
            min_interval_s,
            retry_base_s,
            retry_max_s,
        )

        if not all(
            math.isfinite(v) and v >= 0.0
            for v in values
        ):
            raise ValueError(
                "Timing parameters must be nonnegative and finite"
            )

        if retry_max_s < retry_base_s:
            raise ValueError(
                "retry_max_s must be >= retry_base_s"
            )

        if type(max_failures) is not int or max_failures < 1:
            raise ValueError(
                "max_failures must be a positive integer"
            )

        self.settle_s = settle_s
        self.min_interval_s = min_interval_s
        self.retry_base_s = retry_base_s
        self.retry_max_s = retry_max_s
        self.max_failures = max_failures

        self.goal_key: Optional[Hashable] = None
        self.pending = False
        self.in_flight = False
        self.consecutive_failures = 0

        self.last_change_s: Optional[float] = None
        self.last_attempt_s: Optional[float] = None
        self.retry_after_s: Optional[float] = None
        self._last_now_s: Optional[float] = None

    def _check_time(self, now_s: float) -> None:
        if not math.isfinite(now_s):
            raise ValueError("Time must be finite")

        if (
            self._last_now_s is not None
            and now_s < self._last_now_s
        ):
            raise ValueError(
                "Monotonic time cannot move backwards"
            )

        self._last_now_s = now_s

    def set_goal(
        self,
        goal_key: Optional[Hashable],
        now_s: float,
    ) -> bool:
        """Reset failure budget only when goal identity changes."""

        self._check_time(now_s)

        if goal_key == self.goal_key:
            return False

        if goal_key is not None:
            hash(goal_key)

        self.goal_key = goal_key
        self.pending = goal_key is not None
        self.in_flight = False
        self.consecutive_failures = 0
        self.retry_after_s = None

        self.last_change_s = (
            now_s if goal_key is not None else None
        )

        # Preserve global request rate limit across goal changes.
        return True

    def context_changed(self, now_s: float) -> None:
        """Handle a real map/pose change.

        Caller must cancel the old action if one is active.
        Identical repeated map messages must not call this.
        """

        self._check_time(now_s)

        if self.goal_key is None:
            return

        self.pending = True
        self.in_flight = False
        self.last_change_s = now_s

        # Do not reset failure budget for the same goal.

    def map_changed(self, now_s: float) -> None:
        """Allow fresh validation after real map content/geometry changes."""
        # R16_172_RELEASE_STABILITY
        self._check_time(now_s)
        if self.goal_key is None:
            return
        self.pending = True
        self.in_flight = False
        self.consecutive_failures = 0
        self.retry_after_s = None
        self.last_change_s = now_s

    def request_recheck(self, now_s: float) -> None:
        """Recheck an expired result without extra map settling."""

        self._check_time(now_s)

        if self.goal_key is None:
            return

        if self.in_flight:
            raise RuntimeError(
                "Cannot recheck while an action is active"
            )

        self.pending = True

    def decision(self, now_s: float) -> ReplanDecision:
        """Return whether a new planning request is allowed."""

        self._check_time(now_s)

        if self.goal_key is None:
            return ReplanDecision(False, "No goal")

        if self.in_flight:
            return ReplanDecision(
                False,
                "Request already in flight",
            )

        if not self.pending:
            return ReplanDecision(
                False,
                "No replan requested",
            )

        if self.consecutive_failures >= self.max_failures:
            return ReplanDecision(
                False,
                "Failure limit reached",
            )

        deadlines = []

        if self.last_change_s is not None:
            deadlines.append((
                self.last_change_s + self.settle_s,
                "Context settling",
            ))

        if self.last_attempt_s is not None:
            deadlines.append((
                self.last_attempt_s + self.min_interval_s,
                "Minimum request interval",
            ))

        if self.retry_after_s is not None:
            deadlines.append((
                self.retry_after_s,
                "Failure backoff",
            ))

        if deadlines:
            deadline, reason = max(
                deadlines,
                key=lambda item: item[0],
            )

            remaining = deadline - now_s

            if remaining > 1e-9:
                return ReplanDecision(
                    False,
                    reason,
                    remaining,
                )

        return ReplanDecision(True, "Ready")

    def begin_attempt(self, now_s: float) -> None:
        """Record a planning request only when permitted."""

        decision = self.decision(now_s)

        if not decision.allowed:
            raise RuntimeError(
                "Planning is not allowed: "
                f"{decision.reason}"
            )

        self.pending = False
        self.in_flight = True
        self.last_attempt_s = now_s

    def finish_attempt(
        self,
        success: bool,
        now_s: float,
    ) -> None:
        """Record success or schedule bounded failure retry."""

        self._check_time(now_s)

        if not self.in_flight:
            raise RuntimeError(
                "No active planning attempt"
            )

        if type(success) is not bool:
            raise ValueError(
                "success must be boolean"
            )

        self.in_flight = False

        if success:
            self.consecutive_failures = 0
            self.pending = False
            self.retry_after_s = None
            return

        self.consecutive_failures += 1
        self.pending = True

        delay = min(
            self.retry_max_s,
            self.retry_base_s * (
                2 ** (self.consecutive_failures - 1)
            ),
        )

        self.retry_after_s = now_s + delay
