"""Validate timestamps of the robot's dynamic TF chain.

Checks map -> odom and odom -> base_footprint separately.

All timestamps must use the same ROS simulation clock.
This module does not send navigation or velocity commands.
"""

import math
from dataclasses import dataclass
from typing import Optional


@dataclass(frozen=True)
class TfFreshnessResult:
    valid: bool
    reason: str
    map_odom_age_s: Optional[float] = None
    odom_base_age_s: Optional[float] = None


class TfFreshnessChecker:
    """Reject missing, stale, or excessively future-dated TF."""

    def __init__(
        self,
        max_past_age_s=0.50,
        max_future_offset_s=0.30,
    ):
        for name, value in (
            ("max_past_age_s", max_past_age_s),
            ("max_future_offset_s", max_future_offset_s),
        ):
            if not math.isfinite(value) or value <= 0.0:
                raise ValueError(
                    f"{name} must be finite and positive"
                )

        self.max_past_age_s = float(max_past_age_s)
        self.max_future_offset_s = float(
            max_future_offset_s
        )

    def validate(
        self,
        sim_now_s,
        map_odom_stamp_s,
        odom_base_stamp_s,
    ):
        """Check two dynamic TF stamps against simulation time."""

        if (
            not math.isfinite(sim_now_s)
            or sim_now_s <= 0.0
        ):
            return TfFreshnessResult(
                False,
                "Simulation clock unavailable",
            )

        stamps = (
            ("map -> odom", map_odom_stamp_s),
            ("odom -> base_footprint", odom_base_stamp_s),
        )

        ages = []

        for frame_name, stamp in stamps:
            if not math.isfinite(stamp) or stamp <= 0.0:
                return TfFreshnessResult(
                    False,
                    f"{frame_name}: invalid TF timestamp",
                )

            age_s = sim_now_s - stamp
            ages.append(age_s)

            if age_s > self.max_past_age_s:
                return TfFreshnessResult(
                    False,
                    f"{frame_name}: TF too old "
                    f"({age_s:.3f} s)",
                    *(
                        ages
                        if len(ages) == 2
                        else (ages[0], None)
                    ),
                )

            if age_s < -self.max_future_offset_s:
                return TfFreshnessResult(
                    False,
                    f"{frame_name}: TF too far in future "
                    f"({age_s:.3f} s)",
                    *(
                        ages
                        if len(ages) == 2
                        else (ages[0], None)
                    ),
                )

        return TfFreshnessResult(
            valid=True,
            reason="TF timestamps are current",
            map_odom_age_s=ages[0],
            odom_base_age_s=ages[1],
        )
