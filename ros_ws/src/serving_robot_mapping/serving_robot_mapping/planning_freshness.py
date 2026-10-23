"""Track whether a frontier planning result is still current.

Pure Python logic; does not publish ROS messages or move robots.
"""

import hashlib
import math
from array import array
from dataclasses import dataclass
from typing import Optional, Sequence


@dataclass(frozen=True)
class PlanningSnapshot:
    """Map and robot pose recorded for a planning request."""

    map_revision: int
    robot_x: float
    robot_y: float
    robot_yaw: float
    created_at_s: float


class PlanningFreshnessMonitor:
    """Detect changes requiring a new planning request."""

    def __init__(
        self,
        max_age_s: float = 10.0,
        max_translation_m: float = 0.15,
        max_rotation_rad: float = math.radians(15.0),
    ) -> None:
        if not all(
            math.isfinite(value) and value > 0
            for value in (
                max_age_s,
                max_translation_m,
                max_rotation_rad,
            )
        ):
            raise ValueError(
                "Freshness thresholds must be finite and positive"
            )

        self.max_age_s = max_age_s
        self.max_translation_m = max_translation_m
        self.max_rotation_rad = max_rotation_rad

        self.map_revision = 0
        self._map_signature = None
        self._robot_pose = None

    def update_map(
        self,
        data: Sequence[int],
        width: int,
        height: int,
        resolution: float,
        origin_x: float,
        origin_y: float,
        origin_yaw: float,
    ) -> bool:
        """Return True only if map contents or geometry changed."""

        if width <= 0 or height <= 0:
            raise ValueError("Invalid map dimensions")

        if len(data) != width * height:
            raise ValueError("Map data size mismatch")

        if not all(
            math.isfinite(value)
            for value in (
                resolution,
                origin_x,
                origin_y,
                origin_yaw,
            )
        ) or resolution <= 0:
            raise ValueError("Invalid map metadata")

        # ROS OccupancyGrid uses signed int8 cell values.
        raw_cells = array("b", data).tobytes()

        digest = hashlib.blake2b(
            raw_cells,
            digest_size=16,
        ).digest()

        # Header timestamps are intentionally excluded.
        signature = (
            width,
            height,
            resolution,
            origin_x,
            origin_y,
            origin_yaw,
            digest,
        )

        if signature == self._map_signature:
            return False

        self._map_signature = signature
        self.map_revision += 1

        return True

    def update_robot_pose(
        self,
        x: float,
        y: float,
        yaw: float,
    ) -> None:
        """Store the latest robot pose in the map frame."""

        if not all(
            math.isfinite(value)
            for value in (x, y, yaw)
        ):
            raise ValueError("Invalid robot pose")

        normalized_yaw = math.atan2(
            math.sin(yaw),
            math.cos(yaw),
        )

        self._robot_pose = (
            x,
            y,
            normalized_yaw,
        )

    def capture(self, now_s: float) -> PlanningSnapshot:
        """Capture the context of one planning request."""

        if self._map_signature is None:
            raise RuntimeError("Map not available")

        if self._robot_pose is None:
            raise RuntimeError("Robot pose not available")

        if not math.isfinite(now_s):
            raise ValueError("Invalid request timestamp")

        x, y, yaw = self._robot_pose

        return PlanningSnapshot(
            map_revision=self.map_revision,
            robot_x=x,
            robot_y=y,
            robot_yaw=yaw,
            created_at_s=now_s,
        )

    def validate(
        self,
        snapshot: PlanningSnapshot,
        now_s: float,
    ) -> tuple[bool, str]:
        """Check whether a planning snapshot remains current."""

        if not math.isfinite(now_s):
            return False, "Invalid current timestamp"

        if self._map_signature is None:
            return False, "Map unavailable"

        if self._robot_pose is None:
            return False, "Robot pose unavailable"

        if snapshot.map_revision != self.map_revision:
            return False, "Map changed"

        age = now_s - snapshot.created_at_s

        if age < 0:
            return False, "Clock moved backwards"

        if age > self.max_age_s:
            return False, "Planning result expired"

        x, y, yaw = self._robot_pose

        distance = math.hypot(
            x - snapshot.robot_x,
            y - snapshot.robot_y,
        )

        if distance > self.max_translation_m:
            return False, (
                f"Robot moved {distance:.3f} m"
            )

        angle_delta = math.atan2(
            math.sin(yaw - snapshot.robot_yaw),
            math.cos(yaw - snapshot.robot_yaw),
        )

        if abs(angle_delta) > self.max_rotation_rad:
            return False, (
                "Robot orientation changed "
                f"{math.degrees(abs(angle_delta)):.2f} deg"
            )

        return True, "Planning context is current"
