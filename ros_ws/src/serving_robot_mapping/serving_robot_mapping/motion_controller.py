"""Reusable motion controllers for autonomous mapping."""

import math
from enum import Enum
from typing import Optional


def normalize_angle(angle: float) -> float:
    """Normalize an angular difference to [-pi, pi]."""
    return math.atan2(math.sin(angle), math.cos(angle))


def quaternion_to_yaw(x: float, y: float, z: float, w: float) -> float:
    """Convert a quaternion into planar yaw."""
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


class RotationStatus(str, Enum):
    """State of a relative-angle rotation command."""

    IDLE = "IDLE"
    RUNNING = "RUNNING"
    SETTLING = "SETTLING"
    COMPLETE = "COMPLETE"


class RelativeRotationController:
    """Closed-loop controller for arbitrary relative rotations."""

    def __init__(
        self,
        kp: float,
        max_angular_speed: float,
        min_angular_speed: float,
        angle_tolerance: float,
        stop_angular_velocity: float,
        settle_cycles: int,
    ) -> None:
        self.kp = kp
        self.max_angular_speed = max_angular_speed
        self.min_angular_speed = min_angular_speed
        self.angle_tolerance = angle_tolerance
        self.stop_angular_velocity = stop_angular_velocity
        self.settle_cycles_required = settle_cycles

        self.status = RotationStatus.IDLE

        self.target_angle = 0.0
        self.accumulated_angle = 0.0

        self.previous_yaw: Optional[float] = None
        self.current_angular_velocity = 0.0

        self.settle_cycles = 0

    @property
    def error(self) -> float:
        """Return signed remaining rotation."""
        return self.target_angle - self.accumulated_angle

    @property
    def is_complete(self) -> bool:
        """Return whether the requested rotation has completed."""
        return self.status == RotationStatus.COMPLETE

    def start(self, target_angle: float, current_yaw: float) -> None:
        """Start a new relative rotation command."""
        self.target_angle = target_angle
        self.accumulated_angle = 0.0

        self.previous_yaw = current_yaw
        self.current_angular_velocity = 0.0

        self.settle_cycles = 0
        self.status = RotationStatus.RUNNING

    def update_odometry(
        self,
        current_yaw: float,
        angular_velocity: float,
    ) -> None:
        """Update accumulated signed rotation using odometry."""
        self.current_angular_velocity = angular_velocity

        if self.status == RotationStatus.IDLE:
            self.previous_yaw = current_yaw
            return

        if self.previous_yaw is None:
            self.previous_yaw = current_yaw
            return

        delta = normalize_angle(
            current_yaw - self.previous_yaw
        )

        self.accumulated_angle += delta
        self.previous_yaw = current_yaw

    def compute_command(self) -> float:
        """Return the angular velocity command in rad/s."""
        if self.status in (
            RotationStatus.IDLE,
            RotationStatus.COMPLETE,
        ):
            return 0.0

        error = self.error

        # We are inside the requested angular tolerance.
        if abs(error) <= self.angle_tolerance:
            self.status = RotationStatus.SETTLING

            if (
                abs(self.current_angular_velocity)
                <= self.stop_angular_velocity
            ):
                self.settle_cycles += 1
            else:
                self.settle_cycles = 0

            if self.settle_cycles >= self.settle_cycles_required:
                self.status = RotationStatus.COMPLETE

            return 0.0

        # If inertia moved the robot outside the tolerance again,
        # resume closed-loop correction automatically.
        self.status = RotationStatus.RUNNING
        self.settle_cycles = 0

        raw_command = self.kp * error

        speed = min(
            abs(raw_command),
            self.max_angular_speed,
        )

        speed = max(
            speed,
            self.min_angular_speed,
        )

        if raw_command < 0.0:
            speed = -speed

        return speed

    def cancel(self) -> None:
        """Cancel the active rotation."""
        self.status = RotationStatus.IDLE
        self.target_angle = 0.0
        self.accumulated_angle = 0.0
        self.previous_yaw = None
        self.current_angular_velocity = 0.0
        self.settle_cycles = 0
