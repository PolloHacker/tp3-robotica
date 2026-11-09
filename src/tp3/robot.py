"""Kobuki robot interface for CoppeliaSim simulation."""

from __future__ import annotations

import math
from typing import Any

import numpy as np


class KobukiRobot:
    """Controls the Kobuki robot and reads laser scanner data in CoppeliaSim."""

    WHEEL_RADIUS: float = 0.035
    WHEEL_BASE: float = 0.230
    MAX_LASER_RANGE: float = 5.0
    LASER_RANGE_HIT_THRESHOLD: float = 4.95

    def __init__(
        self,
        sim: Any,
        robot_path: str = "/kobuki",
        laser_path: str = "/fastHokuyo",
        distance_noise_std: float = 0.0,
        angle_noise_std: float = 0.0,
    ) -> None:
        """Initialize robot object handles and sensor parameters.

        Args:
            sim: CoppeliaSim remote API sim object.
            robot_path: Scene path to the Kobuki robot model.
            laser_path: Scene path to the Hokuyo laser model.
            distance_noise_std: Gaussian noise standard deviation for range readings (meters).
            angle_noise_std: Gaussian noise standard deviation for beam angles (radians).
        """
        self.sim = sim
        self.robot_handle = sim.getObject(robot_path)
        self.laser_handle = sim.getObject(laser_path)
        self.left_motor_handle = sim.getObject(f"{robot_path}/kobuki_leftMotor")
        self.right_motor_handle = sim.getObject(f"{robot_path}/kobuki_rightMotor")

        self.distance_noise_std = distance_noise_std
        self.angle_noise_std = angle_noise_std

    def get_pose(self) -> tuple[float, float, float]:
        """Get robot position (x, y) and yaw angle in the world frame.

        Returns:
            Tuple of (x, y, yaw) in meters and radians.
        """
        position = self.sim.getObjectPosition(self.robot_handle, self.sim.handle_world)
        orientation = self.sim.getObjectOrientation(
            self.robot_handle, self.sim.handle_world
        )
        return float(position[0]), float(position[1]), float(orientation[2])

    def set_pose(self, x: float, y: float, yaw: float) -> None:
        """Set robot position and yaw angle in the world frame.

        Args:
            x: Target x coordinate in meters.
            y: Target y coordinate in meters.
            yaw: Target yaw angle in radians.
        """
        current_pos = self.sim.getObjectPosition(
            self.robot_handle, self.sim.handle_world
        )
        z = current_pos[2]
        self.sim.setObjectPosition(
            self.robot_handle, self.sim.handle_world, [x, y, z]
        )
        self.sim.setObjectOrientation(
            self.robot_handle, self.sim.handle_world, [0.0, 0.0, yaw]
        )

    def set_velocities(self, linear_velocity: float, angular_velocity: float) -> None:
        """Set robot linear and angular velocity using differential drive kinematics.

        Args:
            linear_velocity: Desired forward velocity in m/s.
            angular_velocity: Desired rotational velocity in rad/s.
        """
        left_wheel_speed = (
            linear_velocity - (angular_velocity * self.WHEEL_BASE / 2.0)
        ) / self.WHEEL_RADIUS
        right_wheel_speed = (
            linear_velocity + (angular_velocity * self.WHEEL_BASE / 2.0)
        ) / self.WHEEL_RADIUS

        self.sim.setJointTargetVelocity(self.left_motor_handle, left_wheel_speed)
        self.sim.setJointTargetVelocity(self.right_motor_handle, right_wheel_speed)

    def stop(self) -> None:
        """Stop robot motors immediately."""
        self.set_velocities(0.0, 0.0)

    def read_laser_scan(
        self,
    ) -> tuple[np.ndarray, np.ndarray] | tuple[None, None]:
        """Read laser distances and angles from CoppeliaSim buffer signals.

        Returns:
            Tuple of (ranges, angles) as numpy arrays, or (None, None) if unavailable.
        """
        range_buffer = self.sim.getBufferSignal("hokuyo_range_data")
        angle_buffer = self.sim.getBufferSignal("hokuyo_angle_data")

        if not range_buffer or not angle_buffer:
            return None, None

        raw_ranges = np.array(self.sim.unpackFloatTable(range_buffer), dtype=np.float32)
        raw_angles = np.array(self.sim.unpackFloatTable(angle_buffer), dtype=np.float32)

        if self.distance_noise_std > 0.0:
            distance_noise = np.random.normal(
                0.0, self.distance_noise_std, size=raw_ranges.shape
            )
            raw_ranges = np.clip(
                raw_ranges + distance_noise, 0.05, self.MAX_LASER_RANGE
            )

        if self.angle_noise_std > 0.0:
            angle_noise = np.random.normal(
                0.0, self.angle_noise_std, size=raw_angles.shape
            )
            raw_angles = raw_angles + angle_noise

        return raw_ranges, raw_angles

    def get_laser_points_in_world_frame(
        self,
        ranges: np.ndarray,
        angles: np.ndarray,
        robot_x: float,
        robot_y: float,
        robot_yaw: float,
    ) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
        """Transform laser readings to world coordinate frame.

        Args:
            ranges: Laser beam distances in meters.
            angles: Laser beam angles in laser frame (radians).
            robot_x: Robot x coordinate in world frame.
            robot_y: Robot y coordinate in world frame.
            robot_yaw: Robot yaw heading in world frame (radians).

        Returns:
            Tuple of (hit_x, hit_y, is_hit_mask).
        """
        beam_world_angles = robot_yaw + angles
        point_x = robot_x + ranges * np.cos(beam_world_angles)
        point_y = robot_y + ranges * np.sin(beam_world_angles)
        is_hit = ranges < self.LASER_RANGE_HIT_THRESHOLD

        return point_x, point_y, is_hit
