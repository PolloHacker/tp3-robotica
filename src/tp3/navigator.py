"""Reactive obstacle avoidance and exploration navigator for differential drive robots."""

from __future__ import annotations

import math
import random
from typing import Sequence

import numpy as np


class ReactiveNavigator:
    """Computes linear and angular velocities to explore the environment safely."""

    def __init__(
        self,
        base_linear_velocity: float = 0.30,
        turn_angular_velocity: float = 1.0,
        front_obstacle_threshold: float = 0.70,
        side_obstacle_threshold: float = 0.50,
        stuck_check_interval: int = 40,
        stuck_distance_threshold: float = 0.08,
    ) -> None:
        """Initialize navigation parameters and state.

        Args:
            base_linear_velocity: Forward nominal velocity in m/s.
            turn_angular_velocity: Nominal rotational velocity during turns in rad/s.
            front_obstacle_threshold: Minimum front distance to trigger an avoidance turn in meters.
            side_obstacle_threshold: Side proximity distance to apply gentle steering bias in meters.
            stuck_check_interval: Number of steps between robot displacement checks.
            stuck_distance_threshold: Minimum displacement required to not be considered stuck.
        """
        self.base_linear_velocity = base_linear_velocity
        self.turn_angular_velocity = turn_angular_velocity
        self.front_obstacle_threshold = front_obstacle_threshold
        self.side_obstacle_threshold = side_obstacle_threshold

        self.stuck_check_interval = stuck_check_interval
        self.stuck_distance_threshold = stuck_distance_threshold

        self.step_counter: int = 0
        self.last_check_position: tuple[float, float] | None = None
        self.escape_steps_remaining: int = 0
        self.escape_turn_direction: float = 1.0

    def compute_sector_distances(
        self, ranges: np.ndarray, angles: np.ndarray
    ) -> tuple[float, float, float]:
        """Compute the minimum obstacle distance in front, left, and right sectors.

        Args:
            ranges: Array of laser distances in meters.
            angles: Array of beam angles in laser frame (radians).

        Returns:
            Tuple of (min_front_distance, min_left_distance, min_right_distance).
        """
        front_angle_limit = math.radians(25.0)
        side_angle_limit = math.radians(85.0)

        front_mask = (angles >= -front_angle_limit) & (angles <= front_angle_limit)
        left_mask = (angles > front_angle_limit) & (angles <= side_angle_limit)
        right_mask = (angles < -front_angle_limit) & (angles >= -side_angle_limit)

        front_min = float(np.min(ranges[front_mask])) if np.any(front_mask) else 5.0
        left_min = float(np.min(ranges[left_mask])) if np.any(left_mask) else 5.0
        right_min = float(np.min(ranges[right_mask])) if np.any(right_mask) else 5.0

        return front_min, left_min, right_min

    def compute_commands(
        self,
        robot_x: float,
        robot_y: float,
        ranges: np.ndarray,
        angles: np.ndarray,
    ) -> tuple[float, float]:
        """Compute velocity commands (v, omega) based on laser sector distances.

        Args:
            robot_x: Current robot x coordinate in world frame.
            robot_y: Current robot y coordinate in world frame.
            ranges: Array of laser distances in meters.
            angles: Array of laser angles in radians.

        Returns:
            Tuple of (linear_velocity, angular_velocity).
        """
        self.step_counter += 1

        if self.step_counter % self.stuck_check_interval == 0:
            if self.last_check_position is not None:
                dx = robot_x - self.last_check_position[0]
                dy = robot_y - self.last_check_position[1]
                distance_moved = math.hypot(dx, dy)
                if distance_moved < self.stuck_distance_threshold:
                    self.escape_steps_remaining = 25
                    self.escape_turn_direction = 1.0 if random.random() < 0.5 else -1.0
            self.last_check_position = (robot_x, robot_y)

        if self.escape_steps_remaining > 0:
            self.escape_steps_remaining -= 1
            if self.escape_steps_remaining > 12:
                return -0.15, 0.0
            return 0.0, self.escape_turn_direction * self.turn_angular_velocity

        front_dist, left_dist, right_dist = self.compute_sector_distances(
            ranges, angles
        )

        if front_dist < self.front_obstacle_threshold:
            turn_direction = 1.0 if left_dist >= right_dist else -1.0
            linear_velocity = 0.05 if front_dist > 0.45 else 0.0
            angular_velocity = turn_direction * self.turn_angular_velocity
            return linear_velocity, angular_velocity

        linear_velocity = self.base_linear_velocity
        angular_velocity = 0.0

        if left_dist < self.side_obstacle_threshold:
            angular_velocity -= 0.45 * (
                (self.side_obstacle_threshold - left_dist) / self.side_obstacle_threshold
            )
        elif right_dist < self.side_obstacle_threshold:
            angular_velocity += 0.45 * (
                (self.side_obstacle_threshold - right_dist) / self.side_obstacle_threshold
            )
        else:
            if random.random() < 0.08:
                angular_velocity += random.uniform(-0.35, 0.35)

        return linear_velocity, angular_velocity
