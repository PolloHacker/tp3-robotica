"""Occupancy Grid Mapping using Log-Odds representation and Bresenham ray-casting."""

from __future__ import annotations

import math
from typing import Sequence

import matplotlib.pyplot as plt
import numpy as np
from PIL import Image


def bresenham_line(
    x0: int, y0: int, x1: int, y1: int
) -> list[tuple[int, int]]:
    """Compute grid coordinates along a 2D line segment using Bresenham's algorithm.

    Args:
        x0: Starting column index.
        y0: Starting row index.
        x1: Ending column index.
        y1: Ending row index.

    Returns:
        List of (x, y) grid coordinates from start to end inclusive.
    """
    cells: list[tuple[int, int]] = []
    dx = abs(x1 - x0)
    dy = abs(y1 - y0)
    sx = 1 if x0 < x1 else -1
    sy = 1 if y0 < y1 else -1
    err = dx - dy

    current_x = x0
    current_y = y0

    while True:
        cells.append((current_x, current_y))
        if current_x == x1 and current_y == y1:
            break
        error_doubled = 2 * err
        if error_doubled > -dy:
            err -= dy
            current_x += sx
        if error_doubled < dx:
            err += dx
            current_y += sy

    return cells


class OccupancyGrid:
    """Represents a 2D occupancy grid map updated with log-odds."""

    def __init__(
        self,
        min_x: float = -5.5,
        max_x: float = 5.5,
        min_y: float = -5.5,
        max_y: float = 5.5,
        cell_size: float = 0.10,
        prob_occ: float = 0.85,
        prob_free: float = 0.40,
        log_odds_min: float = -5.0,
        log_odds_max: float = 5.0,
    ) -> None:
        """Initialize grid dimensions, log-odds values, and limits.

        Args:
            min_x: Minimum x bound of the map in meters.
            max_x: Maximum x bound of the map in meters.
            min_y: Minimum y bound of the map in meters.
            max_y: Maximum y bound of the map in meters.
            cell_size: Side length of each square cell in meters.
            prob_occ: Sensor inverse model probability for occupied cell.
            prob_free: Sensor inverse model probability for free cell.
            log_odds_min: Minimum clamp threshold for log-odds.
            log_odds_max: Maximum clamp threshold for log-odds.
        """
        self.min_x = min_x
        self.max_x = max_x
        self.min_y = min_y
        self.max_y = max_y
        self.cell_size = cell_size

        self.width_cells = int(math.ceil((max_x - min_x) / cell_size))
        self.height_cells = int(math.ceil((max_y - min_y) / cell_size))

        self.log_odds = np.zeros(
            (self.height_cells, self.width_cells), dtype=np.float32
        )

        self.log_odds_occ = float(math.log(prob_occ / (1.0 - prob_occ)))
        self.log_odds_free = float(math.log(prob_free / (1.0 - prob_free)))

        self.log_odds_min = log_odds_min
        self.log_odds_max = log_odds_max

    def world_to_grid(self, x: float, y: float) -> tuple[int, int]:
        """Convert continuous world coordinates (meters) to integer grid indices (col, row).

        Args:
            x: World x coordinate in meters.
            y: World y coordinate in meters.

        Returns:
            Tuple of (column_index, row_index).
        """
        column = int(math.floor((x - self.min_x) / self.cell_size))
        row = int(math.floor((y - self.min_y) / self.cell_size))
        return column, row

    def grid_to_world(self, column: int, row: int) -> tuple[float, float]:
        """Convert integer grid indices (col, row) to world coordinates (meters) at cell center.

        Args:
            column: Grid column index.
            row: Grid row index.

        Returns:
            Tuple of (world_x, world_y) in meters.
        """
        world_x = self.min_x + (column + 0.5) * self.cell_size
        world_y = self.min_y + (row + 0.5) * self.cell_size
        return world_x, world_y

    def is_inside(self, column: int, row: int) -> bool:
        """Check if grid cell coordinates are within the grid bounds.

        Args:
            column: Grid column index.
            row: Grid row index.

        Returns:
            True if inside grid boundaries, False otherwise.
        """
        return 0 <= column < self.width_cells and 0 <= row < self.height_cells

    def update(
        self,
        robot_x: float,
        robot_y: float,
        robot_yaw: float,
        ranges: np.ndarray,
        angles: np.ndarray,
        beam_step: int = 2,
        hit_threshold: float = 4.95,
    ) -> None:
        """Update occupancy grid log-odds using laser scan and robot pose.

        Args:
            robot_x: Robot x coordinate in world frame.
            robot_y: Robot y coordinate in world frame.
            robot_yaw: Robot orientation in world frame (radians).
            ranges: Array of laser distances in meters.
            angles: Array of laser beam angles in laser frame (radians).
            beam_step: Subsampling stride across laser beams to balance detail and speed.
            hit_threshold: Distance below which a beam is treated as hitting an obstacle.
        """
        start_col, start_row = self.world_to_grid(robot_x, robot_y)
        if not self.is_inside(start_col, start_row):
            return

        total_beams = len(ranges)
        for index in range(0, total_beams, beam_step):
            beam_range = float(ranges[index])
            beam_angle = float(angles[index])
            world_angle = robot_yaw + beam_angle

            has_hit = beam_range < hit_threshold
            end_x = robot_x + beam_range * math.cos(world_angle)
            end_y = robot_y + beam_range * math.sin(world_angle)

            end_col, end_row = self.world_to_grid(end_x, end_y)

            ray_cells = bresenham_line(start_col, start_row, end_col, end_row)

            free_cells = ray_cells[:-1] if has_hit else ray_cells
            for col, row in free_cells:
                if self.is_inside(col, row):
                    self.log_odds[row, col] += self.log_odds_free

            if has_hit and self.is_inside(end_col, end_row):
                self.log_odds[end_row, end_col] += self.log_odds_occ

        np.clip(
            self.log_odds,
            self.log_odds_min,
            self.log_odds_max,
            out=self.log_odds,
        )

    def get_probability_map(self) -> np.ndarray:
        """Compute the occupancy probability map from log-odds.

        Returns:
            2D numpy array with values in [0.0, 1.0].
        """
        return 1.0 - (1.0 / (1.0 + np.exp(self.log_odds)))

    def save_map_image(self, file_path: str) -> None:
        """Save the occupancy grid as a grayscale image.

        Free space is white (high brightness), occupied cells are black (dark),
        and unknown cells are gray (medium brightness).

        Args:
            file_path: Destination path for the saved image file.
        """
        prob_map = self.get_probability_map()
        display_map = 1.0 - prob_map
        pixel_array = np.uint8(np.clip(display_map * 255.0, 0, 255))
        image = Image.fromarray(np.flipud(pixel_array), mode="L")
        image.save(file_path)

    def plot_map(
        self,
        ax: plt.Axes | None = None,
        title: str = "Occupancy Grid Map",
    ) -> plt.Axes:
        """Plot the occupancy grid map using matplotlib.

        Args:
            ax: Optional matplotlib axes to draw on.
            title: Title text for the plot.

        Returns:
            Matplotlib axes with the drawn map.
        """
        if ax is None:
            _, ax = plt.subplots(figsize=(7, 7))

        prob_map = self.get_probability_map()
        extent = [self.min_x, self.max_x, self.min_y, self.max_y]

        image_handle = ax.imshow(
            prob_map,
            cmap="gray_r",
            origin="lower",
            extent=extent,
            vmin=0.0,
            vmax=1.0,
        )
        ax.set_title(title)
        ax.set_xlabel("X (m)")
        ax.set_ylabel("Y (m)")
        ax.grid(True, linestyle=":", alpha=0.5)

        plt.colorbar(image_handle, ax=ax, label="Occupancy Probability")
        return ax
