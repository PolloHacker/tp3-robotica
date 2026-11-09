"""Simulation orchestrator for robot exploration and occupancy grid mapping."""

from __future__ import annotations

import time
from typing import Any

import matplotlib.pyplot as plt
import numpy as np

from tp3.navigator import ReactiveNavigator
from tp3.occupancy_grid import OccupancyGrid
from tp3.robot import KobukiRobot


def run_exploration(
    sim: Any,
    scene_path: str = "/workspace/cenas/cena-tp3-estatico.ttt",
    initial_pose: tuple[float, float, float] | None = None,
    duration_seconds: float = 60.0,
    cell_size: float = 0.10,
    distance_noise_std: float = 0.02,
    angle_noise_std: float = 0.005,
    step_sleep: float = 0.04,
) -> tuple[OccupancyGrid, list[tuple[float, float]], np.ndarray, np.ndarray]:
    """Execute autonomous exploration and build an occupancy grid map.

    Args:
        sim: CoppeliaSim Remote API sim object.
        scene_path: Path to the CoppeliaSim .ttt scene file.
        initial_pose: Optional starting pose (x, y, yaw) in meters and radians.
        duration_seconds: Total duration to run the exploration simulation.
        cell_size: Side length of each grid cell in meters.
        distance_noise_std: Gaussian noise standard deviation for range readings.
        angle_noise_std: Gaussian noise standard deviation for angle readings.
        step_sleep: Pause duration between iterations in seconds.

    Returns:
        Tuple of (occupancy_grid, trajectory, hit_points_x, hit_points_y).
    """
    sim.loadScene(scene_path)
    time.sleep(0.5)

    robot = KobukiRobot(
        sim=sim,
        distance_noise_std=distance_noise_std,
        angle_noise_std=angle_noise_std,
    )

    if initial_pose is not None:
        robot.set_pose(initial_pose[0], initial_pose[1], initial_pose[2])

    grid = OccupancyGrid(
        min_x=-5.5,
        max_x=5.5,
        min_y=-5.5,
        max_y=5.5,
        cell_size=cell_size,
    )
    navigator = ReactiveNavigator()

    sim.startSimulation()
    time.sleep(0.5)

    trajectory: list[tuple[float, float]] = []
    all_hits_x: list[float] = []
    all_hits_y: list[float] = []

    start_time = time.time()
    try:
        while time.time() - start_time < duration_seconds:
            robot_x, robot_y, robot_yaw = robot.get_pose()
            trajectory.append((robot_x, robot_y))

            ranges, angles = robot.read_laser_scan()
            if ranges is not None and angles is not None:
                grid.update(robot_x, robot_y, robot_yaw, ranges, angles)

                pts_x, pts_y, hit_mask = robot.get_laser_points_in_world_frame(
                    ranges, angles, robot_x, robot_y, robot_yaw
                )
                if np.any(hit_mask):
                    all_hits_x.extend(pts_x[hit_mask].tolist())
                    all_hits_y.extend(pts_y[hit_mask].tolist())

                v_cmd, omega_cmd = navigator.compute_commands(
                    robot_x, robot_y, ranges, angles
                )
                robot.set_velocities(v_cmd, omega_cmd)

            time.sleep(step_sleep)
    finally:
        robot.stop()
        sim.stopSimulation()
        while sim.getSimulationState() != sim.simulation_stopped:
            time.sleep(0.05)

    return (
        grid,
        trajectory,
        np.array(all_hits_x, dtype=np.float32),
        np.array(all_hits_y, dtype=np.float32),
    )


def plot_exploration_results(
    grid: OccupancyGrid,
    trajectory: list[tuple[float, float]],
    hit_points_x: np.ndarray,
    hit_points_y: np.ndarray,
    output_image_path: str | None = None,
    title_suffix: str = "",
) -> plt.Figure:
    """Create a two-panel figure comparing raw laser hit points with the occupancy grid.

    Args:
        grid: Populated OccupancyGrid object.
        trajectory: List of robot positions (x, y) along the path.
        hit_points_x: Array of x coordinates of obstacle hits in world frame.
        hit_points_y: Array of y coordinates of obstacle hits in world frame.
        output_image_path: Optional path to save the generated figure.
        title_suffix: Optional label to append to plot titles.

    Returns:
        Matplotlib figure instance.
    """
    fig, (ax_hits, ax_grid) = plt.subplots(1, 2, figsize=(14, 6))

    if len(hit_points_x) > 0:
        subsample_step = max(1, len(hit_points_x) // 10000)
        ax_hits.scatter(
            hit_points_x[::subsample_step],
            hit_points_y[::subsample_step],
            s=2,
            c="red",
            alpha=0.3,
            label="Laser Hits",
        )

    if trajectory:
        traj_array = np.array(trajectory)
        ax_hits.plot(
            traj_array[:, 0],
            traj_array[:, 1],
            color="blue",
            linewidth=1.8,
            label="Robot Trajectory",
        )
        ax_hits.plot(
            traj_array[0, 0],
            traj_array[0, 1],
            marker="o",
            markersize=8,
            color="green",
            label="Start",
        )
        ax_hits.plot(
            traj_array[-1, 0],
            traj_array[-1, 1],
            marker="s",
            markersize=8,
            color="black",
            label="End",
        )

    ax_hits.set_title(f"Robot Trajectory and Laser Hits {title_suffix}".strip())
    ax_hits.set_xlabel("X (m)")
    ax_hits.set_ylabel("Y (m)")
    ax_hits.set_xlim(-5.5, 5.5)
    ax_hits.set_ylim(-5.5, 5.5)
    ax_hits.set_aspect("equal")
    ax_hits.grid(True, linestyle=":", alpha=0.5)
    ax_hits.legend(loc="upper right")

    grid.plot_map(
        ax=ax_grid,
        title=f"Occupancy Grid Map (cell={grid.cell_size}m) {title_suffix}".strip(),
    )
    ax_grid.set_aspect("equal")

    if trajectory:
        traj_array = np.array(trajectory)
        ax_grid.plot(
            traj_array[:, 0],
            traj_array[:, 1],
            color="cyan",
            linewidth=1.2,
            alpha=0.8,
        )

    plt.tight_layout()

    if output_image_path:
        fig.savefig(output_image_path, dpi=200, bbox_inches="tight")

    return fig
