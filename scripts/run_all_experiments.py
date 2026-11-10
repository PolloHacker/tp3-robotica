"""Script to execute all required experimental evaluations and save result figures."""

from __future__ import annotations

import os
import time
from coppeliasim_zmqremoteapi_client import RemoteAPIClient
import matplotlib.pyplot as plt
import numpy as np

from tp3.navigator import ReactiveNavigator
from tp3.occupancy_grid import OccupancyGrid
from tp3.robot import KobukiRobot
from tp3.simulation import load_scene, plot_exploration_results


def run_multi_resolution_experiment(
    sim: RemoteAPIClient,
    scene_path: str,
    duration_seconds: float = 50.0,
    results_dir: str = "results",
) -> None:
    """Evaluate mapping with cell sizes 0.01m, 0.10m, and 0.50m simultaneously.

    Args:
        sim: CoppeliaSim sim object.
        scene_path: Scene path to static scene.
        duration_seconds: Run duration.
        results_dir: Output directory.
    """
    print("=== Running Experiment 1: Cell Size Evaluation (0.01m, 0.10m, 0.50m) ===")
    load_scene(sim, scene_path)
    time.sleep(0.5)

    robot = KobukiRobot(sim=sim, distance_noise_std=0.02, angle_noise_std=0.005)
    grid_001 = OccupancyGrid(cell_size=0.01)
    grid_010 = OccupancyGrid(cell_size=0.10)
    grid_050 = OccupancyGrid(cell_size=0.50)
    navigator = ReactiveNavigator()

    sim.startSimulation()
    time.sleep(0.5)

    trajectory: list[tuple[float, float]] = []
    hits_x: list[float] = []
    hits_y: list[float] = []

    start_time = time.time()
    try:
        while time.time() - start_time < duration_seconds:
            rx, ry, ryaw = robot.get_pose()
            trajectory.append((rx, ry))

            ranges, angles = robot.read_laser_scan()
            if ranges is not None and angles is not None:
                grid_001.update(rx, ry, ryaw, ranges, angles, beam_step=3)
                grid_010.update(rx, ry, ryaw, ranges, angles, beam_step=2)
                grid_050.update(rx, ry, ryaw, ranges, angles, beam_step=2)

                px, py, mask = robot.get_laser_points_in_world_frame(
                    ranges, angles, rx, ry, ryaw
                )
                if np.any(mask):
                    hits_x.extend(px[mask].tolist())
                    hits_y.extend(py[mask].tolist())

                v, omega = navigator.compute_commands(rx, ry, ranges, angles)
                robot.set_velocities(v, omega)

            time.sleep(0.04)
    finally:
        robot.stop()
        sim.stopSimulation()
        while sim.getSimulationState() != sim.simulation_stopped:
            time.sleep(0.05)

    os.makedirs(results_dir, exist_ok=True)

    grid_001.save_map_image(os.path.join(results_dir, "exp1_grid_cell_0.01.png"))
    grid_010.save_map_image(os.path.join(results_dir, "exp1_grid_cell_0.10.png"))
    grid_050.save_map_image(os.path.join(results_dir, "exp1_grid_cell_0.50.png"))

    fig, axes = plt.subplots(1, 3, figsize=(18, 5.5))
    grid_001.plot_map(ax=axes[0], title="Cell size = 0.01 m (Fine)")
    grid_010.plot_map(ax=axes[1], title="Cell size = 0.10 m (Recommended)")
    grid_050.plot_map(ax=axes[2], title="Cell size = 0.50 m (Coarse)")
    plt.tight_layout()
    fig.savefig(
        os.path.join(results_dir, "exp1_cell_size_comparison.png"),
        dpi=200,
        bbox_inches="tight",
    )
    plt.close(fig)

    fig_full = plot_exploration_results(
        grid=grid_010,
        trajectory=trajectory,
        hit_points_x=np.array(hits_x),
        hit_points_y=np.array(hits_y),
        output_image_path=os.path.join(results_dir, "exp1_static_overview.png"),
        title_suffix="(Static Scenario)",
    )
    plt.close(fig_full)
    print("Experiment 1 completed and figures saved.")


def run_single_experiment(
    sim: RemoteAPIClient,
    scene_path: str,
    initial_pose: tuple[float, float, float],
    duration_seconds: float,
    cell_size: float,
    output_prefix: str,
    title_label: str,
    results_dir: str = "results",
) -> None:
    """Run a single exploration experiment with designated initial pose.

    Args:
        sim: CoppeliaSim sim object.
        scene_path: Scene path to load.
        initial_pose: Tuple of (x, y, yaw).
        duration_seconds: Total simulation time in seconds.
        cell_size: Grid cell size in meters.
        output_prefix: Filename prefix for outputs.
        title_label: Descriptive label for plot headers.
        results_dir: Output directory.
    """
    print(f"=== Running: {title_label} ===")
    load_scene(sim, scene_path)
    time.sleep(0.5)

    robot = KobukiRobot(sim=sim, distance_noise_std=0.02, angle_noise_std=0.005)
    robot.set_pose(initial_pose[0], initial_pose[1], initial_pose[2])

    grid = OccupancyGrid(cell_size=cell_size)
    navigator = ReactiveNavigator()

    sim.startSimulation()
    time.sleep(0.5)

    trajectory: list[tuple[float, float]] = []
    hits_x: list[float] = []
    hits_y: list[float] = []

    start_time = time.time()
    try:
        while time.time() - start_time < duration_seconds:
            rx, ry, ryaw = robot.get_pose()
            trajectory.append((rx, ry))

            ranges, angles = robot.read_laser_scan()
            if ranges is not None and angles is not None:
                grid.update(rx, ry, ryaw, ranges, angles)

                px, py, mask = robot.get_laser_points_in_world_frame(
                    ranges, angles, rx, ry, ryaw
                )
                if np.any(mask):
                    hits_x.extend(px[mask].tolist())
                    hits_y.extend(py[mask].tolist())

                v, omega = navigator.compute_commands(rx, ry, ranges, angles)
                robot.set_velocities(v, omega)

            time.sleep(0.04)
    finally:
        robot.stop()
        sim.stopSimulation()
        while sim.getSimulationState() != sim.simulation_stopped:
            time.sleep(0.05)

    os.makedirs(results_dir, exist_ok=True)
    grid.save_map_image(os.path.join(results_dir, f"{output_prefix}_grid.png"))

    fig = plot_exploration_results(
        grid=grid,
        trajectory=trajectory,
        hit_points_x=np.array(hits_x),
        hit_points_y=np.array(hits_y),
        output_image_path=os.path.join(results_dir, f"{output_prefix}_overview.png"),
        title_suffix=f"({title_label})",
    )
    plt.close(fig)
    print(f"Completed {title_label}.")


def run_noise_comparison_experiment(
    sim: RemoteAPIClient,
    scene_path: str,
    duration_seconds: float = 40.0,
    results_dir: str = "results",
) -> None:
    """Evaluate mapping quality across different sensor noise levels.

    Args:
        sim: CoppeliaSim sim object.
        scene_path: Static scene path.
        duration_seconds: Duration for test.
        results_dir: Output directory.
    """
    print("=== Running Experiment: Sensor Noise Comparison ===")
    noise_levels = [
        ("Low Noise (1cm)", 0.01, 0.002),
        ("Medium Noise (5cm)", 0.05, 0.010),
        ("High Noise (15cm)", 0.15, 0.030),
    ]

    grids: list[OccupancyGrid] = []

    for label, d_noise, a_noise in noise_levels:
        load_scene(sim, scene_path)
        time.sleep(0.5)

        robot = KobukiRobot(
            sim=sim, distance_noise_std=d_noise, angle_noise_std=a_noise
        )
        grid = OccupancyGrid(cell_size=0.10)
        navigator = ReactiveNavigator()

        sim.startSimulation()
        time.sleep(0.5)

        start_time = time.time()
        try:
            while time.time() - start_time < duration_seconds:
                rx, ry, ryaw = robot.get_pose()
                ranges, angles = robot.read_laser_scan()
                if ranges is not None and angles is not None:
                    grid.update(rx, ry, ryaw, ranges, angles)
                    v, omega = navigator.compute_commands(rx, ry, ranges, angles)
                    robot.set_velocities(v, omega)
                time.sleep(0.04)
        finally:
            robot.stop()
            sim.stopSimulation()
            while sim.getSimulationState() != sim.simulation_stopped:
                time.sleep(0.05)

        grids.append(grid)

    fig, axes = plt.subplots(1, 3, figsize=(18, 5.5))
    for i, (label, _, _) in enumerate(noise_levels):
        grids[i].plot_map(ax=axes[i], title=label)
    plt.tight_layout()
    fig.savefig(
        os.path.join(results_dir, "exp_noise_comparison.png"),
        dpi=200,
        bbox_inches="tight",
    )
    plt.close(fig)
    print("Sensor noise comparison experiment completed.")


def main() -> None:
    """Execute all experiments in sequence."""
    client = RemoteAPIClient()
    sim = client.require("sim")

    static_scene = "cenas/cena-tp3-estatico.ttt"
    dynamic_scene = "cenas/cena-tp3-dinamico.ttt"

    run_multi_resolution_experiment(sim, static_scene, duration_seconds=50.0)

    run_single_experiment(
        sim=sim,
        scene_path=static_scene,
        initial_pose=(-4.0, -4.0, 0.0),
        duration_seconds=50.0,
        cell_size=0.10,
        output_prefix="exp2_static_pos1",
        title_label="Static Scenario - Pose 1 (-4, -4)",
    )

    run_single_experiment(
        sim=sim,
        scene_path=static_scene,
        initial_pose=(3.0, 3.0, np.pi),
        duration_seconds=50.0,
        cell_size=0.10,
        output_prefix="exp3_static_pos2",
        title_label="Static Scenario - Pose 2 (3, 3)",
    )

    run_single_experiment(
        sim=sim,
        scene_path=dynamic_scene,
        initial_pose=(-4.0, -4.0, 0.0),
        duration_seconds=50.0,
        cell_size=0.10,
        output_prefix="exp4_dynamic_pos1",
        title_label="Dynamic Scenario - Pose 1 (-4, -4)",
    )

    run_single_experiment(
        sim=sim,
        scene_path=dynamic_scene,
        initial_pose=(3.0, 3.0, np.pi),
        duration_seconds=50.0,
        cell_size=0.10,
        output_prefix="exp5_dynamic_pos2",
        title_label="Dynamic Scenario - Pose 2 (3, 3)",
    )

    run_noise_comparison_experiment(sim, static_scene, duration_seconds=35.0)

    print("\nAll experiments successfully completed! All results saved in results/.")


if __name__ == "__main__":
    main()
