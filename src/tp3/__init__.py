"""TP3 - Robotic Exploration and Occupancy Grid Mapping."""

from __future__ import annotations

import os
import argparse
from coppeliasim_zmqremoteapi_client import RemoteAPIClient

from tp3.navigator import ReactiveNavigator
from tp3.occupancy_grid import OccupancyGrid
from tp3.robot import KobukiRobot
from tp3.simulation import (
    get_candidate_scene_paths,
    load_scene,
    plot_exploration_results,
    run_exploration,
)

__all__ = [
    "KobukiRobot",
    "OccupancyGrid",
    "ReactiveNavigator",
    "run_exploration",
    "plot_exploration_results",
    "load_scene",
    "get_candidate_scene_paths",
    "main",
]


def main() -> None:
    """Run an exploration experiment from the command line."""
    parser = argparse.ArgumentParser(
        description="Run Kobuki exploration and occupancy grid mapping in CoppeliaSim."
    )
    parser.add_argument(
        "--scene",
        type=str,
        default="cenas/cena-tp3-estatico.ttt",
        help="Path to CoppeliaSim scene file (default: cenas/cena-tp3-estatico.ttt)",
    )
    parser.add_argument(
        "--duration",
        type=float,
        default=45.0,
        help="Simulation exploration duration in seconds (default: 45.0)",
    )
    parser.add_argument(
        "--cell-size",
        type=float,
        default=0.10,
        help="Occupancy grid cell resolution in meters (default: 0.10)",
    )
    parser.add_argument(
        "--output",
        type=str,
        default="results/cli_demo_result.png",
        help="Output image file path (default: results/cli_demo_result.png)",
    )

    args = parser.parse_args()

    print(f"Connecting to CoppeliaSim...")
    client = RemoteAPIClient()
    sim = client.require("sim")

    print(f"Starting exploration ({args.duration}s, cell={args.cell_size}m)...")
    grid, traj, hits_x, hits_y = run_exploration(
        sim=sim,
        scene_path=args.scene,
        duration_seconds=args.duration,
        cell_size=args.cell_size,
    )

    print(f"Simulation completed. Saving results to {args.output}...")
    import os
    os.makedirs(os.path.dirname(args.output) or ".", exist_ok=True)
    plot_exploration_results(
        grid=grid,
        trajectory=traj,
        hit_points_x=hits_x,
        hit_points_y=hits_y,
        output_image_path=args.output,
    )
    print("Done!")


if __name__ == "__main__":
    main()
