"""CLI entrypoint for the modern RobotSim demo."""

from __future__ import annotations

import argparse
import json
import time
from pathlib import Path

from .controllers import TrajectoryController
from .robot import Pose, RobotModel
from .trajectory import Trajectory
from .viewer import MatplotlibArmViewer


def _demo_trajectory() -> Trajectory:
    start = Pose.from_xyz_rpy((0.45, 0.0, 0.45), (0.0, 0.0, 0.0))
    mid = Pose.from_xyz_rpy((0.35, 0.25, 0.50), (0.0, 15.0, 0.0))
    end = Pose.from_xyz_rpy((0.40, -0.25, 0.40), (0.0, -15.0, 45.0))
    first_leg = Trajectory.linear_path(start, mid, samples=10, duration=2.5).keyframes
    second_leg = Trajectory.linear_path(mid, end, samples=10, duration=2.5).keyframes
    keyframes = list(first_leg)
    keyframes += [(t + 2.5, pose) for t, pose in second_leg[1:]]
    return Trajectory(keyframes)


def run_demo(
    duration: float,
    dt: float,
    headless: bool,
    log_path: Path | None,
    joint_count: int | None,
) -> None:
    robot = RobotModel.default(joint_count)
    trajectory = _demo_trajectory()
    controller = TrajectoryController(robot, trajectory)

    viewer = None if headless else MatplotlibArmViewer.create()
    log: list[dict[str, float]] = []

    steps = max(1, int(duration / dt))
    for step in range(steps):
        t = (step / steps) * trajectory.duration()
        state = controller.step(t)
        pose = robot.forward_kinematics(state.joint_angles)
        if viewer:
            viewer.update(robot.all_joint_positions(state.joint_angles))
        log.append(
            {
                "time": t,
                "px": float(pose.position[0]),
                "py": float(pose.position[1]),
                "pz": float(pose.position[2]),
            }
        )
        time.sleep(dt)
    if viewer:
        viewer.show()
    if log_path:
        log_path.write_text(json.dumps(log, indent=2))


def main(argv: list[str] | None = None) -> None:
    parser = argparse.ArgumentParser(description="RobotSim Python demo")
    parser.add_argument("--duration", type=float, default=5.0, help="Simulation duration [s]")
    parser.add_argument("--dt", type=float, default=0.05, help="Control loop period [s]")
    parser.add_argument("--headless", action="store_true", help="Run without graphical viewer")
    parser.add_argument("--log", type=Path, help="Optional JSON log output path")
    parser.add_argument(
        "--joint-count",
        type=int,
        default=None,
        help="Number of robot joints to simulate (1-6). Default matches the base arm.",
    )
    args = parser.parse_args(argv)

    run_demo(args.duration, args.dt, args.headless, args.log, args.joint_count)


if __name__ == "__main__":
    main()
