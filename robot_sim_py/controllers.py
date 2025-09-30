"""High-level controllers for the robot."""

from __future__ import annotations

import logging
from dataclasses import dataclass
from typing import Callable, Iterable, Optional

import numpy as np

from .robot import Pose, RobotModel, RobotState
from .trajectory import Trajectory


@dataclass
class ManualController:
    """Simple proportional controller for manual joint manipulation."""

    robot: RobotModel
    state: RobotState

    def step(self, joint_velocities: Iterable[float], dt: float) -> RobotState:
        velocities = np.asarray(list(joint_velocities), dtype=float)
        dof = self.robot.dof
        if velocities.shape != (dof,):
            raise ValueError(f"joint_velocities must contain {dof} values")
        self.state.joint_angles = self.state.joint_angles + velocities * dt
        self.state = self.robot.clamp(self.state)
        return self.state


class TrajectoryController:
    """Tracks a cartesian trajectory using damped-least-squares IK."""

    def __init__(
        self,
        robot: RobotModel,
        trajectory: Trajectory,
        gain: float = 0.8,
        damping: float = 1e-2,
        max_iters: int = 80,
    ) -> None:
        self.robot = robot
        self.trajectory = trajectory
        self.gain = gain
        self.damping = damping
        self.max_iters = max_iters
        self.state = RobotState.zeros(self.robot.dof)
        self.on_pose_reached: Callable[[Pose], None] | None = None
        self.last_error: Optional[str] = None

    def step(self, t: float) -> RobotState:
        target_pose = self.trajectory.pose_at(t)
        self.last_error = None
        try:
            self.state = self.robot.inverse_kinematics(
                target_pose,
                initial=self.state,
                tolerance=1e-3,
                damping=self.damping,
                max_iters=self.max_iters,
            )
            if self.on_pose_reached:
                self.on_pose_reached(target_pose)
        except RuntimeError as exc:
            self.last_error = f"IK failed (t={t:.2f}s): {exc}"
        return self.state
