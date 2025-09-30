from __future__ import annotations

from dataclasses import dataclass
from typing import Dict

import numpy as np

from .robot import RobotArm


@dataclass
class ControllerState:
    mode: str
    selected_joint: int
    joint_step_deg: float
    ik_gain: float
    error_norm: float
    target: np.ndarray


class RobotController:
    """Application-level state machine for manual and automatic modes."""

    def __init__(self, robot: RobotArm) -> None:
        self.robot = robot
        self.joint_angles = robot.home_configuration()
        self.mode = "manual"
        self.selected_joint = 0
        self.joint_step = np.deg2rad(2.0)
        self.ik_gain = 0.8
        self.target = robot.project_target(np.array([0.6, 0.2, 0.6]))
        self.error_norm = self._compute_error_norm()

    # ------------------------------------------------------------------
    # High-level actions
    # ------------------------------------------------------------------
    def reset(self) -> None:
        self.joint_angles = self.robot.home_configuration()
        self.error_norm = self._compute_error_norm()

    def set_mode(self, mode: str) -> None:
        if mode not in {"manual", "auto"}:
            raise ValueError("mode must be 'manual' or 'auto'")
        self.mode = mode

    def toggle_mode(self) -> None:
        self.mode = "auto" if self.mode == "manual" else "manual"

    def cycle_joint(self, direction: int) -> None:
        self.selected_joint = (self.selected_joint + direction) % self.robot.dofs

    def nudge_selected_joint(self, direction: int) -> None:
        if self.mode != "manual":
            return
        delta = direction * self.joint_step
        updated = self.joint_angles.copy()
        updated[self.selected_joint] += delta
        self.joint_angles = self.robot.clamp(updated)
        self.error_norm = self._compute_error_norm()

    def randomize_target(self) -> None:
        self.target = self.robot.random_reachable_target()
        self.error_norm = self._compute_error_norm()

    def set_target(self, target: np.ndarray) -> None:
        self.target = self.robot.project_target(target)
        self.error_norm = self._compute_error_norm()

    def adjust_joint_step(self, factor: float) -> None:
        self.joint_step = float(np.clip(self.joint_step * factor, np.deg2rad(0.1), np.deg2rad(15.0)))

    def adjust_ik_gain(self, factor: float) -> None:
        self.ik_gain = float(np.clip(self.ik_gain * factor, 0.1, 3.0))

    # ------------------------------------------------------------------
    # Simulation loop
    # ------------------------------------------------------------------
    def update(self, dt: float) -> None:
        if self.mode == "auto":
            step_scale = self.ik_gain * dt
            self.joint_angles, error = self.robot.iter_inverse_step(
                self.joint_angles, self.target, step_scale=step_scale
            )
            self.error_norm = float(np.linalg.norm(error))
        else:
            self.error_norm = self._compute_error_norm()

    # ------------------------------------------------------------------
    # Diagnostics
    # ------------------------------------------------------------------
    def state(self) -> ControllerState:
        return ControllerState(
            mode=self.mode,
            selected_joint=self.selected_joint,
            joint_step_deg=float(np.rad2deg(self.joint_step)),
            ik_gain=self.ik_gain,
            error_norm=self.error_norm,
            target=self.target.copy(),
        )

    def status_text(self) -> Dict[str, str]:
        return {
            "Mode": self.mode.upper(),
            "Joint": f"J{self.selected_joint + 1}",
            "Step": f"{np.rad2deg(self.joint_step):.1f}°",
            "IK gain": f"{self.ik_gain:.2f}",
            "Error": f"{self.error_norm:.3f} m",
        }

    # ------------------------------------------------------------------
    def _compute_error_norm(self) -> float:
        effector = self.robot.end_effector(self.joint_angles)
        return float(np.linalg.norm(self.target - effector))
