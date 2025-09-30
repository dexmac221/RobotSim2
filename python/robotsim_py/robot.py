from __future__ import annotations

from dataclasses import dataclass
from typing import Iterable, List, Sequence, Tuple

import numpy as np


@dataclass(frozen=True)
class DHParam:
    """Single Denavit–Hartenberg parameter tuple."""

    a: float
    alpha: float
    d: float
    theta_offset: float = 0.0


def dh_transform(a: float, alpha: float, d: float, theta: float) -> np.ndarray:
    """Compute the homogeneous transform for one DH link."""

    sa, ca = np.sin(alpha), np.cos(alpha)
    st, ct = np.sin(theta), np.cos(theta)

    return np.array(
        [
            [ct, -st * ca, st * sa, a * ct],
            [st, ct * ca, -ct * sa, a * st],
            [0.0, sa, ca, d],
            [0.0, 0.0, 0.0, 1.0],
        ],
        dtype=float,
    )


class RobotArm:
    """6-DOF articulated arm with DH-based kinematics."""

    def __init__(
        self,
        dh_params: Sequence[DHParam],
        joint_limits: Sequence[Tuple[float, float]] | None = None,
    ) -> None:
        self.dh_params: Tuple[DHParam, ...] = tuple(dh_params)
        self.dofs = len(self.dh_params)

        if joint_limits is None:
            joint_limits = [(-np.pi, np.pi)] * self.dofs
        if len(joint_limits) != self.dofs:
            raise ValueError("joint_limits must match number of joints")
        self.joint_limits = np.asarray(joint_limits, dtype=float)

        link_lengths = [abs(p.a) for p in self.dh_params]
        link_offsets = [abs(p.d) for p in self.dh_params]
        self.max_reach = float(sum(link_lengths) + sum(link_offsets))

    # ------------------------------------------------------------------
    # Factory helpers
    # ------------------------------------------------------------------
    @classmethod
    def default_puma(cls) -> "RobotArm":
        """Instantiate a PUMA-560 inspired geometry."""

        dh = (
            DHParam(a=0.0, alpha=-np.pi / 2.0, d=0.6718),
            DHParam(a=0.4318, alpha=0.0, d=0.0),
            DHParam(a=0.0203, alpha=np.pi / 2.0, d=0.0),
            DHParam(a=0.0, alpha=-np.pi / 2.0, d=0.4331),
            DHParam(a=0.0, alpha=np.pi / 2.0, d=0.0),
            DHParam(a=0.0, alpha=0.0, d=0.0558),
        )

        joint_limits = (
            (-np.pi, np.pi),
            (-120.0 * np.pi / 180.0, 120.0 * np.pi / 180.0),
            (-155.0 * np.pi / 180.0, 155.0 * np.pi / 180.0),
            (-np.pi, np.pi),
            (-150.0 * np.pi / 180.0, 150.0 * np.pi / 180.0),
            (-np.pi, np.pi),
        )

        return cls(dh, joint_limits)

    # ------------------------------------------------------------------
    # Core kinematics
    # ------------------------------------------------------------------
    def home_configuration(self) -> np.ndarray:
        return np.zeros(self.dofs, dtype=float)

    def transforms(self, joint_angles: Sequence[float]) -> List[np.ndarray]:
        """Return cumulative transforms from base to each joint/end-effector."""

        if len(joint_angles) != self.dofs:
            raise ValueError("joint_angles must have length %d" % self.dofs)

        T = np.eye(4, dtype=float)
        frames: List[np.ndarray] = [T]
        for angle, param in zip(joint_angles, self.dh_params):
            T = T @ dh_transform(
                param.a, param.alpha, param.d, angle + param.theta_offset
            )
            frames.append(T)
        return frames

    def joint_positions(self, joint_angles: Sequence[float]) -> np.ndarray:
        """Return (N+1, 3) array of joint positions including the base."""

        frames = self.transforms(joint_angles)
        return np.vstack([frame[:3, 3] for frame in frames])

    def end_effector_pose(self, joint_angles: Sequence[float]) -> np.ndarray:
        """Homogeneous transform of the end effector."""

        return self.transforms(joint_angles)[-1]

    def end_effector(self, joint_angles: Sequence[float]) -> np.ndarray:
        return self.end_effector_pose(joint_angles)[:3, 3]

    def jacobian(self, joint_angles: Sequence[float]) -> np.ndarray:
        """Geometric Jacobian for end-effector position."""

        frames = self.transforms(joint_angles)
        end_pos = frames[-1][:3, 3]
        J = np.zeros((3, self.dofs), dtype=float)
        for idx in range(self.dofs):
            z_axis = frames[idx][:3, 2]
            origin = frames[idx][:3, 3]
            J[:, idx] = np.cross(z_axis, end_pos - origin)
        return J

    # ------------------------------------------------------------------
    # Utilities
    # ------------------------------------------------------------------
    def clamp(self, joint_angles: Sequence[float]) -> np.ndarray:
        angles = np.asarray(joint_angles, dtype=float)
        return np.clip(angles, self.joint_limits[:, 0], self.joint_limits[:, 1])

    def within_limits(self, joint_angles: Sequence[float]) -> bool:
        angles = np.asarray(joint_angles, dtype=float)
        return np.all(angles >= self.joint_limits[:, 0]) and np.all(
            angles <= self.joint_limits[:, 1]
        )

    def project_target(self, target: Sequence[float]) -> np.ndarray:
        """Pull a target onto the reachable sphere if needed."""

        target_vec = np.asarray(target, dtype=float)
        norm = np.linalg.norm(target_vec)
        if norm <= self.max_reach:
            return target_vec
        if norm == 0:
            return np.array([0.0, 0.0, self.max_reach], dtype=float)
        return target_vec / norm * self.max_reach

    def random_reachable_target(self, rng: np.random.Generator | None = None) -> np.ndarray:
        rng = rng or np.random.default_rng()
        for _ in range(1000):
            candidate = rng.normal(size=3)
            candidate = candidate / np.linalg.norm(candidate)
            radius = rng.uniform(0.2 * self.max_reach, 0.9 * self.max_reach)
            point = candidate * radius
            if point[2] < 0:
                point[2] = abs(point[2])  # keep above the ground plane
            return point
        raise RuntimeError("Failed to sample reachable target")

    def finite_difference_jacobian(
        self, joint_angles: Sequence[float], epsilon: float = 1e-6
    ) -> np.ndarray:
        joint_angles = np.asarray(joint_angles, dtype=float)
        base = self.end_effector(joint_angles)
        J = np.zeros((3, self.dofs), dtype=float)
        for idx in range(self.dofs):
            perturbed = joint_angles.copy()
            perturbed[idx] += epsilon
            forward = self.end_effector(perturbed)
            J[:, idx] = (forward - base) / epsilon
        return J

    def iter_inverse_step(
        self,
        joint_angles: np.ndarray,
        target: Sequence[float],
        step_scale: float = 1.0,
        damping: float = 1e-4,
        max_step: float = 0.15,
    ) -> Tuple[np.ndarray, np.ndarray]:
        """Compute a single Jacobian-based update toward a target."""

        target_vec = np.asarray(target, dtype=float)
        current_end = self.end_effector(joint_angles)
        error = target_vec - current_end
        J = self.jacobian(joint_angles)
        # Damped least squares for stability
        JT = J.T
        identity = np.eye(J.shape[0])
        dls = JT @ np.linalg.inv(J @ JT + damping * identity)
        delta_q = dls @ (error * step_scale)
        norm = np.linalg.norm(delta_q)
        if norm > max_step:
            delta_q = delta_q / norm * max_step
        new_angles = self.clamp(joint_angles + delta_q)
        return new_angles, error
