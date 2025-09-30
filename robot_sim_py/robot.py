"""Robot model and kinematics helpers for the modern RobotSim port."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Iterable, Sequence

import numpy as np


def _rot_x(theta: float) -> np.ndarray:
    c, s = np.cos(theta), np.sin(theta)
    return np.array(
        [[1.0, 0.0, 0.0, 0.0], [0.0, c, -s, 0.0], [0.0, s, c, 0.0], [0.0, 0.0, 0.0, 1.0]]
    )


def _rot_y(theta: float) -> np.ndarray:
    c, s = np.cos(theta), np.sin(theta)
    return np.array(
        [[c, 0.0, s, 0.0], [0.0, 1.0, 0.0, 0.0], [-s, 0.0, c, 0.0], [0.0, 0.0, 0.0, 1.0]]
    )


def _rot_z(theta: float) -> np.ndarray:
    c, s = np.cos(theta), np.sin(theta)
    return np.array(
        [[c, -s, 0.0, 0.0], [s, c, 0.0, 0.0], [0.0, 0.0, 1.0, 0.0], [0.0, 0.0, 0.0, 1.0]]
    )


def _translate(vec: Sequence[float]) -> np.ndarray:
    tx, ty, tz = vec
    return np.array(
        [[1.0, 0.0, 0.0, tx], [0.0, 1.0, 0.0, ty], [0.0, 0.0, 1.0, tz], [0.0, 0.0, 0.0, 1.0]]
    )


def rotation_matrix_to_vector(rot: np.ndarray) -> np.ndarray:
    """Convert a rotation matrix into the equivalent axis-angle vector."""
    trace = np.trace(rot)
    cos_angle = (trace - 1.0) / 2.0
    cos_angle = np.clip(cos_angle, -1.0, 1.0)
    angle = np.arccos(cos_angle)
    if np.isclose(angle, 0.0):
        return np.zeros(3)

    sin_angle = np.sin(angle)
    if np.isclose(sin_angle, 0.0):
        # fallback for angle close to pi
        axis = np.sqrt(np.maximum(np.diagonal(rot) + 1.0, 1e-9))
        axis = axis / np.linalg.norm(axis)
    else:
        axis = np.array(
            [
                rot[2, 1] - rot[1, 2],
                rot[0, 2] - rot[2, 0],
                rot[1, 0] - rot[0, 1],
            ]
        ) / (2.0 * sin_angle)

    return angle * axis


@dataclass
class Pose:
    position: np.ndarray
    rotation: np.ndarray

    @classmethod
    def from_xyz_rpy(cls, position: Sequence[float], rpy_degrees: Sequence[float]) -> "Pose":
        x, y, z = position
        roll, pitch, yaw = np.deg2rad(rpy_degrees)
        rot = _rot_z(yaw) @ _rot_y(pitch) @ _rot_x(roll)
        return cls(np.array([x, y, z], dtype=float), rot[:3, :3])

    def as_matrix(self) -> np.ndarray:
        mat = np.eye(4)
        mat[:3, :3] = self.rotation
        mat[:3, 3] = self.position
        return mat


@dataclass
class RobotState:
    joint_angles: np.ndarray

    def copy(self) -> "RobotState":
        return RobotState(self.joint_angles.copy())

    @classmethod
    def zeros(cls, dof: int = 6) -> "RobotState":
        return cls(np.zeros(int(dof), dtype=float))


_BASE_AXES: tuple[str, ...] = ("z", "y", "y", "x", "y", "x")
_BASE_TRANSLATION_DIRECTIONS: tuple[tuple[float, float, float], ...] = (
    (0.0, 0.0, 1.0),
    (1.0, 0.0, 0.0),
    (1.0, 0.0, 0.0),
    (0.0, 0.0, 1.0),
    (0.0, 0.0, 1.0),
    (1.0, 0.0, 0.0),
)
_BASE_LINK_LENGTHS: tuple[float, ...] = (0.32, 0.38, 0.24, 0.18, 0.14, 0.08)
_BASE_JOINT_LIMITS: tuple[tuple[float, float], ...] = (
    (-np.pi, np.pi),
    (-np.pi / 2, np.pi / 2),
    (-np.pi / 2, np.pi / 2),
    (-np.pi, np.pi),
    (-np.pi, np.pi),
    (-np.pi, np.pi),
)


class RobotModel:
    """Analytic robot description providing forward/inverse kinematics."""

    def __init__(
        self,
        link_lengths: Iterable[float],
        joint_limits: Sequence[tuple[float, float]],
        base_height: float = 0.35,
        joint_axes: Sequence[str] | None = None,
    ) -> None:
        link_array = np.array(list(link_lengths), dtype=float)
        if link_array.ndim != 1 or link_array.size == 0:
            raise ValueError("link_lengths must be a non-empty 1D iterable")
        self.link_lengths = link_array

        dof = int(self.link_lengths.size)
        if joint_axes is None:
            if dof > len(_BASE_AXES):
                raise ValueError(
                    f"joint_axes not provided and joint count {dof} exceeds supported maximum {len(_BASE_AXES)}"
                )
            self.joint_axes: tuple[str, ...] = tuple(_BASE_AXES[:dof])
        else:
            if len(joint_axes) != dof:
                raise ValueError("joint_axes length must match number of link lengths")
            for axis in joint_axes:
                if axis not in ("x", "y", "z"):
                    raise ValueError(f"Unsupported axis {axis}")
            self.joint_axes = tuple(joint_axes)

        if len(joint_limits) != dof:
            raise ValueError("joint_limits must contain (min, max) tuples for each joint")
        self.joint_limits = np.array(joint_limits, dtype=float)
        self.base_height = float(base_height)
        self.dof = dof

        self._link_vectors: list[np.ndarray] = []
        for idx in range(self.dof):
            if idx < len(_BASE_TRANSLATION_DIRECTIONS):
                direction = np.array(_BASE_TRANSLATION_DIRECTIONS[idx], dtype=float)
            else:
                direction = np.array([1.0, 0.0, 0.0], dtype=float)
            self._link_vectors.append(direction * self.link_lengths[idx])

    @classmethod
    def default(cls, joint_count: int | None = None) -> "RobotModel":
        if joint_count is None:
            joint_count = len(_BASE_LINK_LENGTHS)
        joint_count = int(joint_count)
        if joint_count < 1 or joint_count > len(_BASE_LINK_LENGTHS):
            raise ValueError(
                f"joint_count must be between 1 and {len(_BASE_LINK_LENGTHS)} (got {joint_count})"
            )
        lengths = _BASE_LINK_LENGTHS[:joint_count]
        limits = _BASE_JOINT_LIMITS[:joint_count]
        return cls(lengths, limits)

    def clamp(self, state: RobotState) -> RobotState:
        clamped = np.clip(state.joint_angles, self.joint_limits[:, 0], self.joint_limits[:, 1])
        if clamped.shape != (self.dof,):
            clamped = clamped[: self.dof]
        return RobotState(clamped)

    def _axis_matrix(self, axis_name: str, theta: float) -> np.ndarray:
        match axis_name:
            case "x":
                return _rot_x(theta)
            case "y":
                return _rot_y(theta)
            case "z":
                return _rot_z(theta)
            case _:
                raise ValueError(f"Unsupported axis {axis_name}")

    def _link_vector(self, index: int) -> np.ndarray:
        return self._link_vectors[index]

    def all_joint_transforms(self, joint_angles: np.ndarray) -> list[np.ndarray]:
        joint_angles = np.asarray(joint_angles, dtype=float)
        if joint_angles.shape != (self.dof,):
            raise ValueError(f"joint_angles must be a {self.dof}-vector")
        transforms: list[np.ndarray] = []
        current = _translate((0.0, 0.0, self.base_height))
        transforms.append(current.copy())
        for axis, theta, link_vec in zip(self.joint_axes, joint_angles, self._link_vectors):
            current = current @ self._axis_matrix(axis, theta)
            transforms.append(current.copy())
            current = current @ _translate(link_vec)
            transforms.append(current.copy())
        return transforms

    def all_joint_positions(self, joint_angles: np.ndarray) -> np.ndarray:
        transforms = self.all_joint_transforms(joint_angles)
        points = [transforms[0][:3, 3]]
        for i in range(self.dof):
            points.append(transforms[2 * i + 2][:3, 3])
        return np.vstack(points)

    def forward_kinematics(self, joint_angles: np.ndarray) -> Pose:
        transforms = self.all_joint_transforms(joint_angles)
        effector_transform = transforms[-1]
        position = effector_transform[:3, 3]
        rotation = effector_transform[:3, :3]
        return Pose(position, rotation)

    def jacobian(self, joint_angles: np.ndarray) -> np.ndarray:
        transforms = self.all_joint_transforms(joint_angles)
        effector_pos = transforms[-1][:3, 3]
        jac = np.zeros((6, self.dof))
        for i, axis_name in enumerate(self.joint_axes):
            axis_transform = transforms[2 * i + 1]
            rotation = axis_transform[:3, :3]
            axis_local = np.array([0.0, 0.0, 1.0])
            if axis_name == "x":
                axis_local = np.array([1.0, 0.0, 0.0])
            elif axis_name == "y":
                axis_local = np.array([0.0, 1.0, 0.0])
            axis_world = rotation @ axis_local
            joint_pos = axis_transform[:3, 3]
            linear = np.cross(axis_world, effector_pos - joint_pos)
            jac[:3, i] = linear
            jac[3:, i] = axis_world
        return jac

    def inverse_kinematics(
        self,
        target: Pose,
        initial: RobotState | None = None,
        max_iters: int = 200,
        tolerance: float = 1e-4,
        damping: float = 1e-2,
    ) -> RobotState:
        if initial is None:
            current_state = RobotState.zeros(self.dof)
        else:
            current_state = initial.copy()

        for _ in range(max_iters):
            pose = self.forward_kinematics(current_state.joint_angles)
            position_error = target.position - pose.position
            rotation_error_matrix = target.rotation @ pose.rotation.T
            rotation_error = rotation_matrix_to_vector(rotation_error_matrix)
            error = np.concatenate([position_error, rotation_error])
            if np.linalg.norm(error) < tolerance:
                return self.clamp(current_state)

            jac = self.jacobian(current_state.joint_angles)
            jtj = jac @ jac.T
            damped_identity = damping ** 2 * np.eye(6)
            delta = jac.T @ np.linalg.solve(jtj + damped_identity, error)
            current_state.joint_angles += delta
            current_state = self.clamp(current_state)

        raise RuntimeError("Inverse kinematics failed to converge")
