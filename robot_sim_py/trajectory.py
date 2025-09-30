"""Trajectory primitives for pose interpolation."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Iterable, List, Sequence, Tuple

import numpy as np

from .robot import Pose


def _matrix_to_quaternion(matrix: np.ndarray) -> np.ndarray:
    m = matrix
    trace = m[0, 0] + m[1, 1] + m[2, 2]
    if trace > 0:
        s = 0.5 / np.sqrt(trace + 1.0)
        w = 0.25 / s
        x = (m[2, 1] - m[1, 2]) * s
        y = (m[0, 2] - m[2, 0]) * s
        z = (m[1, 0] - m[0, 1]) * s
    else:
        if m[0, 0] > m[1, 1] and m[0, 0] > m[2, 2]:
            s = 2.0 * np.sqrt(1.0 + m[0, 0] - m[1, 1] - m[2, 2])
            w = (m[2, 1] - m[1, 2]) / s
            x = 0.25 * s
            y = (m[0, 1] + m[1, 0]) / s
            z = (m[0, 2] + m[2, 0]) / s
        elif m[1, 1] > m[2, 2]:
            s = 2.0 * np.sqrt(1.0 + m[1, 1] - m[0, 0] - m[2, 2])
            w = (m[0, 2] - m[2, 0]) / s
            x = (m[0, 1] + m[1, 0]) / s
            y = 0.25 * s
            z = (m[1, 2] + m[2, 1]) / s
        else:
            s = 2.0 * np.sqrt(1.0 + m[2, 2] - m[0, 0] - m[1, 1])
            w = (m[1, 0] - m[0, 1]) / s
            x = (m[0, 2] + m[2, 0]) / s
            y = (m[1, 2] + m[2, 1]) / s
            z = 0.25 * s
    quat = np.array([w, x, y, z])
    return quat / np.linalg.norm(quat)


def _quaternion_to_matrix(quat: np.ndarray) -> np.ndarray:
    w, x, y, z = quat
    return np.array(
        [
            [1 - 2 * (y**2 + z**2), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x**2 + z**2), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x**2 + y**2)],
        ]
    )


def _slerp(q0: np.ndarray, q1: np.ndarray, u: float) -> np.ndarray:
    cos_theta = np.dot(q0, q1)
    if cos_theta < 0.0:
        q1 = -q1
        cos_theta = -cos_theta
    if cos_theta > 0.9995:
        result = q0 + u * (q1 - q0)
        return result / np.linalg.norm(result)
    theta = np.arccos(np.clip(cos_theta, -1.0, 1.0))
    sin_theta = np.sin(theta)
    w0 = np.sin((1 - u) * theta) / sin_theta
    w1 = np.sin(u * theta) / sin_theta
    return w0 * q0 + w1 * q1


@dataclass
class Trajectory:
    """A time-parameterized sequence of poses."""

    keyframes: List[Tuple[float, Pose]]

    def __post_init__(self) -> None:
        if not self.keyframes:
            raise ValueError("Trajectory requires at least one keyframe")
        self.keyframes.sort(key=lambda x: x[0])

    def duration(self) -> float:
        return self.keyframes[-1][0]

    def pose_at(self, t: float) -> Pose:
        if t <= self.keyframes[0][0]:
            return self.keyframes[0][1]
        if t >= self.keyframes[-1][0]:
            return self.keyframes[-1][1]
        for (t0, pose0), (t1, pose1) in zip(self.keyframes, self.keyframes[1:]):
            if t0 <= t <= t1:
                u = (t - t0) / (t1 - t0)
                position = (1 - u) * pose0.position + u * pose1.position
                q0 = _matrix_to_quaternion(pose0.rotation)
                q1 = _matrix_to_quaternion(pose1.rotation)
                rotation = _quaternion_to_matrix(_slerp(q0, q1, u))
                return Pose(position, rotation)
        return self.keyframes[-1][1]

    @classmethod
    def linear_path(
        cls,
        start: Pose,
        end: Pose,
        samples: int,
        duration: float,
    ) -> "Trajectory":
        if samples < 2:
            raise ValueError("samples must be >= 2")
        keyframes: List[Tuple[float, Pose]] = []
        for i in range(samples):
            t = duration * i / (samples - 1)
            u = i / (samples - 1)
            position = (1 - u) * start.position + u * end.position
            q0 = _matrix_to_quaternion(start.rotation)
            q1 = _matrix_to_quaternion(end.rotation)
            rotation = _quaternion_to_matrix(_slerp(q0, q1, u))
            keyframes.append((t, Pose(position, rotation)))
        return cls(keyframes)
