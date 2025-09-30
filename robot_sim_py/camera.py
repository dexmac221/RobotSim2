from __future__ import annotations

from dataclasses import dataclass
import numpy as np


@dataclass
class CameraView:
    """Pose and intrinsic description for a perspective camera capture."""

    position: np.ndarray
    forward: np.ndarray
    up: np.ndarray
    right: np.ndarray
    fov_y: float
    aspect: float
    image_width: int
    image_height: int
