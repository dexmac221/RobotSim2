"""Visualization helpers using Matplotlib."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Iterable

import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D  # noqa: F401 imported for side effect
import numpy as np


@dataclass
class MatplotlibArmViewer:
    """Minimal 3D viewer for the robot arm."""

    figure: plt.Figure
    axes: plt.Axes
    line: any

    @classmethod
    def create(cls) -> "MatplotlibArmViewer":
        plt.ion()
        fig = plt.figure(figsize=(6, 6))
        ax = fig.add_subplot(111, projection="3d")
        ax.set_xlim(-0.8, 0.8)
        ax.set_ylim(-0.8, 0.8)
        ax.set_zlim(0.0, 1.2)
        ax.set_box_aspect((1, 1, 1))
        ax.set_xlabel("X [m]")
        ax.set_ylabel("Y [m]")
        ax.set_zlabel("Z [m]")
        (line,) = ax.plot([], [], [], marker="o", color="tab:orange")
        return cls(fig, ax, line)

    def update(self, positions: Iterable[Iterable[float]]) -> None:
        pts = np.asarray(list(positions), dtype=float)
        if pts.ndim != 2 or pts.shape[1] != 3:
            raise ValueError("positions must be a sequence of 3D coordinates")
        self.line.set_data(pts[:, 0], pts[:, 1])
        self.line.set_3d_properties(pts[:, 2])
        self.axes.figure.canvas.draw()
        self.axes.figure.canvas.flush_events()

    def show(self) -> None:
        plt.ioff()
        plt.show()
