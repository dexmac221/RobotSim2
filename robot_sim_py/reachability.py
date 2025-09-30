"""Shared reachability helpers for RobotSim trajectory planning."""

from __future__ import annotations

import math
from typing import List, Tuple

import numpy as np  # type: ignore[import]

from .robot import Pose, RobotModel, RobotState
from .scene import TABLE_HEIGHT, TARGET_HEIGHT_OFFSET, TIP_FORWARD_OFFSET

DEFAULT_ORIENTATION: Tuple[float, float, float] = (180.0, 0.0, 0.0)


def _wrap_angle_deg(angle: float) -> float:
    return ((float(angle) + 180.0) % 360.0) - 180.0


def orientation_candidates(
    position: np.ndarray,
    *,
    last_orientation: Tuple[float, float, float] = DEFAULT_ORIENTATION,
    allow_free_orientation: bool = False,
) -> List[Tuple[float, float, float]]:
    candidates: List[Tuple[float, float, float]] = []

    def add(candidate: Tuple[float, float, float]) -> None:
        if candidate not in candidates:
            candidates.append(candidate)

    add(DEFAULT_ORIENTATION)
    add(tuple(last_orientation))
    if not allow_free_orientation:
        return candidates

    yaw_center = math.degrees(math.atan2(float(position[1]), float(position[0]) + 1e-6))
    yaw_offsets = [
        0.0,
        -30.0,
        30.0,
        -60.0,
        60.0,
        -90.0,
        90.0,
        -120.0,
        120.0,
        -150.0,
        150.0,
        180.0,
    ]
    pitch_candidates = [0.0, -10.0, 10.0, -20.0, 20.0]
    for pitch in pitch_candidates:
        for offset in yaw_offsets:
            yaw = _wrap_angle_deg(yaw_center + offset)
            add((180.0, pitch, yaw))
    return candidates


def find_feasible_orientation(
    robot: RobotModel,
    state: RobotState,
    tip_position: np.ndarray,
    *,
    allow_free_orientation: bool = False,
    last_orientation: Tuple[float, float, float] = DEFAULT_ORIENTATION,
    hover_padding: float = 0.05,
) -> Tuple[bool, Tuple[float, float, float]]:
    approach_height = max(TABLE_HEIGHT + hover_padding, float(tip_position[2]) + hover_padding)
    grasp_height = max(TABLE_HEIGHT + TARGET_HEIGHT_OFFSET, float(tip_position[2]))

    candidates = orientation_candidates(
        tip_position,
        last_orientation=last_orientation,
        allow_free_orientation=allow_free_orientation,
    )

    target_xy = float(tip_position[0]), float(tip_position[1])

    for rpy in candidates:
        orientation_pose = Pose.from_xyz_rpy((0.0, 0.0, 0.0), rpy)
        rotation = orientation_pose.rotation
        tip_offset = rotation @ np.array([TIP_FORWARD_OFFSET, 0.0, 0.0], dtype=float)

        approach_tip = np.array([target_xy[0], target_xy[1], approach_height], dtype=float)
        grasp_tip = np.array([target_xy[0], target_xy[1], grasp_height], dtype=float)

        approach_wrist = approach_tip - tip_offset
        grasp_wrist = grasp_tip - tip_offset

        approach_pose = Pose.from_xyz_rpy(tuple(approach_wrist), rpy)
        grasp_pose = Pose.from_xyz_rpy(tuple(grasp_wrist), rpy)

        try:
            approach_state = robot.inverse_kinematics(approach_pose, initial=state)
            robot.inverse_kinematics(grasp_pose, initial=approach_state)
        except RuntimeError:
            continue
        return True, tuple(rpy)

    return False, tuple(last_orientation)