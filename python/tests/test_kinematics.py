from __future__ import annotations

import numpy as np

from robotsim_py.robot import RobotArm


def test_jacobian_matches_finite_difference() -> None:
    robot = RobotArm.default_puma()
    q = np.array([0.2, -0.3, 0.4, 0.1, -0.2, 0.3])
    analytic = robot.jacobian(q)
    numerical = robot.finite_difference_jacobian(q, epsilon=1e-6)
    assert np.allclose(analytic, numerical, atol=1e-4)


def test_iterative_step_reduces_error() -> None:
    robot = RobotArm.default_puma()
    start_angles = robot.home_configuration()
    target = np.array([0.5, 0.2, 0.6])

    initial_error = target - robot.end_effector(start_angles)
    angles_next, _ = robot.iter_inverse_step(start_angles, target, step_scale=1.0)
    new_error = target - robot.end_effector(angles_next)

    assert np.linalg.norm(new_error) < np.linalg.norm(initial_error)
