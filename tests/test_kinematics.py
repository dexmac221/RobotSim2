import numpy as np

from robot_sim_py.robot import Pose, RobotModel, RobotState


def test_forward_kinematics_returns_pose():
    model = RobotModel.default()
    angles = np.deg2rad(np.array([10, -20, 15, 30, -45, 60]))
    pose = model.forward_kinematics(angles)
    assert pose.position.shape == (3,)
    rot = pose.rotation
    should_be_identity = rot @ rot.T
    assert np.allclose(should_be_identity, np.eye(3), atol=1e-6)


def test_jacobian_shape():
    model = RobotModel.default()
    angles = np.deg2rad(np.array([-15, 5, 20, -10, 30, -25]))
    jac = model.jacobian(angles)
    assert jac.shape == (6, 6)


def test_inverse_kinematics_recovers_target():
    model = RobotModel.default()
    target = Pose.from_xyz_rpy((0.35, 0.1, 0.55), (10.0, -5.0, 20.0))
    state = model.inverse_kinematics(target, initial=RobotState.zeros(), max_iters=150)
    pose = model.forward_kinematics(state.joint_angles)
    assert np.allclose(pose.position, target.position, atol=1e-2)
    rotation_error = target.rotation @ pose.rotation.T
    assert np.allclose(rotation_error, np.eye(3), atol=5e-2)
