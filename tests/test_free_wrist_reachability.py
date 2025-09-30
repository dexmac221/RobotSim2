import math
import numpy as np  # type: ignore[import]

from robot_sim_py.ogl_app import OpenGLArmApp
from robot_sim_py.scene import TABLE_HEIGHT, TARGET_HEIGHT_OFFSET


def test_free_wrist_flag_expands_reachability() -> None:
    target = np.array([0.05, -0.45, TABLE_HEIGHT + TARGET_HEIGHT_OFFSET], dtype=float)

    constrained_app = OpenGLArmApp(enable_hand_view=False)
    assert not constrained_app._ensure_target_reachable(target)

    free_app = OpenGLArmApp(enable_hand_view=False, allow_free_wrist=True)
    assert free_app._ensure_target_reachable(target)
    yaw = free_app._target_orientation[2]
    assert not math.isclose(yaw, 0.0, abs_tol=1e-3)
