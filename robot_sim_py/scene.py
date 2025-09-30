"""Shared scene constants for RobotSim rendering and controllers."""

TABLE_HEIGHT: float = 0.04
TABLE_EXTENT_X: float = 0.8
TABLE_EXTENT_Y: float = 0.8

# Preferred tabletop target height offset to keep demo objects slightly above the surface.
TARGET_HEIGHT_OFFSET: float = 0.06

# Geometric dimensions of the physical gripper used for rendering and targeting.
WRIST_LENGTH: float = 0.06
PALM_LENGTH: float = 0.085
FINGER_LENGTH: float = 0.11
FINGER_TIP_LENGTH: float = 0.025

# Forward distance from the wrist frame origin to the fingertip, used to convert between
# wrist-centric kinematics and fingertip-centric target specifications.
TIP_FORWARD_OFFSET: float = WRIST_LENGTH + PALM_LENGTH + FINGER_LENGTH + FINGER_TIP_LENGTH / 2.0

# Offset from the robot base origin to the head-mounted RGB camera ("hand" inset view).
HEAD_CAMERA_OFFSET: tuple[float, float, float] = (-0.30, 0.0, 1.05)

# Conservative bounds for reachable Cartesian workspace (tip position).
WORKSPACE_X_BOUNDS: tuple[float, float] = (0.05, 0.55)
WORKSPACE_Y_BOUNDS: tuple[float, float] = (-0.45, 0.45)
