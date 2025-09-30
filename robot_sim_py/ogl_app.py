"""Interactive OpenGL viewer for the RobotSim Python port."""

from __future__ import annotations

import argparse
import json
import math
import os
import random
import shutil
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Optional, Sequence, TYPE_CHECKING

import numpy as np

try:  # pragma: no cover - optional dependency fallback
    from PIL import Image
except ImportError:  # pragma: no cover
    Image = None

try:
    import glfw
    from OpenGL.GL import (
        GL_AMBIENT,
        GL_AMBIENT_AND_DIFFUSE,
        GL_BACK,
        GL_COLOR_BUFFER_BIT,
        GL_COLOR_MATERIAL,
        GL_DEPTH_BUFFER_BIT,
        GL_DEPTH_TEST,
        GL_LINES,
        GL_LINE_STRIP,
        GL_MODELVIEW,
        GL_PACK_ALIGNMENT,
        GL_PROJECTION,
        GL_QUADS,
        GL_RGBA,
        GL_SCISSOR_TEST,
        GL_UNSIGNED_BYTE,
        glBegin,
        glClear,
        glClearColor,
        glColor3f,
        glDisable,
        glEnable,
        glEnd,
        glFinish,
        glLineWidth,
        glLoadIdentity,
        glMatrixMode,
        glNormal3f,
        glRotatef,
        glPixelStorei,
        glPopMatrix,
        glPushMatrix,
        glReadBuffer,
        glReadPixels,
        glScissor,
        glMultMatrixf,
        glTranslatef,
        glVertex3f,
        glViewport,
    )
    from OpenGL.GLU import (
        gluCylinder,
        gluDeleteQuadric,
        gluDisk,
        gluLookAt,
        gluNewQuadric,
        gluPerspective,
        gluSphere,
    )
except ImportError as exc:
    raise RuntimeError(
        "OpenGLArmApp requires PyOpenGL and glfw. Install extras via pip install -e .[dev]"
    ) from exc

from .camera import CameraView
from .controllers import TrajectoryController
from .robot import Pose, RobotModel, RobotState
from .scene import (
    TABLE_HEIGHT,
    TABLE_EXTENT_X,
    TABLE_EXTENT_Y,
    TARGET_HEIGHT_OFFSET,
    WRIST_LENGTH,
    PALM_LENGTH,
    FINGER_LENGTH,
    FINGER_TIP_LENGTH,
    TIP_FORWARD_OFFSET,
    HEAD_CAMERA_OFFSET,
    WORKSPACE_X_BOUNDS,
    WORKSPACE_Y_BOUNDS,
)
from .reachability import DEFAULT_ORIENTATION, find_feasible_orientation
from .trajectory import Trajectory

if TYPE_CHECKING:  # pragma: no cover
    from .gemini_agent import GeminiRobotAgent


@dataclass
class _InputState:
    auto_mode: bool = False
    selected_joint: int = 0
    cam_yaw: float = -45.0
    cam_pitch: float = 30.0
    cam_distance: float = 2.2
    joint_speed: float = math.radians(45.0)
    hand_view_enabled: bool = True


FINGER_TIP_HALF = FINGER_TIP_LENGTH / 2.0

# Soft, high-contrast joint/link colors (avoid red so the Gemini target stays visually distinct).
JOINT_COLORS: list[tuple[float, float, float]] = [
    (0.35, 0.65, 0.95),  # base
    (0.30, 0.80, 0.50),
    (0.90, 0.80, 0.25),
    (0.75, 0.55, 0.95),
    (0.30, 0.75, 0.85),
    (0.85, 0.45, 0.35),
    (0.60, 0.80, 0.40),
]


class OpenGLArmApp:
    """Minimal OpenGL window mirroring the original RobotSim functionality."""

    def __init__(
        self,
        robot: Optional[RobotModel] = None,
        trajectory: Optional[Trajectory] = None,
        window_size: tuple[int, int] = (960, 720),
        log_path: Optional[Path] = None,
        enable_hand_view: bool = True,
        allow_free_wrist: bool = False,
        hand_capture_dir: Optional[Path] = None,
        top_capture_dir: Optional[Path] = None,
        top_capture_resolution: Optional[tuple[int, int]] = None,
        gemini_agent: Optional["GeminiRobotAgent"] = None,
        gemini_single_shot: bool = False,
        gemini_debug_dir: Optional[Path] = None,
    ) -> None:
        self.robot = robot or RobotModel.default()
        self.state = RobotState.zeros(self.robot.dof)
        self._custom_trajectory = trajectory is not None
        self._free_wrist = allow_free_wrist
        self._target_orientation: tuple[float, float, float] = DEFAULT_ORIENTATION
        self._target_position = np.array(
            [0.30, 0.0, TABLE_HEIGHT + TARGET_HEIGHT_OFFSET], dtype=float
        )
        if self._custom_trajectory:
            self.trajectory = trajectory  # type: ignore[assignment]
        else:
            self.trajectory = self._create_trajectory_for_target(self._target_position)
        self.trajectory_controller = TrajectoryController(self.robot, self.trajectory)
        self.trajectory_controller.state = self.state
        self._input = _InputState(hand_view_enabled=enable_hand_view)
        self._window: Optional[glfw._GLFWwindow] = None
        self._window_size = window_size
        self._log_path = log_path
        self._time_in_traj = 0.0
        self._log: list[dict[str, float]] = []
        self._should_quit = False
        self._hand_view_enabled = enable_hand_view
        self._hand_capture_dir = hand_capture_dir
        if self._hand_capture_dir:
            self._hand_capture_dir.mkdir(parents=True, exist_ok=True)
        if gemini_single_shot and top_capture_dir is None:
            default_capture_dir = Path("data/head_frames")
            default_capture_dir.mkdir(parents=True, exist_ok=True)
            self._perception_capture_dir = default_capture_dir
        else:
            self._perception_capture_dir = top_capture_dir
        if self._perception_capture_dir:
            self._perception_capture_dir.mkdir(parents=True, exist_ok=True)
        self._perception_capture_resolution = top_capture_resolution
        self._hand_frame_index = 0
        self._perception_frame_index = 0
        self._perception_frame_serial = -1
        self._current_pose: Pose = self.robot.forward_kinematics(self.state.joint_angles)
        self._latest_positions = self.robot.all_joint_positions(self.state.joint_angles)
        self._controls_printed = False
        self._gemini_agent = gemini_agent
        self._gemini_agent_available = gemini_agent is not None
        self._gemini_agent_enabled = False
        self._gemini_sequence_complete = False
        self._latest_perception_frame: Optional[np.ndarray] = None
        self._latest_perception_frame_path: Optional[Path] = None
        self._latest_perception_view: Optional[CameraView] = None
        self._gemini_single_shot = gemini_single_shot and self._gemini_agent_available
        self._gemini_request_made = False
        self._gemini_debug_dir = gemini_debug_dir
        if self._gemini_debug_dir:
            self._gemini_debug_dir.mkdir(parents=True, exist_ok=True)
        self._status_message = ""
        self._status_expiry = 0.0
        self._trajectory_complete = False
        self._gripper_open_max = 0.055
        self._gripper_open_min = 0.012
        self._gripper_open_amount = self._gripper_open_max
        self._gripper_target_open = self._gripper_open_max
        self._gripper_hold_until = 0.0
        self._request_gripper_open()
        self._gemini_radius_threshold = 0.06
        self._gemini_height_threshold = 0.08
        self._gemini_motion_active = False
        self._gemini_motion_start = self.state.joint_angles.copy()
        self._gemini_motion_target = self.state.joint_angles.copy()
        self._gemini_motion_duration = 1.0
        self._gemini_motion_elapsed = 0.0
        if not self._custom_trajectory:
            if not self._apply_target_position(randomize=True, reset_time=False):
                fallback = np.array(
                    [0.24, 0.0, TABLE_HEIGHT + TARGET_HEIGHT_OFFSET], dtype=float
                )
                if not self._apply_target_position(
                    position=fallback,
                    randomize=False,
                    reset_time=False,
                ):
                    self._target_position = fallback
                    self.trajectory = self._create_trajectory_for_target(fallback)
                    self.trajectory_controller = TrajectoryController(self.robot, self.trajectory)
                    self.trajectory_controller.state = self.state
                self._set_status_message(
                    "Using fallback target near workspace center.",
                    duration=4.0,
                    log_to_console=True,
                )

    def run(self) -> None:
        self._print_controls()
        if self._gemini_agent_available:
            print("Gemini agent ready. Press 'G' to start/stop Gemini control loop.")
        self._init_window()
        last_frame = time.perf_counter()
        while not glfw.window_should_close(self._window) and not self._should_quit:
            now = time.perf_counter()
            dt = max(1e-4, now - last_frame)
            last_frame = now
            glfw.poll_events()
            self._update(dt)
            self._render()
            glfw.swap_buffers(self._window)
        self._shutdown()

    # ------------------------------------------------------------------
    # Setup & teardown
    def _init_window(self) -> None:
        if not glfw.init():
            raise RuntimeError("Failed to initialize GLFW (OpenGL context)")
        glfw.window_hint(glfw.RESIZABLE, glfw.TRUE)
        self._window = glfw.create_window(
            self._window_size[0], self._window_size[1], "RobotSim OpenGL", None, None
        )
        if not self._window:
            glfw.terminate()
            raise RuntimeError("Unable to create GLFW window")
        glfw.make_context_current(self._window)
        glfw.set_key_callback(self._window, self._on_key)
        glEnable(GL_DEPTH_TEST)
        glClearColor(0.18, 0.18, 0.22, 1.0)

    def _shutdown(self) -> None:
        if self._log_path and self._log:
            import json

            self._log_path.write_text(json.dumps(self._log, indent=2))
        if self._window:
            glfw.destroy_window(self._window)
        glfw.terminate()

    def _randomize_target_position(self) -> np.ndarray:
        x = random.uniform(WORKSPACE_X_BOUNDS[0], WORKSPACE_X_BOUNDS[1])
        y = random.uniform(WORKSPACE_Y_BOUNDS[0], WORKSPACE_Y_BOUNDS[1])
        return np.array([x, y, TABLE_HEIGHT + TARGET_HEIGHT_OFFSET], dtype=float)

    def _ensure_target_reachable(self, position: np.ndarray) -> bool:
        success, orientation = find_feasible_orientation(
            self.robot,
            self.state,
            position,
            allow_free_orientation=self._free_wrist,
            last_orientation=self._target_orientation,
        )
        if success:
            self._target_orientation = orientation
            return True
        return False

    def _set_status_message(
        self,
        message: str,
        duration: float = 3.0,
        *,
        log_to_console: bool = False,
    ) -> None:
        self._status_message = message
        self._status_expiry = time.perf_counter() + max(0.5, duration)
        if log_to_console:
            print(message)

    def _apply_target_position(
        self,
        *,
        position: Optional[np.ndarray] = None,
        randomize: bool = False,
        reset_time: bool = True,
    ) -> bool:
        attempts = 0
        candidate = position.copy() if position is not None else None
        fallback_used = False
        while True:
            if randomize or candidate is None:
                candidate = self._randomize_target_position()
            if self._ensure_target_reachable(candidate):
                break
            attempts += 1
            if randomize and not fallback_used and attempts >= 8:
                candidate = np.array(
                    [0.24, 0.0, TABLE_HEIGHT + TARGET_HEIGHT_OFFSET], dtype=float
                )
                fallback_used = True
                continue
            if not randomize or attempts >= 16:
                self._set_status_message(
                    "Target position unreachable for current robot configuration.",
                    duration=4.0,
                    log_to_console=True,
                )
                return False
            candidate = None

        self._target_position = candidate
        self._trajectory_complete = False
        self._request_gripper_open()
        if not self._custom_trajectory:
            self.trajectory = self._create_trajectory_for_target(candidate)
            self.trajectory_controller = TrajectoryController(self.robot, self.trajectory)
            self.trajectory_controller.state = self.state
            if reset_time:
                self._time_in_traj = 0.0
        return True

    def _adjust_target_position(self, dx: float = 0.0, dy: float = 0.0, dz: float = 0.0) -> None:
        if self._target_position is None:
            return
        new_position = self._target_position.copy()
        new_position[0] = np.clip(
            new_position[0] + dx,
            -TABLE_EXTENT_X + 0.05,
            TABLE_EXTENT_X - 0.05,
        )
        new_position[1] = np.clip(
            new_position[1] + dy,
            -TABLE_EXTENT_Y + 0.05,
            TABLE_EXTENT_Y - 0.05,
        )
        min_height = TABLE_HEIGHT + TARGET_HEIGHT_OFFSET
        new_position[2] = np.clip(new_position[2] + dz, min_height, TABLE_HEIGHT + 0.3)

        if not self._apply_target_position(position=new_position, randomize=False):
            self._set_status_message(
                "Adjusted target unreachable; reverted to previous position.",
                duration=4.0,
                log_to_console=True,
            )
            return

        x, y, z = self._target_position
        self._set_status_message(
            f"Target updated: x={x:+.3f} m, y={y:+.3f} m, z={z:+.3f} m",
            duration=4.0,
            log_to_console=True,
        )

    def _request_gripper_open(self) -> None:
        self._gripper_target_open = self._gripper_open_max
        self._gripper_hold_until = time.perf_counter()

    def _request_gripper_close(self, hold_seconds: float = 2.5) -> None:
        self._gripper_target_open = self._gripper_open_min
        hold_until = time.perf_counter() + max(0.1, hold_seconds)
        self._gripper_hold_until = max(self._gripper_hold_until, hold_until)

    def _cancel_gemini_motion(self) -> None:
        self._gemini_motion_active = False
        self._gemini_motion_elapsed = 0.0

    def _schedule_gemini_motion(self, target_state: RobotState) -> None:
        target_angles = np.array(target_state.joint_angles, dtype=float)
        start_angles = np.array(self.state.joint_angles, dtype=float)
        if self._gemini_motion_active and np.allclose(
            target_angles, self._gemini_motion_target, atol=1e-4
        ):
            return
        distance = float(np.linalg.norm(target_angles - start_angles))
        if distance < 1e-5:
            self.state = self.robot.clamp(RobotState(target_angles))
            self._cancel_gemini_motion()
            return
        duration = float(np.clip(distance / 0.9, 0.35, 1.8))
        self._gemini_motion_start = start_angles
        self._gemini_motion_target = target_angles
        self._gemini_motion_duration = duration
        self._gemini_motion_elapsed = 0.0
        self._gemini_motion_active = True

    def _update_gemini_motion(self, dt: float) -> None:
        if not self._gemini_motion_active:
            return
        self._gemini_motion_elapsed += dt
        duration = max(1e-3, self._gemini_motion_duration)
        alpha = float(np.clip(self._gemini_motion_elapsed / duration, 0.0, 1.0))
        new_angles = (1.0 - alpha) * self._gemini_motion_start + alpha * self._gemini_motion_target
        self.state = self.robot.clamp(RobotState(new_angles.copy()))
        if alpha >= 1.0:
            self._cancel_gemini_motion()

    def _hand_tip_position(self) -> Optional[np.ndarray]:
        if self._current_pose is None:
            return None
        offset_vec = np.array([TIP_FORWARD_OFFSET, 0.0, 0.0])
        return self._current_pose.position + self._current_pose.rotation @ offset_vec

    def _hand_on_target(self) -> bool:
        if self._target_position is None or self._current_pose is None:
            return False
        tip_position = self._hand_tip_position()
        reference = tip_position if tip_position is not None else self._current_pose.position
        distance = float(np.linalg.norm(reference - self._target_position))
        vertical_gap = abs(reference[2] - self._target_position[2])
        return (
            distance <= self._gemini_radius_threshold
            and vertical_gap <= self._gemini_height_threshold
        )

    def _update_gripper_target(self, pose: Pose) -> None:
        if self._target_position is None:
            return
        tip_position = self._hand_tip_position()
        reference = tip_position if tip_position is not None else pose.position
        distance = np.linalg.norm(reference - self._target_position)
        vertical_gap = abs(reference[2] - self._target_position[2])
        if distance < 0.05 and vertical_gap < 0.08:
            self._request_gripper_close(hold_seconds=2.5)
        elif time.perf_counter() > self._gripper_hold_until:
            self._gripper_target_open = self._gripper_open_max

    def _update_gripper_animation(self, dt: float) -> None:
        target = np.clip(self._gripper_target_open, self._gripper_open_min, self._gripper_open_max)
        blend = 1.0 - math.exp(-dt * 6.0)
        self._gripper_open_amount += (target - self._gripper_open_amount) * blend
        self._gripper_open_amount = float(np.clip(self._gripper_open_amount, self._gripper_open_min, self._gripper_open_max))

    def _persist_gemini_debug(
        self,
        *,
        status: str,
        suggestion: Optional[RobotState],
    ) -> Optional[Path]:
        if not self._gemini_debug_dir or self._gemini_agent is None:
            return None

        timestamp = time.strftime("%Y%m%d_%H%M%S")
        frame_id = max(0, self._perception_frame_serial)
        base_name = f"gemini_{timestamp}_{frame_id:05d}"

        image_path: Optional[Path] = None
        if self._latest_perception_frame_path:
            source_path = Path(self._latest_perception_frame_path)
            if source_path.exists():
                destination = self._gemini_debug_dir / source_path.name
                try:
                    if source_path.resolve() != destination.resolve():
                        shutil.copy2(source_path, destination)
                    image_path = destination
                except OSError:
                    image_path = source_path

        if image_path is None and self._latest_perception_frame is not None:
            if Image is not None:
                image_path = self._gemini_debug_dir / f"{base_name}.png"
                try:
                    Image.fromarray(self._latest_perception_frame, "RGB").save(image_path)
                except OSError:
                    image_path = None
            else:
                image_path = self._gemini_debug_dir / f"{base_name}.npy"
                try:
                    np.save(str(image_path), self._latest_perception_frame)
                except OSError:
                    image_path = None

        payload = {
            "timestamp": time.time(),
            "status": status,
            "frame_serial": int(self._perception_frame_serial),
            "perception_frame_path": str(image_path) if image_path else None,
            "source_frame_path": str(self._latest_perception_frame_path)
            if self._latest_perception_frame_path
            else None,
            "target_world": self._target_position.tolist() if self._target_position is not None else None,
            "response_text": self._gemini_agent.last_response_text,
            "parsed_points": self._gemini_agent.last_points,
        }

        agent = self._gemini_agent
        payload["gemini_mapping_source"] = getattr(agent, "last_mapping_source", "unknown")
        payload["gemini_projection_clamped"] = bool(getattr(agent, "last_projection_clamped", False))
        if getattr(agent, "last_camera_projection", None) is not None:
            payload["gemini_camera_projection"] = agent.last_camera_projection.tolist()
        if getattr(agent, "last_tip_world", None) is not None:
            payload["gemini_tip_world"] = agent.last_tip_world.tolist()
        payload["gemini_orientation_rpy"] = list(getattr(agent, "last_orientation", DEFAULT_ORIENTATION))
        if getattr(agent, "last_hover_pose", None) is not None:
            hover_pose = agent.last_hover_pose
            payload["gemini_hover_pose"] = {
                "position": hover_pose.position.tolist(),
                "rotation": hover_pose.rotation.tolist(),
            }

        if suggestion is not None:
            payload["joint_angles"] = suggestion.joint_angles.tolist()

        if self._latest_perception_view is not None:
            view = self._latest_perception_view
            payload["camera_view"] = {
                "position": view.position.tolist(),
                "forward": view.forward.tolist(),
                "up": view.up.tolist(),
                "right": view.right.tolist(),
                "fov_y_rad": view.fov_y,
                "aspect": view.aspect,
                "image_width": view.image_width,
                "image_height": view.image_height,
            }

        json_path = self._gemini_debug_dir / f"{base_name}.json"
        json_path.write_text(json.dumps(payload, indent=2))
        return json_path

    def _report_single_shot(self, *, status: str, debug_path: Optional[Path]) -> None:
        if not self._gemini_single_shot:
            return

        if status == "success":
            message = "Gemini single-shot succeeded"
        elif status == "ik_failure":
            message = "Gemini single-shot produced points but IK failed"
        elif status == "no_target":
            message = "Gemini single-shot did not identify a target"
        elif status == "no_response":
            message = "Gemini single-shot produced no response"
        else:
            message = f"Gemini single-shot finished with status '{status}'"

        agent = self._gemini_agent
        if agent is not None and agent.last_points and agent.last_tip_world is not None:
            first_point = agent.last_points[0].get("point")
            if isinstance(first_point, Sequence) and not isinstance(first_point, (str, bytes)):
                point_display = tuple(int(round(float(v))) for v in first_point[:2])
            else:
                point_display = first_point
            tip = agent.last_tip_world
            orientation = agent.last_orientation
            mapping_source = getattr(agent, "last_mapping_source", "unknown")
            extras = [
                f"px={point_display}",
                f"world=({tip[0]:+.3f}, {tip[1]:+.3f}, {tip[2]:+.3f})",
                f"rpy=({orientation[0]:+.1f}°, {orientation[1]:+.1f}°, {orientation[2]:+.1f}°)",
                f"source={mapping_source}",
            ]
            if getattr(agent, "last_projection_clamped", False):
                extras.append("clamped")
            message += " | " + ", ".join(extras)

        if debug_path is not None:
            message += f". Debug saved to {debug_path}"
        print(message)

    # ------------------------------------------------------------------
    # Simulation update
    def _update(self, dt: float) -> None:
        window = self._window
        if window is None:
            return

        # Camera controls
        if glfw.get_key(window, glfw.KEY_LEFT) == glfw.PRESS:
            self._input.cam_yaw -= 60.0 * dt
        if glfw.get_key(window, glfw.KEY_RIGHT) == glfw.PRESS:
            self._input.cam_yaw += 60.0 * dt
        if glfw.get_key(window, glfw.KEY_UP) == glfw.PRESS:
            self._input.cam_pitch = min(85.0, self._input.cam_pitch + 40.0 * dt)
        if glfw.get_key(window, glfw.KEY_DOWN) == glfw.PRESS:
            self._input.cam_pitch = max(5.0, self._input.cam_pitch - 40.0 * dt)
        if glfw.get_key(window, glfw.KEY_PAGE_UP) == glfw.PRESS:
            self._input.cam_distance = max(1.0, self._input.cam_distance - 1.5 * dt)
        if glfw.get_key(window, glfw.KEY_PAGE_DOWN) == glfw.PRESS:
            self._input.cam_distance = min(4.0, self._input.cam_distance + 1.5 * dt)

        self._update_gemini_motion(dt)

        agent = self._gemini_agent
        agent_available = agent is not None
        agent_active = agent_available and self._gemini_agent_enabled

        if self.robot.dof:
            self._input.selected_joint = max(0, min(self._input.selected_joint, self.robot.dof - 1))

        if agent_active and self._hand_on_target():
            if not self._gemini_sequence_complete:
                self._gemini_sequence_complete = True
                self._gemini_agent_enabled = False
                self._cancel_gemini_motion()
                self._set_status_message(
                    "Gemini control complete: target reached.",
                    duration=4.0,
                    log_to_console=True,
                )
                self._request_gripper_close(hold_seconds=3.0)
            agent_active = False

        if agent_active:
            executed_request = False
            suggestion: Optional[RobotState] = None

            if not (self._gemini_single_shot and self._gemini_request_made):
                if self._latest_perception_frame is not None and self._perception_frame_serial >= 0:
                    suggestion = agent.compute_next_state(
                        current_state=self.state,
                        top_frame=self._latest_perception_frame,
                        frame_serial=self._perception_frame_serial,
                        camera_view=self._latest_perception_view,
                    )
                    executed_request = True
                elif not self._gemini_single_shot:
                    suggestion = agent.compute_next_state(
                        current_state=self.state,
                        top_frame=self._latest_perception_frame,
                        frame_serial=self._perception_frame_serial,
                        camera_view=self._latest_perception_view,
                    )

            if suggestion is not None:
                self._schedule_gemini_motion(suggestion)

            if executed_request and agent is not None:
                point_display = None
                if agent.last_points:
                    first_point = agent.last_points[0].get("point")
                    if isinstance(first_point, Sequence) and not isinstance(first_point, (str, bytes)):
                        point_display = tuple(int(round(float(v))) for v in first_point[:2])
                tip = getattr(agent, "last_tip_world", None)
                orientation = getattr(agent, "last_orientation", DEFAULT_ORIENTATION)
                mapping_source = getattr(agent, "last_mapping_source", "unknown")
                clamped = bool(getattr(agent, "last_projection_clamped", False))

                if suggestion is not None and tip is not None:
                    msg = "Gemini target"
                    if point_display is not None:
                        msg += f" px={point_display}"
                    msg += (
                        f" → world=({tip[0]:+.3f}, {tip[1]:+.3f}, {tip[2]:+.3f})"
                        f" rpy=({orientation[0]:+.1f}°, {orientation[1]:+.1f}°, {orientation[2]:+.1f}°)"
                    )
                    msg += f" via {mapping_source}"
                    if clamped:
                        msg += " (clamped)"
                    self._set_status_message(msg, duration=5.0, log_to_console=True)
                elif agent.last_points:
                    reason = "inverse kinematics"
                    if getattr(agent, "last_hover_pose", None) is None:
                        reason = "orientation search"
                    detail = "Gemini mapping"
                    if point_display is not None and tip is not None:
                        detail += (
                            f" px={point_display} → world=({tip[0]:+.3f}, {tip[1]:+.3f}, {tip[2]:+.3f})"
                        )
                    detail += f", source={mapping_source}"
                    if clamped:
                        detail += " (clamped)"
                    self._set_status_message(
                        f"Gemini {reason} failed; {detail}",
                        duration=6.0,
                        log_to_console=True,
                    )

            if executed_request and self._gemini_single_shot:
                status: str
                if suggestion is not None:
                    status = "success"
                else:
                    if self._gemini_agent.last_points:
                        status = "ik_failure"
                    elif self._gemini_agent.last_response_text:
                        status = "no_target"
                    else:
                        status = "no_response"
                debug_path = self._persist_gemini_debug(status=status, suggestion=suggestion)
                self._report_single_shot(status=status, debug_path=debug_path)
                self._gemini_request_made = True

            self._time_in_traj += dt
        elif self._input.auto_mode:
            self.trajectory_controller.state = self.state
            duration = self.trajectory.duration()
            self._time_in_traj = min(duration, self._time_in_traj + dt)
            new_state = self.trajectory_controller.step(self._time_in_traj)
            self.state = self.robot.clamp(new_state)
            if self.trajectory_controller.last_error:
                if self._status_message != self.trajectory_controller.last_error:
                    self._set_status_message(self.trajectory_controller.last_error, duration=4.0)
                self._trajectory_complete = False
            elif not self._trajectory_complete and self._time_in_traj >= duration - 1e-3:
                self._set_status_message("Target reached", duration=3.5)
                self._trajectory_complete = True
                self._request_gripper_close(hold_seconds=3.0)
        else:
            self._time_in_traj += dt
            delta = 0.0
            if glfw.get_key(window, glfw.KEY_K) == glfw.PRESS:
                delta += self._input.joint_speed * dt
            if glfw.get_key(window, glfw.KEY_J) == glfw.PRESS:
                delta -= self._input.joint_speed * dt
            if abs(delta) > 1e-9:
                angles = self.state.joint_angles.copy()
                angles[self._input.selected_joint] += delta
                self.state = self.robot.clamp(RobotState(angles))

        pose = self.robot.forward_kinematics(self.state.joint_angles)
        self._current_pose = pose
        self._latest_positions = self.robot.all_joint_positions(self.state.joint_angles)
        self._log.append(
            {
                "time": self._time_in_traj,
                "px": float(pose.position[0]),
                "py": float(pose.position[1]),
                "pz": float(pose.position[2]),
            }
        )

        self._update_gripper_target(pose)
        self._update_gripper_animation(dt)

        if self._status_message and time.perf_counter() > self._status_expiry:
            self._status_message = ""

        if agent_active:
            state = "GEMINI"
        else:
            state = "AUTO" if self._input.auto_mode else "MANUAL"
        joint_label = self._input.selected_joint + 1
        title = f"RobotSim OpenGL - {state} (Joint {joint_label})"
        if self._status_message:
            title += f" | {self._status_message}"
        glfw.set_window_title(self._window, title)

    # ------------------------------------------------------------------
    # Rendering
    def _render(self) -> None:
        if self._window is None or self._latest_positions is None:
            return
        width, height = glfw.get_framebuffer_size(self._window)
        self._render_primary_scene(width, height)

        need_perception_scene = self._perception_capture_dir is not None or (
            self._gemini_agent_enabled and self._gemini_agent is not None
        )
        if need_perception_scene:
            capture_w, capture_h = self._render_perception_scene(width, height)
            rgb_frame, frame_path = self._capture_perception_frame(
                capture_w,
                capture_h,
                save_to_disk=self._perception_capture_dir is not None,
            )
            if rgb_frame is not None:
                self._latest_perception_frame = rgb_frame
                self._perception_frame_serial += 1
                self._latest_perception_frame_path = frame_path
            self._render_primary_scene(width, height)

        if self._hand_view_enabled and self._current_pose is not None:
            inset_w = max(80, int(width * 0.32))
            inset_h = max(80, int(height * 0.32))
            inset_x = width - inset_w - 18
            inset_y = 18
            glEnable(GL_SCISSOR_TEST)
            glScissor(inset_x, inset_y, inset_w, inset_h)
            glViewport(inset_x, inset_y, inset_w, inset_h)
            glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT)
            glDisable(GL_SCISSOR_TEST)
            hand_aspect = inset_w / max(1, inset_h)
            self._apply_hand_camera(hand_aspect)
            self._draw_scene(include_grid=False)
            if self._hand_capture_dir:
                self._capture_hand_frame(inset_x, inset_y, inset_w, inset_h)

    def _render_primary_scene(self, width: int, height: int) -> None:
        aspect = width / max(1, height)
        glViewport(0, 0, width, height)
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT)
        self._apply_main_camera(aspect)
        self._draw_scene(include_grid=True)

    def _render_perception_scene(self, window_width: int, window_height: int) -> tuple[int, int]:
        if self._perception_capture_resolution:
            capture_w, capture_h = self._perception_capture_resolution
        else:
            capture_w, capture_h = window_width, window_height
        capture_w = max(32, capture_w)
        capture_h = max(32, capture_h)
        glViewport(0, 0, capture_w, capture_h)
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT)
        aspect = capture_w / max(1, capture_h)
        view = self._apply_hand_camera(aspect, record=True)
        if view is not None:
            view.image_width = capture_w
            view.image_height = capture_h
            self._latest_perception_view = view
        self._draw_scene(include_grid=False)
        return capture_w, capture_h

    def _apply_scene_lighting(self) -> None:
        return

    def _apply_main_camera(self, aspect: float) -> None:
        glMatrixMode(GL_PROJECTION)
        glLoadIdentity()
        gluPerspective(45.0, aspect, 0.1, 12.0)
        glMatrixMode(GL_MODELVIEW)
        glLoadIdentity()
        cam_pitch_rad = math.radians(self._input.cam_pitch)
        cam_yaw_rad = math.radians(self._input.cam_yaw)
        cam_dist = self._input.cam_distance
        eye_x = cam_dist * math.cos(cam_pitch_rad) * math.cos(cam_yaw_rad)
        eye_y = cam_dist * math.cos(cam_pitch_rad) * math.sin(cam_yaw_rad)
        eye_z = cam_dist * math.sin(cam_pitch_rad)
        gluLookAt(eye_x, eye_y, eye_z, 0.0, 0.0, 0.4, 0.0, 0.0, 1.0)
        self._apply_scene_lighting()

    def _apply_hand_camera(self, aspect: float, *, record: bool = False) -> Optional[CameraView]:
        if self._current_pose is None:
            return None
        glMatrixMode(GL_PROJECTION)
        glLoadIdentity()
        fov_y_deg = 90.0
        gluPerspective(fov_y_deg, aspect, 0.05, 8.0)
        glMatrixMode(GL_MODELVIEW)
        glLoadIdentity()

        base_position = (
            self._latest_positions[0]
            if self._latest_positions is not None and len(self._latest_positions) > 0
            else np.array([0.0, 0.0, TABLE_HEIGHT])
        )
        base_yaw = float(self.state.joint_angles[0]) if self.state is not None else 0.0
        cy, sy = math.cos(base_yaw), math.sin(base_yaw)
        rotation_z = np.array([[cy, -sy, 0.0], [sy, cy, 0.0], [0.0, 0.0, 1.0]])

        camera_offset_body = np.array(HEAD_CAMERA_OFFSET, dtype=float)
        camera_offset = rotation_z @ camera_offset_body
        eye = base_position + camera_offset

        camera_pitch = math.radians(28.0)
        cp, sp = math.cos(camera_pitch), math.sin(camera_pitch)
        rotation_pitch = np.array([[cp, 0.0, sp], [0.0, 1.0, 0.0], [-sp, 0.0, cp]])
        orientation = rotation_z @ rotation_pitch

        focus_body = np.array([0.08, 0.0, 0.32])
        focus = base_position + rotation_z @ focus_body

        direction = focus - eye
        up = orientation @ np.array([0.0, 0.0, 1.0])
        direction_norm = np.linalg.norm(direction)
        if direction_norm < 1e-6:
            direction = np.array([1.0, 0.0, -0.15])
            direction_norm = np.linalg.norm(direction)
        direction /= direction_norm
        up = up - np.dot(up, direction) * direction
        up /= max(np.linalg.norm(up), 1e-6)
        right = np.cross(direction, up)
        right /= max(np.linalg.norm(right), 1e-6)
        up = np.cross(right, direction)
        up /= max(np.linalg.norm(up), 1e-6)

        center = eye + direction
        gluLookAt(
            eye[0],
            eye[1],
            eye[2],
            center[0],
            center[1],
            center[2],
            up[0],
            up[1],
            up[2],
        )
        self._apply_scene_lighting()
        view = CameraView(
            position=eye.copy(),
            forward=direction.copy(),
            up=up.copy(),
            right=right.copy(),
            fov_y=math.radians(fov_y_deg),
            aspect=float(aspect),
            image_width=0,
            image_height=0,
        )
        if record:
            self._latest_perception_view = view
        return view

    def _draw_scene(self, include_grid: bool) -> None:
        if include_grid:
            self._draw_table()
            self._draw_grid()
        if self._latest_positions is not None:
            self._draw_robot(self._latest_positions)
        if self._target_position is not None:
            self._draw_target(None)

    def _draw_table(self) -> None:
        table_thickness = 0.04
        glPushMatrix()
        glColor3f(0.72, 0.72, 0.78)
        glTranslatef(0.0, 0.0, TABLE_HEIGHT - table_thickness / 2.0)
        self._draw_box(2.2 * TABLE_EXTENT_X, 2.2 * TABLE_EXTENT_Y, table_thickness)
        glPopMatrix()

        border = 1.1
        glColor3f(0.78, 0.78, 0.82)
        glBegin(GL_LINE_STRIP)
        glVertex3f(-TABLE_EXTENT_X * border, -TABLE_EXTENT_Y * border, TABLE_HEIGHT + 1e-4)
        glVertex3f(-TABLE_EXTENT_X * border, TABLE_EXTENT_Y * border, TABLE_HEIGHT + 1e-4)
        glVertex3f(TABLE_EXTENT_X * border, TABLE_EXTENT_Y * border, TABLE_HEIGHT + 1e-4)
        glVertex3f(TABLE_EXTENT_X * border, -TABLE_EXTENT_Y * border, TABLE_HEIGHT + 1e-4)
        glVertex3f(-TABLE_EXTENT_X * border, -TABLE_EXTENT_Y * border, TABLE_HEIGHT + 1e-4)
        glEnd()

    def _draw_grid(self) -> None:
        size = 1.2
        step = 0.1
        glColor3f(0.15, 0.15, 0.22)
        glLineWidth(1.0)
        glBegin(GL_LINES)
        for i in np.arange(-size, size + step, step):
            glVertex3f(i, -size, 0.0)
            glVertex3f(i, size, 0.0)
            glVertex3f(-size, i, 0.0)
            glVertex3f(size, i, 0.0)
        glEnd()

    def _draw_robot(self, positions: np.ndarray) -> None:
        quadric = gluNewQuadric()
        self._draw_robot_base(quadric)

        link_colors = JOINT_COLORS
        num_links = len(positions) - 1
        for idx in range(num_links):
            start = positions[idx]
            end = positions[idx + 1]
            color = link_colors[idx % len(link_colors)]
            highlight = idx == self._input.selected_joint
            self._draw_link_segment(quadric, start, end, color=color, highlight=highlight)

        for idx, (x, y, z) in enumerate(positions):
            glPushMatrix()
            glTranslatef(x, y, z)
            if idx == 0:
                radius = 0.055
            elif idx == len(positions) - 1:
                radius = 0.038
            else:
                radius = 0.045
            if idx == self._input.selected_joint + 1:
                glColor3f(0.2, 0.95, 0.4)
            else:
                color = link_colors[idx % len(link_colors)]
                glColor3f(*color)
            gluSphere(quadric, radius, 24, 24)
            glPopMatrix()

        if self.robot.dof >= 6:
            self._draw_gripper(quadric)
        gluDeleteQuadric(quadric)

    def _draw_robot_base(self, quadric) -> None:
        base_height = max(0.05, self.robot.base_height - TABLE_HEIGHT)
        base_radius = 0.14
        glPushMatrix()
        glTranslatef(0.0, 0.0, TABLE_HEIGHT)
        glColor3f(0.58, 0.60, 0.66)
        gluCylinder(quadric, base_radius, base_radius * 0.94, base_height, 36, 1)
        glPushMatrix()
        gluDisk(quadric, 0.0, base_radius, 36, 1)
        glTranslatef(0.0, 0.0, base_height)
        gluDisk(quadric, 0.0, base_radius * 0.94, 36, 1)
        glPopMatrix()
        glPopMatrix()

    def _draw_link_segment(self, quadric, start: np.ndarray, end: np.ndarray, *, color: tuple[float, float, float], highlight: bool) -> None:
        vector = end - start
        length = float(np.linalg.norm(vector))
        if length < 1e-6:
            return
        direction = vector / length
        up = np.array([0.0, 0.0, 1.0])
        axis = np.cross(up, direction)
        axis_norm = float(np.linalg.norm(axis))
        dot = float(np.clip(np.dot(up, direction), -1.0, 1.0))
        angle = math.degrees(math.acos(dot)) if axis_norm > 1e-6 else 0.0

        glPushMatrix()
        glTranslatef(*start)
        if axis_norm > 1e-6:
            glRotatef(angle, *(axis / axis_norm))
        radius = 0.03 if highlight else 0.027
        if highlight:
            glColor3f(0.95, 0.6, 0.2)
        else:
            glColor3f(*color)
        gluCylinder(quadric, radius, radius * 0.9, length, 28, 1)
        glPushMatrix()
        gluDisk(quadric, 0.0, radius, 28, 1)
        glTranslatef(0.0, 0.0, length)
        gluDisk(quadric, 0.0, radius * 0.9, 28, 1)
        glPopMatrix()
        glPopMatrix()

    def _draw_gripper(self, quadric) -> None:
        if self._current_pose is None:
            return
        transform = np.eye(4, dtype=np.float32)
        transform[:3, :3] = self._current_pose.rotation.astype(np.float32)
        transform[:3, 3] = self._current_pose.position.astype(np.float32)

        glPushMatrix()
        glMultMatrixf(transform.T.flatten())
        wrist_length = WRIST_LENGTH
        wrist_radius = 0.026
        glColor3f(0.64, 0.68, 0.82)
        gluCylinder(quadric, wrist_radius, wrist_radius * 0.92, wrist_length, 20, 1)
        glPushMatrix()
        gluDisk(quadric, 0.0, wrist_radius, 20, 1)
        glTranslatef(0.0, 0.0, wrist_length)
        gluDisk(quadric, 0.0, wrist_radius * 0.92, 20, 1)
        glPopMatrix()

        glTranslatef(wrist_length, 0.0, 0.0)
        palm_length = PALM_LENGTH
        palm_width = 0.055
        palm_thickness = 0.03
        glPushMatrix()
        glColor3f(0.8, 0.83, 0.9)
        glTranslatef(palm_length / 2.0, 0.0, 0.0)
        self._draw_box(palm_length, palm_width, palm_thickness)
        glPopMatrix()

        finger_length = FINGER_LENGTH
        finger_width = 0.018
        finger_thickness = 0.022
        offset = self._gripper_open_amount / 2.0

        for direction in (1.0, -1.0):
            glPushMatrix()
            glTranslatef(palm_length + finger_length / 2.0, direction * (offset + finger_width / 2.0), 0.0)
            glColor3f(0.92, 0.92, 0.96)
            self._draw_box(finger_length, finger_width, finger_thickness)
            glTranslatef(finger_length / 2.0, 0.0, 0.0)
            self._draw_box(FINGER_TIP_LENGTH, finger_width, finger_thickness * 0.6)
            glPopMatrix()

        glPopMatrix()

    def _draw_box(self, length: float, width: float, height: float) -> None:
        hx = length / 2.0
        hy = width / 2.0
        hz = height / 2.0
        glBegin(GL_QUADS)
        # +X face
        glNormal3f(1.0, 0.0, 0.0)
        glVertex3f(hx, -hy, -hz)
        glVertex3f(hx, hy, -hz)
        glVertex3f(hx, hy, hz)
        glVertex3f(hx, -hy, hz)
        # -X face
        glNormal3f(-1.0, 0.0, 0.0)
        glVertex3f(-hx, -hy, -hz)
        glVertex3f(-hx, -hy, hz)
        glVertex3f(-hx, hy, hz)
        glVertex3f(-hx, hy, -hz)
        # +Y face
        glNormal3f(0.0, 1.0, 0.0)
        glVertex3f(-hx, hy, -hz)
        glVertex3f(-hx, hy, hz)
        glVertex3f(hx, hy, hz)
        glVertex3f(hx, hy, -hz)
        # -Y face
        glNormal3f(0.0, -1.0, 0.0)
        glVertex3f(-hx, -hy, -hz)
        glVertex3f(hx, -hy, -hz)
        glVertex3f(hx, -hy, hz)
        glVertex3f(-hx, -hy, hz)
        # +Z face
        glNormal3f(0.0, 0.0, 1.0)
        glVertex3f(-hx, -hy, hz)
        glVertex3f(hx, -hy, hz)
        glVertex3f(hx, hy, hz)
        glVertex3f(-hx, hy, hz)
        # -Z face
        glNormal3f(0.0, 0.0, -1.0)
        glVertex3f(-hx, -hy, -hz)
        glVertex3f(-hx, hy, -hz)
        glVertex3f(hx, hy, -hz)
        glVertex3f(hx, -hy, -hz)
        glEnd()

    def _draw_target(self, pose: Optional[Pose]) -> None:
        _ = pose
        glPushMatrix()
        glTranslatef(*self._target_position)
        quad = gluNewQuadric()
        glColor3f(0.7, 0.1, 0.1)
        gluSphere(quad, 0.07, 28, 28)
        glPopMatrix()
        gluDeleteQuadric(quad)

    def _capture_hand_frame(self, x: int, y: int, width: int, height: int) -> None:
        if not self._hand_capture_dir:
            return
        glReadBuffer(GL_BACK)
        glPixelStorei(GL_PACK_ALIGNMENT, 1)
        glFinish()
        buffer = glReadPixels(x, y, width, height, GL_RGBA, GL_UNSIGNED_BYTE)
        if buffer is None:
            return
        frame = np.frombuffer(buffer, dtype=np.uint8).copy().reshape((height, width, 4))
        frame = np.flipud(frame)
        rgb = frame[:, :, :3]
        if Image is None:
            return
        image = Image.fromarray(rgb, "RGB")
        filename = self._hand_capture_dir / f"hand_{self._hand_frame_index:05d}.png"
        image.save(filename)
        self._hand_frame_index += 1

    def _capture_perception_frame(
        self,
        width: int,
        height: int,
        *,
        save_to_disk: bool,
    ) -> tuple[Optional[np.ndarray], Optional[Path]]:
        glReadBuffer(GL_BACK)
        glPixelStorei(GL_PACK_ALIGNMENT, 1)
        glFinish()
        buffer = glReadPixels(0, 0, width, height, GL_RGBA, GL_UNSIGNED_BYTE)
        if buffer is None:
            return None, None
        frame = np.frombuffer(buffer, dtype=np.uint8).copy().reshape((height, width, 4))
        frame = np.flipud(frame)
        rgb = frame[:, :, :3]
        saved_path: Optional[Path] = None
        if save_to_disk and self._perception_capture_dir and Image is not None:
            image = Image.fromarray(rgb, "RGB")
            filename = self._perception_capture_dir / f"head_{self._perception_frame_index:05d}.png"
            image.save(filename)
            self._perception_frame_index += 1
            saved_path = filename
        return rgb.copy(), saved_path

    # ------------------------------------------------------------------
    # Event handlers
    def _on_key(self, window: glfw._GLFWwindow, key: int, scancode: int, action: int, mods: int) -> None:
        if action not in (glfw.PRESS, glfw.REPEAT):
            return
        if key == glfw.KEY_ESCAPE:
            self._should_quit = True
            return
        if key == glfw.KEY_SPACE:
            self._input.auto_mode = not self._input.auto_mode
            if self._input.auto_mode:
                self.trajectory_controller.state = self.state
                self._time_in_traj = 0.0
                self._trajectory_complete = False
            return
        if key == glfw.KEY_R:
            self.state = RobotState.zeros()
            self._cancel_gemini_motion()
            if self._gemini_single_shot:
                self._gemini_request_made = False
                if not self._apply_target_position(randomize=True):
                    self._set_status_message(
                        "Unable to find a reachable randomized target.",
                        duration=4.0,
                        log_to_console=True,
                    )
            else:
                if not self._apply_target_position(position=self._target_position, randomize=False):
                    self._set_status_message(
                        "Current target unreachable from reset pose; randomizing.",
                        duration=4.0,
                        log_to_console=True,
                    )
                    self._apply_target_position(randomize=True)
            self.trajectory_controller.state = self.state
            self._time_in_traj = 0.0
            if self._gemini_single_shot:
                self._set_status_message(
                    "Gemini single-shot reset: new target at "
                    f"({self._target_position[0]:+.3f}, {self._target_position[1]:+.3f}, {self._target_position[2]:+.3f})",
                    duration=4.0,
                    log_to_console=True,
                )
            return
        if key in (glfw.KEY_1, glfw.KEY_2, glfw.KEY_3, glfw.KEY_4, glfw.KEY_5, glfw.KEY_6, glfw.KEY_7, glfw.KEY_8, glfw.KEY_9):
            desired = key - glfw.KEY_1
            if desired < self.robot.dof:
                self._input.selected_joint = desired
            else:
                self._set_status_message(
                    f"Joint {desired + 1} not available; robot has {self.robot.dof} joints.",
                    duration=3.0,
                    log_to_console=True,
                )
            return
        if key == glfw.KEY_EQUAL or key == glfw.KEY_KP_ADD:
            self._input.joint_speed = math.radians(min(120.0, math.degrees(self._input.joint_speed) + 15.0))
            return
        if key == glfw.KEY_MINUS or key == glfw.KEY_KP_SUBTRACT:
            self._input.joint_speed = math.radians(max(15.0, math.degrees(self._input.joint_speed) - 15.0))
            return
        if key == glfw.KEY_V:
            self._hand_view_enabled = not self._hand_view_enabled
            self._input.hand_view_enabled = self._hand_view_enabled
            return
        if key == glfw.KEY_G and self._gemini_agent is not None:
            if not self._gemini_agent_enabled:
                self._gemini_agent_enabled = True
                self._gemini_sequence_complete = False
                self._gemini_request_made = False
                self._cancel_gemini_motion()
                self._request_gripper_open()
                if self._gemini_single_shot:
                    if self._target_position is None:
                        if not self._apply_target_position(randomize=True):
                            self._set_status_message(
                                "Unable to find a reachable target for Gemini single-shot.",
                                duration=4.0,
                                log_to_console=True,
                            )
                            self._gemini_agent_enabled = False
                        else:
                            self._set_status_message(
                                "Gemini single-shot armed with fallback target.",
                                duration=4.0,
                                log_to_console=True,
                            )
                    elif self._target_position is not None:
                        x, y, z = self._target_position
                        self._set_status_message(
                            "Gemini single-shot armed. Target locked at "
                            f"({x:+.3f}, {y:+.3f}, {z:+.3f}).",
                            duration=3.5,
                            log_to_console=True,
                        )
                else:
                    self._set_status_message(
                        "Gemini control engaged. Press G again to stop.",
                        duration=3.5,
                        log_to_console=True,
                    )
            else:
                self._gemini_agent_enabled = False
                self._cancel_gemini_motion()
                self._set_status_message(
                    "Gemini control halted by user.",
                    duration=3.0,
                    log_to_console=True,
                )
            status = "enabled" if self._gemini_agent_enabled else "disabled"
            self._print_controls(force=True)
            print(f"Gemini agent {status}.")
            return
        if key == glfw.KEY_A and self._gemini_agent is not None:
            self._gemini_agent_enabled = True
            self._gemini_sequence_complete = False
            self._gemini_request_made = False
            self._cancel_gemini_motion()
            self._request_gripper_open()
            self._set_status_message(
                "Gemini capture requested. Press G to stop.",
                duration=3.0,
                log_to_console=True,
            )
            return
        if key in (
            glfw.KEY_KP_8,
            glfw.KEY_KP_2,
            glfw.KEY_KP_4,
            glfw.KEY_KP_6,
            glfw.KEY_KP_7,
            glfw.KEY_KP_1,
            glfw.KEY_KP_5,
        ):
            step_xy = 0.02
            step_z = 0.02
            if key == glfw.KEY_KP_8:
                self._adjust_target_position(dy=step_xy)
            elif key == glfw.KEY_KP_2:
                self._adjust_target_position(dy=-step_xy)
            elif key == glfw.KEY_KP_4:
                self._adjust_target_position(dx=-step_xy)
            elif key == glfw.KEY_KP_6:
                self._adjust_target_position(dx=step_xy)
            elif key == glfw.KEY_KP_7:
                self._adjust_target_position(dz=step_z)
            elif key == glfw.KEY_KP_1:
                self._adjust_target_position(dz=-step_z)
            elif key == glfw.KEY_KP_5:
                if self._apply_target_position(randomize=True):
                    self._set_status_message(
                        "Target randomized via numpad: "
                        f"({self._target_position[0]:+.3f}, {self._target_position[1]:+.3f}, {self._target_position[2]:+.3f})",
                        duration=4.0,
                        log_to_console=True,
                    )
            return
        if key == glfw.KEY_B:
            if self._target_position is not None:
                x, y, z = self._target_position
                self._set_status_message(
                    f"Target sphere position: x={x:+.3f} m, y={y:+.3f} m, z={z:+.3f} m",
                    duration=5.0,
                    log_to_console=True,
                )
            else:
                self._set_status_message(
                    "Target sphere position unavailable.",
                    duration=3.0,
                    log_to_console=True,
                )
            return
        if key == glfw.KEY_H:
            self._print_controls(force=True)
            return

    # ------------------------------------------------------------------
    def _print_controls(self, force: bool = False) -> None:
        if self._controls_printed and not force:
            return
        self._controls_printed = True
        lines = [
            "RobotSim OpenGL controls:",
            "  Arrow keys  : Orbit camera",
            "  Page Up/Down: Zoom camera",
            "  Space       : Toggle auto trajectory",
            "  R           : Reset robot pose",
            "  NumPad 8/2  : Move target +Y / -Y",
            "  NumPad 4/6  : Move target -X / +X",
            "  NumPad 7/1  : Move target +Z / -Z",
            "  NumPad 5    : Randomize target",
            "  B           : Print target sphere position",
            "  1-6         : Select joint",
            "  J / K       : Nudge selected joint",
            "  +/-         : Adjust joint speed",
            "  V           : Toggle hand camera inset",
            "  H           : Reprint this help",
            "  Esc         : Quit",
        ]
        if self._gemini_agent is not None:
            insert_at = len(lines) - 2
            lines.insert(insert_at, "  G           : Start/stop Gemini control loop")
            lines.insert(insert_at + 1, "  A           : Trigger another Gemini capture")
        print("\n".join(lines))

    # ------------------------------------------------------------------
    def _create_trajectory_for_target(self, target: np.ndarray) -> Trajectory:
        orientation_rpy = tuple(self._target_orientation)
        orientation_pose = Pose.from_xyz_rpy((0.0, 0.0, 0.0), orientation_rpy)
        rotation = orientation_pose.rotation
        tip_offset = rotation @ np.array([TIP_FORWARD_OFFSET, 0.0, 0.0])

        hover_height = TABLE_HEIGHT + 0.24
        safe_height = hover_height + 0.06
        target_x = float(target[0])
        target_y = float(target[1])
        target_z = float(target[2])

        grasp_height = max(TABLE_HEIGHT + TARGET_HEIGHT_OFFSET, target_z)
        approach_height = grasp_height + 0.05

        approach_axis = rotation @ np.array([1.0, 0.0, 0.0])
        backoff_direction = approach_axis.copy()
        backoff_direction[2] = 0.0
        if np.linalg.norm(backoff_direction) < 1e-6:
            backoff_direction = np.array([1.0, 0.0, 0.0])
        else:
            backoff_direction = backoff_direction / np.linalg.norm(backoff_direction)

        pre_hover_backoff = 0.12

        def pose_for_tip(tip_position: np.ndarray) -> Pose:
            wrist_position = tip_position - tip_offset
            return Pose.from_xyz_rpy(tuple(wrist_position), orientation_rpy)

        hover_tip = np.array([target_x, target_y, hover_height], dtype=float)
        approach_tip = np.array([target_x, target_y, approach_height], dtype=float)
        grasp_tip = np.array([target_x, target_y, grasp_height], dtype=float)
        pre_hover_tip = hover_tip - backoff_direction * pre_hover_backoff
        start_tip = np.array([target_x, target_y, safe_height], dtype=float) - backoff_direction * (pre_hover_backoff + 0.18)

        pre_hover = pose_for_tip(pre_hover_tip)
        hover = pose_for_tip(hover_tip)
        approach = pose_for_tip(approach_tip)
        grasp = pose_for_tip(grasp_tip)
        start = pose_for_tip(start_tip)

        leg1 = Trajectory.linear_path(start, pre_hover, samples=12, duration=2.5)
        leg2 = Trajectory.linear_path(pre_hover, hover, samples=10, duration=2.2)
        leg3 = Trajectory.linear_path(hover, approach, samples=10, duration=2.0)
        leg4 = Trajectory.linear_path(approach, grasp, samples=8, duration=1.4)

        segments = [leg1, leg2, leg3, leg4]
        keyframes: list[tuple[float, Pose]] = []
        elapsed = 0.0
        for segment in segments:
            if not keyframes:
                keyframes.extend(segment.keyframes)
            else:
                keyframes.extend((t + elapsed, pose) for t, pose in segment.keyframes[1:])
            elapsed += segment.duration()
        return Trajectory(keyframes)


def main(argv: Optional[list[str]] = None) -> None:
    parser = argparse.ArgumentParser(description="OpenGL viewer for RobotSim")
    parser.add_argument("--width", type=int, default=960, help="Window width in pixels")
    parser.add_argument("--height", type=int, default=720, help="Window height in pixels")
    parser.add_argument(
        "--joint-count",
        type=int,
        default=None,
        help="Number of robot joints to simulate (1-6). Default matches the base arm.",
    )
    parser.add_argument("--auto", action="store_true", help="Start in automatic trajectory mode")
    parser.add_argument("--no-hand-view", action="store_true", help="Disable the hand camera viewport")
    parser.add_argument(
        "--free-wrist",
        action="store_true",
        help="Relax wrist orientation when searching for reachable targets",
    )
    parser.add_argument(
        "--free-joints",
        action="store_true",
        help="Alias for --free-wrist; also extends relaxed orientation to Gemini planning",
    )
    parser.add_argument("--capture-dir", type=Path, help="Directory to save hand camera frames")
    parser.add_argument(
        "--top-capture-dir",
        type=Path,
        help="Directory to save head-camera RGB frames (used for Gemini perception)",
    )
    parser.add_argument(
        "--top-capture-size",
        type=int,
        nargs=2,
        metavar=("WIDTH", "HEIGHT"),
        help="Resolution for head-camera captures (defaults to window size)",
    )
    parser.add_argument("--log", type=Path, help="Optional JSON log output path")
    parser.add_argument("--gemini", action="store_true", help="Enable Gemini Robotics agent control")
    parser.add_argument("--gemini-api-key", type=str, help="Gemini API key (overrides environment)")
    parser.add_argument(
        "--gemini-model",
        type=str,
        default="gemini-robotics-er-1.5-preview",
        help="Gemini Robotics model identifier",
    )
    parser.add_argument(
        "--gemini-interval",
        type=float,
        default=1.0,
        help="Minimum seconds between Gemini API calls",
    )
    parser.add_argument(
        "--gemini-hover-height",
        type=float,
        default=0.12,
        help="Hover height (meters) above the table for Gemini-guided poses",
    )
    parser.add_argument(
        "--gemini-thinking-budget",
        type=int,
        default=0,
        help="Thinking budget to allocate to the Gemini Robotics model",
    )
    parser.add_argument(
        "--gemini-target-label",
        type=str,
        default="target",
        help="Label Gemini should match when parsing detections",
    )
    parser.add_argument(
        "--gemini-joint-count",
        type=int,
        default=None,
        help="Override the joint count mentioned in the Gemini prompt",
    )
    parser.add_argument(
        "--gemini-mode",
        choices=("single", "stream"),
        default="single",
        help="Gemini call cadence: single = once per reset (default), stream = continuous",
    )
    args = parser.parse_args(argv)

    top_res = tuple(args.top_capture_size) if args.top_capture_size else None
    robot_model = RobotModel.default(args.joint_count)
    gemini_agent: Optional["GeminiRobotAgent"] = None
    if args.gemini:
        from .gemini_agent import GeminiRobotAgent

        api_key = (
            args.gemini_api_key
            or os.getenv("GEMINI_API_KEY")
            or os.getenv("GENAI_API_KEY")
            or os.getenv("GOOGLE_API_KEY")
        )
        if api_key is None:
            raise RuntimeError(
                "A Gemini API key is required. Provide --gemini-api-key or set GEMINI_API_KEY."
            )
        resolved_joint_count = args.gemini_joint_count
        if resolved_joint_count is None:
            resolved_joint_count = args.joint_count if args.joint_count is not None else robot_model.dof
        gemini_agent = GeminiRobotAgent(
            robot=robot_model,
            api_key=api_key,
            model_id=args.gemini_model,
            min_request_interval=args.gemini_interval,
            hover_height=args.gemini_hover_height,
            thinking_budget=args.gemini_thinking_budget,
            target_label=args.gemini_target_label,
            allow_free_orientation=(args.free_wrist or args.free_joints),
            joint_count=resolved_joint_count,
        )

    app = OpenGLArmApp(
        robot=robot_model,
        window_size=(args.width, args.height),
        log_path=args.log,
        enable_hand_view=not args.no_hand_view,
        allow_free_wrist=(args.free_wrist or args.free_joints),
        hand_capture_dir=args.capture_dir,
        top_capture_dir=args.top_capture_dir,
        top_capture_resolution=top_res,
        gemini_agent=gemini_agent,
        gemini_single_shot=(args.gemini_mode == "single"),
    )
    if args.auto:
        app._input.auto_mode = True
        app.trajectory_controller.state = app.state
        app._time_in_traj = 0.0
    app.run()


if __name__ == "__main__":
    main()
