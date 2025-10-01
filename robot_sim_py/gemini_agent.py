"""Gemini Robotics integration utilities for the RobotSim OpenGL viewer."""

from __future__ import annotations

import io
import json
import logging
import math
import re
import time
from dataclasses import dataclass, field
from pathlib import Path
from typing import Optional, Sequence

import numpy as np
from PIL import Image

from .camera import CameraView
from .robot import Pose, RobotModel, RobotState
from .scene import (
    TABLE_HEIGHT,
    TABLE_EXTENT_X,
    TABLE_EXTENT_Y,
    TARGET_HEIGHT_OFFSET,
    TIP_FORWARD_OFFSET,
    WORKSPACE_X_BOUNDS,
    WORKSPACE_Y_BOUNDS,
)
from .reachability import DEFAULT_ORIENTATION, find_feasible_orientation

try:  # pragma: no cover - import guard
    from google import genai  # type: ignore[import]
    from google.genai import types as genai_types  # type: ignore[import]
except ImportError as exc:  # pragma: no cover - handled at runtime
    raise RuntimeError(
        "google-genai must be installed to use the GeminiRobotAgent"
    ) from exc

LOGGER = logging.getLogger(__name__)

_TIP_FORWARD_VECTOR = np.array([TIP_FORWARD_OFFSET, 0.0, 0.0], dtype=float)

# Note: Double braces {{}} are required to escape literal braces in Python format strings.
# This prevents KeyError when formatting the template with joint_count while preserving
# the JSON structure in the prompt sent to the Gemini model.
_GEMINI_PROMPT_TEMPLATE = (
    "You are assisting a {joint_count} degree-of-freedom robot arm that sees the workspace from a "
    "head-mounted RGB camera. Identify the small red target object on the table and return its "
    "location in the image. Respond only with JSON in the format: "
    "[{{\"point\": [y, x], \"label\": \"target\"}}]. The coordinates must be integers "
    "between 0 and 1000 inclusive. If the target is not visible, return an empty JSON list []."
)


@dataclass
class GeminiRobotAgent:
    """Helper that queries a Gemini Robotics model for a head-camera target location.

    The agent periodically sends the latest head-mounted camera frame to the Gemini Robotics
    model and converts the returned pixel coordinates into a Cartesian target in the robot's
    workspace. It then solves inverse kinematics to produce a reachable `RobotState` for
    the manipulator.
    """

    robot: RobotModel
    hover_height: float = TARGET_HEIGHT_OFFSET
    min_request_interval: float = 1.0
    model_id: str = "gemini-robotics-er-1.5-preview"
    target_label: str = "target"
    thinking_budget: int = 0
    joint_count: Optional[int] = None
    table_extent: Sequence[float] = (TABLE_EXTENT_X, TABLE_EXTENT_Y)
    quota_backoff: float = 20.0
    client: Optional[genai.Client] = None
    api_key: Optional[str] = None
    allow_free_orientation: bool = False
    last_tip_world: Optional[np.ndarray] = field(default=None, init=False)
    last_camera_projection: Optional[np.ndarray] = field(default=None, init=False)
    last_projection_clamped: bool = field(default=False, init=False)
    last_mapping_source: str = field(default="table_bounds", init=False)
    last_hover_pose: Optional[Pose] = field(default=None, init=False)
    last_orientation: tuple[float, float, float] = field(default=DEFAULT_ORIENTATION, init=False)
    _last_request_time: float = field(default=0.0, init=False)
    _last_frame_serial: int = field(default=-1, init=False)
    _cooldown_until: float = field(default=0.0, init=False)
    _last_error: str = field(default="", init=False)

    def __post_init__(self) -> None:
        if self.client is not None:
            self._client = self.client
        else:
            client_kwargs = {}
            if self.api_key:
                client_kwargs["api_key"] = self.api_key
            self._client = genai.Client(**client_kwargs)
        self.last_response_text: str = ""
        self.last_points: list[dict[str, object]] = []
        LOGGER.info(
            "Initialized GeminiRobotAgent with model=%s, joints=%d, tip_height=%.3f m",
            self.model_id,
            self._resolve_joint_count(),
            TABLE_HEIGHT + max(self.hover_height, TARGET_HEIGHT_OFFSET),
        )

    def compute_next_state(
        self,
        current_state: RobotState,
        top_frame: Optional[np.ndarray],
        frame_serial: int,
        camera_view: Optional[CameraView] = None,
    ) -> Optional[RobotState]:
        """Return a new robot state based on the latest Gemini observation.

        Parameters
        ----------
        current_state:
            The current joint position state.
        top_frame:
            Latest RGB image from the perception camera (shape HxWx3, dtype uint8).
        frame_serial:
            Monotonic identifier that increments each time a new top frame is captured.
        camera_view:
            Optional description of the camera pose and intrinsics for projecting detections.
        """

        if top_frame is None:
            LOGGER.debug("GeminiRobotAgent.compute_next_state called without a top frame")
            return None
        if frame_serial == self._last_frame_serial:
            return None

        now = time.perf_counter()
        if now < self._cooldown_until:
            remaining = self._cooldown_until - now
            LOGGER.debug("Gemini request skipped due to cooldown (%.2fs remaining)", remaining)
            return None
        if now - self._last_request_time < self.min_request_interval:
            return None

        image_bytes = self._encode_image(top_frame)
        response_points, raw_text = self._request_target_point(image_bytes)
        self.last_response_text = raw_text
        self.last_points = response_points
        if not response_points and not raw_text:
            # Preserve the cooldown if one was scheduled during the request.
            return None
        if not response_points:
            return None

        target_point = self._select_point(response_points)
        if target_point is None:
            return None

        self.last_hover_pose = None
        tip_position = self._normalized_to_world(target_point, camera_view=camera_view)
        self.last_tip_world = tip_position.copy()

        if self.allow_free_orientation:
            success, orientation = find_feasible_orientation(
                self.robot,
                current_state,
                tip_position,
                allow_free_orientation=True,
                last_orientation=self.last_orientation,
            )
            if not success:
                LOGGER.warning(
                    "Gemini target at world=(%.3f, %.3f, %.3f) rejected: orientation search failed",
                    tip_position[0],
                    tip_position[1],
                    tip_position[2],
                )
                return None
            self.last_orientation = orientation
            # Use the approach height that was tested in find_feasible_orientation
            # to ensure consistency between orientation search and actual IK solve
            hover_padding = 0.05
            approach_height = max(TABLE_HEIGHT + hover_padding, float(tip_position[2]) + hover_padding)
            hover_tip_position = np.array([tip_position[0], tip_position[1], approach_height], dtype=float)
        else:
            orientation = DEFAULT_ORIENTATION
            self.last_orientation = orientation
            hover_tip_position = tip_position

        hover_pose = self._hover_pose_for_tip(hover_tip_position, orientation)
        self.last_hover_pose = hover_pose

        try:
            next_state = self.robot.inverse_kinematics(hover_pose, initial=current_state)
        except RuntimeError as exc:
            LOGGER.warning(
                "Gemini IK failed for point=%s mapped to world=(%.3f, %.3f, %.3f): %s",
                target_point,
                tip_position[0],
                tip_position[1],
                tip_position[2],
                exc,
            )
            return None

        self._last_request_time = now
        self._last_frame_serial = frame_serial
        LOGGER.info(
            "Gemini target acquired at point=%s mapped to world=(%.3f, %.3f, %.3f)",
            target_point,
            tip_position[0],
            tip_position[1],
            tip_position[2],
        )
        return next_state

    def _encode_image(self, frame: np.ndarray) -> bytes:
        if frame.dtype != np.uint8:
            frame = frame.astype(np.uint8)
        image = Image.fromarray(frame)
        buffer = io.BytesIO()
        image.save(buffer, format="JPEG", quality=92)
        return buffer.getvalue()

    # Public helpers -------------------------------------------------
    def infer_from_frame(self, frame: np.ndarray) -> tuple[list[dict[str, object]], str]:
        """Return Gemini detections for a raw RGB frame without updating IK state."""

        image_bytes = self._encode_image(frame)
        return self._request_target_point(image_bytes)

    def infer_from_file(self, image_path: Path) -> tuple[list[dict[str, object]], str]:
        """Load an image file from disk and query Gemini for detections."""

        with Image.open(image_path) as img:
            rgb = img.convert("RGB")
            frame = np.array(rgb)
        return self.infer_from_frame(frame)

    def _request_target_point(self, image_bytes: bytes) -> tuple[list[dict[str, object]], str]:
        contents = [
            genai_types.Part.from_bytes(data=image_bytes, mime_type="image/jpeg"),
            self._build_prompt(),
        ]
        LOGGER.info("Gemini prompt: %s", contents[1])
        config = genai_types.GenerateContentConfig(
            temperature=0.2,
            thinking_config=genai_types.ThinkingConfig(thinking_budget=self.thinking_budget),
        )
        try:
            response = self._client.models.generate_content(
                model=self.model_id,
                contents=contents,
                config=config,
            )
        except (ConnectionError, TimeoutError) as exc:  # pragma: no cover - network failure
            self._handle_request_exception(exc)
            LOGGER.error("Gemini API network error: %s", exc)
            return [], ""
        except ValueError as exc:  # pragma: no cover - invalid request
            LOGGER.error("Gemini API invalid request: %s", exc)
            return [], ""
        except Exception as exc:  # pragma: no cover - unexpected errors
            self._handle_request_exception(exc)
            LOGGER.error("Gemini API unexpected error: %s", exc)
            return [], ""

        raw_text = response.text or ""
        if not raw_text.strip():
            LOGGER.warning("Gemini response contained no text")
            return [], ""

        parsed = self._parse_response_text(raw_text)
        if parsed is not None:
            self._last_error = ""
            LOGGER.info("Gemini JSON response: %s", json.dumps(parsed, ensure_ascii=False))
            return parsed, raw_text

        LOGGER.error("Gemini response did not contain a valid JSON list: %s", raw_text)
        if raw_text:
            LOGGER.info("Gemini raw response text: %s", raw_text)
        return [], raw_text

    def _build_prompt(self) -> str:
        return _GEMINI_PROMPT_TEMPLATE.format(joint_count=self._resolve_joint_count())

    def prompt_text(self) -> str:
        """Return the current Gemini prompt string."""

        return self._build_prompt()

    def _resolve_joint_count(self) -> int:
        value = self.joint_count if self.joint_count is not None else self.robot.dof
        return max(1, int(value))

    def _parse_response_text(self, raw_text: str) -> Optional[list[dict[str, object]]]:
        """Attempt to parse Gemini text output into a JSON list of detections."""

        candidates: list[str] = []
        text = raw_text.strip()
        if text:
            candidates.append(text)

        if "```" in raw_text:
            for segment in raw_text.split("```"):
                cleaned = segment.strip()
                if not cleaned:
                    continue
                if cleaned.lower().startswith("json"):
                    cleaned = cleaned[4:].lstrip()
                candidates.append(cleaned)

        start = raw_text.find("[")
        end = raw_text.rfind("]")
        if start != -1 and end != -1 and start < end:
            candidates.append(raw_text[start : end + 1])

        for candidate in candidates:
            try:
                parsed = json.loads(candidate)
            except json.JSONDecodeError:
                continue
            if isinstance(parsed, list):
                return parsed
        return None

    def _handle_request_exception(self, exc: Exception) -> None:
        message = str(exc)
        self._last_error = message
        if self._is_quota_error(message):
            backoff = max(0.0, float(self.quota_backoff))
            self._cooldown_until = time.perf_counter() + backoff
            LOGGER.warning(
                "Gemini quota issue detected; backing off for %.1f seconds", backoff
            )

    @staticmethod
    def _is_quota_error(message: str) -> bool:
        lower = message.lower()
        return "quota" in lower or "429" in lower or "rate limit" in lower

    def _select_point(self, items: Sequence[dict[str, object]]) -> Optional[Sequence[float]]:
        for item in items:
            label = str(item.get("label", "")).lower()
            if self.target_label.lower() in label:
                point = item.get("point")
                if isinstance(point, Sequence) and len(point) == 2:
                    return point
        if items:
            first = items[0].get("point")
            if isinstance(first, Sequence) and len(first) == 2:
                LOGGER.debug("No matching label found; using first point %s", first)
                return first
        LOGGER.warning("Gemini response did not include usable points: %s", items)
        return None

    def _normalized_to_world(
        self,
        point: Sequence[float],
        *,
        camera_view: Optional[CameraView] = None,
    ) -> np.ndarray:
        y_norm, x_norm = [float(v) / 1000.0 for v in point]
        tip_height = TABLE_HEIGHT + max(self.hover_height, TARGET_HEIGHT_OFFSET)

        x_min, x_max = WORKSPACE_X_BOUNDS
        y_min, y_max = WORKSPACE_Y_BOUNDS

        self.last_mapping_source = "table_bounds"
        self.last_projection_clamped = False
        self.last_camera_projection = None

        if camera_view is not None:
            projected = self._project_camera_point(x_norm, y_norm, camera_view)
            if projected is not None:
                result = projected.astype(float, copy=True)
                self.last_camera_projection = result.copy()
                self.last_mapping_source = "camera"
                if not (x_min <= result[0] <= x_max) or not (y_min <= result[1] <= y_max):
                    LOGGER.debug(
                        "Projected point outside workspace: (%.3f, %.3f) m; clamping to bounds.",
                        result[0],
                        result[1],
                    )
                    result[0] = float(np.clip(result[0], x_min, x_max))
                    result[1] = float(np.clip(result[1], y_min, y_max))
                    self.last_projection_clamped = True
                result[2] = tip_height
                return result
            LOGGER.debug("Camera projection unavailable; falling back to table-normal mapping.")
            self.last_mapping_source = "camera_fallback"

        span_x = x_max - x_min
        span_y = y_max - y_min
        world_x = x_min + np.clip(x_norm, 0.0, 1.0) * span_x
        world_y = y_max - np.clip(y_norm, 0.0, 1.0) * span_y
        return np.array([world_x, world_y, tip_height], dtype=float)

    def _project_camera_point(
        self,
        x_norm: float,
        y_norm: float,
        view: CameraView,
    ) -> Optional[np.ndarray]:
        x_ndc = (x_norm - 0.5) * 2.0
        y_ndc = (0.5 - y_norm) * 2.0
        tan_half_fov = math.tan(view.fov_y / 2.0)
        ray_dir = (
            view.forward
            + x_ndc * tan_half_fov * view.aspect * view.right
            + y_ndc * tan_half_fov * view.up
        )
        norm = np.linalg.norm(ray_dir)
        if norm < 1e-8:
            return None
        ray_dir /= norm
        denom = ray_dir[2]
        if abs(denom) < 1e-8:
            return None
        t = (TABLE_HEIGHT - view.position[2]) / denom
        if t <= 0:
            return None
        intersection = view.position + ray_dir * t
        return intersection.astype(float)

    def _hover_pose_for_tip(
        self,
        tip_position: np.ndarray,
        orientation_rpy: tuple[float, float, float],
    ) -> Pose:
        rotation = Pose.from_xyz_rpy((0.0, 0.0, 0.0), orientation_rpy).rotation
        wrist_position = tip_position - rotation @ _TIP_FORWARD_VECTOR
        return Pose(wrist_position, rotation)