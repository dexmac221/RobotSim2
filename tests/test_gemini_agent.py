import math
import types

import numpy as np
import pytest
import time
from pathlib import Path
from PIL import Image

from robot_sim_py.camera import CameraView
from robot_sim_py.gemini_agent import GeminiRobotAgent
from robot_sim_py.robot import RobotModel, RobotState
from robot_sim_py.scene import (
    TABLE_HEIGHT,
    TARGET_HEIGHT_OFFSET,
    WORKSPACE_X_BOUNDS,
    WORKSPACE_Y_BOUNDS,
)


class _StubClient:
    """Minimal stub that satisfies the google-genai client interface."""

    def __init__(self) -> None:
        class _Models:
            def generate_content(self, *args, **kwargs):  # pragma: no cover - unused safeguard
                raise AssertionError("Network call not expected during unit tests")

        self.models = _Models()


def test_normalized_to_world_center() -> None:
    robot = RobotModel.default()
    agent = GeminiRobotAgent(robot=robot, client=_StubClient(), min_request_interval=0.0)
    world_point = agent._normalized_to_world([500, 500])
    expected_x = WORKSPACE_X_BOUNDS[0] + 0.5 * (WORKSPACE_X_BOUNDS[1] - WORKSPACE_X_BOUNDS[0])
    expected_y = WORKSPACE_Y_BOUNDS[0] + 0.5 * (WORKSPACE_Y_BOUNDS[1] - WORKSPACE_Y_BOUNDS[0])
    assert world_point[0] == pytest.approx(expected_x, abs=1e-6)
    assert world_point[1] == pytest.approx(expected_y, abs=1e-6)
    assert world_point[2] == pytest.approx(TABLE_HEIGHT + agent.hover_height)


def test_compute_next_state_uses_gemini_detection() -> None:
    robot = RobotModel.default()
    robot.inverse_kinematics = types.MethodType(  # type: ignore[assignment]
        lambda self, target, initial=None, **_: RobotState.zeros(),
        robot,
    )

    class _FakeAgent(GeminiRobotAgent):
        def _request_target_point(self, image_bytes: bytes):  # type: ignore[override]
            return [{"point": [500, 719], "label": "target sphere"}], "[{\"point\": [500, 719], \"label\": \"target sphere\"}]"

    agent = _FakeAgent(
        robot=robot,
        client=_StubClient(),
        min_request_interval=0.0,
        target_label="target",
    )
    state = RobotState.zeros()
    frame = np.zeros((64, 64, 3), dtype=np.uint8)

    new_state = agent.compute_next_state(state, frame, frame_serial=0)
    assert isinstance(new_state, RobotState)
    assert agent._last_frame_serial == 0

    # Reusing the same frame serial should skip additional IK work.
    assert agent.compute_next_state(new_state or state, frame, frame_serial=0) is None


def test_free_orientation_enables_additional_targets() -> None:
    robot = RobotModel.default()
    target_tip = np.array([0.05, -0.45, TABLE_HEIGHT + TARGET_HEIGHT_OFFSET], dtype=float)

    class _TargetAgent(GeminiRobotAgent):
        def _request_target_point(self, image_bytes: bytes):  # type: ignore[override]
            return ([{"point": [100, 200], "label": "target"}], "raw")

        def _normalized_to_world(self, point, *, camera_view=None):  # type: ignore[override]
            return target_tip.copy()

    frame = np.zeros((32, 32, 3), dtype=np.uint8)
    state = RobotState.zeros()

    constrained_agent = _TargetAgent(
        robot=robot,
        client=_StubClient(),
        min_request_interval=0.0,
        allow_free_orientation=False,
    )
    assert constrained_agent.compute_next_state(state, frame, frame_serial=0) is None

    free_agent = _TargetAgent(
        robot=robot,
        client=_StubClient(),
        min_request_interval=0.0,
        allow_free_orientation=True,
    )
    new_state = free_agent.compute_next_state(state, frame, frame_serial=1)
    assert isinstance(new_state, RobotState)
    _, _, yaw = free_agent.last_orientation
    assert not math.isclose(yaw, 0.0, abs_tol=1e-3)


def test_quota_backoff_skips_additional_requests() -> None:
    robot = RobotModel.default()

    class _QuotaAgent(GeminiRobotAgent):
        def __init__(self, *args, **kwargs):  # type: ignore[no-untyped-def]
            super().__init__(*args, **kwargs)
            self.calls = 0
            self.fail_first = True

        def _request_target_point(self, image_bytes):  # type: ignore[override]
            self.calls += 1
            if self.fail_first:
                self.fail_first = False
                self._handle_request_exception(RuntimeError("HTTP 429: quota exceeded"))
                return [], ""
            return super()._request_target_point(image_bytes)

    agent = _QuotaAgent(
        robot=robot,
        client=_StubClient(),
        min_request_interval=0.0,
        quota_backoff=0.5,
    )

    frame = np.zeros((32, 32, 3), dtype=np.uint8)
    state = RobotState.zeros()

    assert agent.compute_next_state(state, frame, frame_serial=0) is None
    assert agent.calls == 1
    cooldown_remaining = agent._cooldown_until - time.perf_counter()
    assert cooldown_remaining > 0

    # Because of the cooldown, a second call should skip contacting Gemini entirely.
    assert agent.compute_next_state(state, frame, frame_serial=1) is None
    assert agent.calls == 1


def test_infer_from_frame_uses_request_stub() -> None:
    robot = RobotModel.default()

    class _StubAgent(GeminiRobotAgent):
        def __init__(self, *args, **kwargs):  # type: ignore[no-untyped-def]
            super().__init__(*args, **kwargs)
            self.request_called = False

        def _request_target_point(self, image_bytes):  # type: ignore[override]
            self.request_called = True
            assert isinstance(image_bytes, bytes)
            return ([{"point": [250, 400], "label": "target"}], "raw")

    agent = _StubAgent(robot=robot, client=_StubClient())
    frame = np.zeros((8, 8, 3), dtype=np.uint8)

    points, raw = agent.infer_from_frame(frame)
    assert agent.request_called
    assert points[0]["point"] == [250, 400]
    assert raw == "raw"


def test_infer_from_file_reads_image(tmp_path: Path) -> None:
    robot = RobotModel.default()

    class _FileAgent(GeminiRobotAgent):
        def _request_target_point(self, image_bytes):  # type: ignore[override]
            # Ensure the encoded bytes are non-empty
            assert isinstance(image_bytes, bytes)
            assert len(image_bytes) > 0
            return ([{"point": [600, 200], "label": "target"}], "text")

    image_path = tmp_path / "frame.png"
    Image.new("RGB", (16, 16), (255, 0, 0)).save(image_path)

    agent = _FileAgent(robot=robot, client=_StubClient())
    points, raw = agent.infer_from_file(image_path)

    assert points and points[0]["point"] == [600, 200]
    assert raw == "text"


def test_project_camera_center_point() -> None:
    robot = RobotModel.default()
    agent = GeminiRobotAgent(robot=robot, client=_StubClient())

    view = CameraView(
        position=np.array([0.0, 0.0, 0.75], dtype=float),
        forward=np.array([0.0, 0.0, -1.0], dtype=float),
        up=np.array([0.0, 1.0, 0.0], dtype=float),
        right=np.array([1.0, 0.0, 0.0], dtype=float),
        fov_y=math.radians(60.0),
        aspect=1.0,
        image_width=640,
        image_height=640,
    )

    world = agent._normalized_to_world([500, 500], camera_view=view)
    assert WORKSPACE_X_BOUNDS[0] <= world[0] <= WORKSPACE_X_BOUNDS[1]
    assert world[0] == pytest.approx(WORKSPACE_X_BOUNDS[0], abs=1e-6)
    assert world[1] == pytest.approx(0.0, abs=1e-6)
    assert world[2] == pytest.approx(TABLE_HEIGHT + agent.hover_height)


def test_project_camera_offset_point() -> None:
    robot = RobotModel.default()
    agent = GeminiRobotAgent(robot=robot, client=_StubClient())

    view = CameraView(
        position=np.array([0.0, 0.0, 0.80], dtype=float),
        forward=np.array([0.0, 0.0, -1.0], dtype=float),
        up=np.array([0.0, 1.0, 0.0], dtype=float),
        right=np.array([1.0, 0.0, 0.0], dtype=float),
        fov_y=math.radians(60.0),
        aspect=1.0,
        image_width=640,
        image_height=640,
    )

    world = agent._normalized_to_world([500, 900], camera_view=view)
    assert WORKSPACE_X_BOUNDS[0] <= world[0] <= WORKSPACE_X_BOUNDS[1]
    assert world[0] > WORKSPACE_X_BOUNDS[0]
    assert abs(world[1]) < 1e-6


def test_parse_response_text_handles_code_block() -> None:
    robot = RobotModel.default()
    agent = GeminiRobotAgent(robot=robot, client=_StubClient())

    response = "Here is the detection:\n```json\n[{\"point\": [120, 345], \"label\": \"target\"}]\n```"
    parsed = agent._parse_response_text(response)
    assert parsed is not None
    assert parsed[0]["point"] == [120, 345]
