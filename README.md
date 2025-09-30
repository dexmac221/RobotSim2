# RobotSim2 OpenGL Simulator with IK and Gemini Robotics Support

![RobotSim2](RobotSim2.png)

Modern Python recreation of my 1999 **RobotSim** desktop demonstrator, originally developed during my M.Sc. studies. This edition keeps the 6-DOF manipulator behaviour while adding clean Python APIs, a refreshed OpenGL renderer, and optional Gemini Robotics perception for automatic target acquisition.

---

## ✨ Highlights

- **Analytic kinematics** – Forward / inverse kinematics with damped least squares, Jacobians, and reusable `RobotModel` + `RobotState` abstractions.
- **Interactive viewer** – OpenGL app with orbit camera, inset hand camera, overhead capture pipeline, and animated gripper that closes around the target.
- **Trajectory tooling** – Manual joint jogs, reusable trajectory controllers, and spline helpers for composing motions programmatically.
- **Gemini Robotics integration** – Single-shot or streaming calls to Gemini Robotics models, with workspace-aware target clamping, orientation search, and rich debug logs (pixel coordinates, camera projection, world pose).
- **Automation ready** – Structured JSON logs, head-camera frame dumps, and a fully tested Python package that plays nicely with VLA agents or other orchestration frameworks.

---

## 📁 Repository layout

```
robot_sim_py/
  __init__.py
  app.py             # CLI entry point for scripted demos
  ogl_app.py         # Interactive OpenGL viewer & Gemini orchestration
  gemini_agent.py    # Gemini Robotics helpers & coordinate transforms
  robot.py           # Robot kinematics, Pose helpers, Jacobians
  controllers.py     # Manual & trajectory controllers
  trajectory.py      # Trajectory primitives and interpolation helpers
  scene.py           # Shared scene constants, workspace limits
  reachability.py    # Orientation search utilities
tests/
  ...                # Pytest suite covering kinematics & Gemini logic
```

Generated assets and logs are written under `data/` and `outputs/`. The legacy C++ Visual Studio project has been removed so the repository is now Python-only.

---

## 🚀 Quick start

1. **Create / activate an environment** (Python 3.10+ recommended):

  ```bash
  conda create -n robotsim python=3.10 -y
  conda activate robotsim
  ```

2. **Install RobotSim** in editable mode with dev extras:

  ```bash
  pip install -e .[dev]
  ```

3. **Run the demo CLI** (headless trajectory playback):

  ```bash
  robot-sim-demo --duration 6 --dt 0.05
  ```

4. **Launch the OpenGL viewer** (requires an OpenGL-capable display):

  ```bash
  robot-sim-ogl --auto
  ```

Common options:

| Option | Purpose |
| --- | --- |
| `--joint-count 4` | Simulate a 4-DOF version of the arm (supports 1–6 joints) |
| `--no-hand-view` | Hide the inset hand camera |
| `--capture-dir data/hand_frames` | Save hand-camera PNG frames |
| `--top-capture-dir data/head_frames` | Save head-camera frames for Gemini |
| `--top-capture-size 640 640` | Override capture resolution |
| `--log robot_log.json` | Dump end-effector telemetry |
| `--free-wrist` / `--free-joints` | Allow relaxed wrist/yaw targeting |

Use `robot-sim-ogl --help` for the full list.

---

## 🎮 Viewer controls

| Keys | Action |
|------|--------|
| `1`–`6` | Select joint to manipulate |
| `J` / `K` | Decrease / increase selected joint angle |
| `Space` | Toggle automatic trajectory playback |
| `R` | Reset joints, rebuild trajectory, trigger Gemini single-shot |
| `NumPad 8 / 2` | Nudge target +Y / -Y |
| `NumPad 4 / 6` | Nudge target -X / +X |
| `NumPad 7 / 1` | Raise / lower target |
| `NumPad 5` | Randomize target position within workspace |
| `B` | Print the target's world coordinates |
| `Arrow keys` | Orbit camera |
| `Page Up` / `Page Down` | Zoom |
| `=` / `+` / `-` | Adjust manual jog speed |
| `V` | Toggle hand camera |
| `G` | Start/stop Gemini control loop |
| `A` | Queue another Gemini capture with the current target |
| `H` | Reprint control help |
| `Esc` | Quit |

Hand-camera and overhead capture directories store PNG sequences (`hand_00000.png`, `head_00000.png`, …) whenever the corresponding options are enabled.

---

## 🤖 Gemini Robotics integration

1. Export your API key: `export GEMINI_API_KEY=...` (alternatively `GENAI_API_KEY` or `GOOGLE_API_KEY`).
2. Launch the viewer with Gemini enabled:

  ```bash
  robot-sim-ogl --gemini --free-joints
  ```

3. Reset the robot (`R`) to trigger a single-shot perception/control cycle.

Key flags:

| Flag | Description |
| --- | --- |
| `--gemini-model` | Gemini Robotics model ID (default `gemini-robotics-er-1.5-preview`) |
| `--gemini-mode` | `single` (default) or `stream` for continuous updates |
| `--joint-count` | Number of arm joints to simulate (1–6). Affects kinematics, rendering, and controllers |
| `--gemini-joint-count` | Override the joint count mentioned in the Gemini prompt (defaults to the simulated arm) |
| `--gemini-interval` | Minimum seconds between requests |
| `--gemini-hover-height` | Hover distance above the table before descent |
| `--free-wrist` / `--free-joints` | Enables orientation search so the arm can reach edge targets |

Every Gemini request now prints a concise summary such as:

```
Gemini target px=(498, 500) → world=(+0.238, -0.399, +0.160) rpy=(180.0°, +0.0°, -128.7°) via camera (clamped)
```

If inverse kinematics or orientation search fails, the status line explains which stage was rejected along with the pixel-to-world mapping that prompted it.

### Debug artifacts

Provide `--gemini-debug-dir logs/gemini` to persist:

- Captured head-camera frame (`.png`),
- Parsed Gemini detections,
- World-frame fingertip targets and chosen wrist orientations,
- Full hover pose transform matrices,
- Latest camera intrinsics/extrinsics snapshot.

This is ideal for auditing Gemini results or building datasets for offline evaluation.

### Offline image testing

You can reproduce a Gemini call with any saved frame:

```bash
robot-sim-gemini-image-test data/head_frames/head_00012.png \
  --dump-json outputs/gemini_result.json \
  --joint-count 4 \
  --gemini-api-key "$GEMINI_API_KEY"
```

The helper prints the raw model response and the mapped tabletop coordinates used by the controller.

---

## 🧪 Running the tests

```bash
pytest
```

The suite covers kinematics, reachability bounds, and Gemini mapping edge cases. All 15 tests pass after the latest workspace and logging improvements.

---

## 🤝 Embedding & automation

Core primitives live in `robot_sim_py` and are designed for direct use inside VLA agents or other planners:

```python
from robot_sim_py.robot import RobotModel, RobotState
from robot_sim_py.controllers import ManualController

robot = RobotModel.default()
controller = ManualController(robot, RobotState.zeros())
state = controller.step([0, 0.1, 0, 0, 0, 0], dt=0.1)
pose = robot.forward_kinematics(state.joint_angles)
print(pose.position)
```

Trajectory builders, controllers, and scene constants make it straightforward to translate natural-language commands into joint-space motions.

---

## 📌 Notes

- The original Visual Studio C++ sources and binaries have been removed to keep the repository focused on the Python reimplementation.
- For headless environments (e.g., CI), you can exercise kinematics and Gemini utilities without launching OpenGL.
- Contributions are welcome—new trajectory primitives, physics integration, or WebGL front-ends are all great future extensions.

---

## 📝 License

This project is licensed under the [Apache License 2.0](LICENSE). The same license is used by Google’s Gemini Robotics samples, ensuring compatibility with downstream integrations and contributions.
