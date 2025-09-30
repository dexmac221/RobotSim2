# RobotSim-Py

Modern Python reimplementation of the 1999 RobotSim robotic arm demo. The new version keeps the 6-DOF arm kinematics, manual joint control, and target-seeking automation while adopting a clean, testable architecture.

## Features

- 6 revolute joint robotic arm using standard Denavit--Hartenberg parameters (PUMA560 inspired).
- Real-time visualization with `matplotlib` 3D animation.
- Manual mode: select joints and nudge them with the keyboard.
- Automatic mode: iterative Jacobian-based inverse kinematics drives the end effector toward the target.
- Adjustable joint velocity and solver aggressiveness.
- Target placement with keyboard shortcuts or live dragging inside the plot window.
- Modular Python package ready for integration with higher-level systems (e.g. Visual Language Action).

## Installation

```bash
python -m pip install --upgrade pip
python -m pip install .[dev]
```

Run the simulator:

```bash
python -m robotsim_py
```

## Controls

| Key | Action |
| --- | ------ |
| `left` / `right` | Cycle active joint (manual mode) |
| `up` / `down` | Increment / decrement selected joint |
| `m` | Toggle manual mode |
| `a` | Toggle automatic tracking |
| `r` | Reset joints to home |
| `t` | Pick a random reachable target |
| `[` / `]` | Decrease / increase joint step size |
| `-` / `=` | Decrease / increase IK gain |
| `q` | Quit |

Use `t` to quickly sample a new reachable target.

## Tests

Run the unit test suite:

```bash
python -m pytest
```

The tests validate Jacobian correctness against finite differences and basic reachability scenarios.

## Project layout

```
robotsim_py/
    robot.py        # Forward/Inverse kinematics and robot geometry
    controller.py   # Modes, manual controls, inverse-kinematics stepping
    rendering.py    # Matplotlib animation and input handling
    main.py         # Application entry point
```

## Next steps

- Swap `matplotlib` for a dedicated OpenGL renderer for higher fidelity.
- Persist user preferences to disk.
- Extend the controller with trajectory playback and scripting hooks.
