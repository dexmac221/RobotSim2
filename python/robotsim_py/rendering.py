from __future__ import annotations

import time
import matplotlib.pyplot as plt

from .controller import RobotController
from .robot import RobotArm


class MatplotlibRenderer:
    """Real-time matplotlib visualiser for the robotic arm."""

    def __init__(
        self,
        controller: RobotController,
        robot: RobotArm,
        fps: int = 30,
    ) -> None:
        self.controller = controller
        self.robot = robot
        self.fps = fps
        self.dt = 1.0 / fps

        self.figure = plt.figure(figsize=(7, 7))
        self.axes = self.figure.add_subplot(111, projection="3d")
        self.axes.set_box_aspect([1, 1, 1])

        reach = robot.max_reach * 1.1
        self.axes.set_xlim(-reach, reach)
        self.axes.set_ylim(-reach, reach)
        self.axes.set_zlim(0.0, reach)
        self.axes.set_xlabel("X [m]")
        self.axes.set_ylabel("Y [m]")
        self.axes.set_zlabel("Z [m]")

        (self.arm_line,) = self.axes.plot([], [], [], "-o", color="C0", lw=2)
        self.target_artist = self.axes.scatter([], [], [], color="C3", s=50, depthshade=True)
        self.text_artist = self.axes.text2D(0.02, 0.95, "", transform=self.axes.transAxes)

        self.figure.canvas.mpl_connect("key_press_event", self.on_key_press)

        self.timer = self.figure.canvas.new_timer(interval=int(1000 / fps))
        self.timer.add_callback(self._tick)
        self.last_time = time.perf_counter()
        self.timer.start()
        self._tick()  # prime frame

    # ------------------------------------------------------------------
    def _tick(self) -> None:
        now = time.perf_counter()
        elapsed = now - self.last_time
        self.last_time = now
        self.controller.update(elapsed)
        self._update_plot()
        self.timer.start()

    def _update_plot(self) -> None:
        joints = self.robot.joint_positions(self.controller.joint_angles)
        self.arm_line.set_data(joints[:, 0], joints[:, 1])
        self.arm_line.set_3d_properties(joints[:, 2])

        target = self.controller.target
        self.target_artist._offsets3d = ([target[0]], [target[1]], [target[2]])

        status_lines = [f"{k}: {v}" for k, v in self.controller.status_text().items()]
        self.text_artist.set_text("\n".join(status_lines))
        self.figure.canvas.draw_idle()

    # ------------------------------------------------------------------
    def on_key_press(self, event) -> None:
        key = event.key
        if key in {"left", "right"}:
            direction = -1 if key == "left" else 1
            self.controller.cycle_joint(direction)
        elif key in {"up", "down"}:
            direction = 1 if key == "up" else -1
            self.controller.nudge_selected_joint(direction)
        elif key == "m":
            self.controller.set_mode("manual")
        elif key == "a":
            self.controller.set_mode("auto")
        elif key == "r":
            self.controller.reset()
        elif key == "t":
            self.controller.randomize_target()
        elif key in {"[", "{"}:
            self.controller.adjust_joint_step(0.8)
        elif key in {"]",
            "}",
        }:
            self.controller.adjust_joint_step(1.25)
        elif key in {"-", "_"}:
            self.controller.adjust_ik_gain(0.85)
        elif key in {"=", "+"}:
            self.controller.adjust_ik_gain(1.15)
        elif key == "q":
            plt.close(self.figure)
        elif key == "enter":
            self.controller.toggle_mode()

    # ------------------------------------------------------------------
    def show(self) -> None:
        plt.show(block=True)
