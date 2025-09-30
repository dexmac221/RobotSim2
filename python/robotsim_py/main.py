from __future__ import annotations

from .controller import RobotController
from .rendering import MatplotlibRenderer
from .robot import RobotArm


def run() -> None:
    robot = RobotArm.default_puma()
    controller = RobotController(robot)
    renderer = MatplotlibRenderer(controller, robot)
    renderer.show()


def main() -> None:  # pragma: no cover - console entry point
    run()


if __name__ == "__main__":
    run()
