#!/usr/bin/env python3
"""Run the Lab 7 SIM mission from the lab directory."""

from pathlib import Path
from runpy import run_path
import sys

import rclpy

from mission.node import MissionNode

EXERCISE_FILE = Path("exercises/autonomous_search.py")


def load_exercise(exercise_file: Path = EXERCISE_FILE) -> type[MissionNode]:
    exercise_file = exercise_file.resolve()
    if not exercise_file.is_file():
        raise FileNotFoundError(
            f"Missing {exercise_file}. Run from the Lab 7 directory"
        )

    student = run_path(str(exercise_file)).get("AutonomousSearch")
    if not isinstance(student, type) or not issubclass(student, MissionNode):
        raise ValueError(
            f"{exercise_file} must define AutonomousSearch as a MissionNode subclass"
        )

    return student


def main(node_type=None):
    rclpy.init()
    node = None
    try:
        if node_type is None:
            node_type = load_exercise()

        node = node_type()
        node.marker_reader.wait_for_camera()
        chosen = node.run_mission()
        if chosen not in node.goal_poses:
            raise RuntimeError("Expected the selected room key")

        room_name = chosen.replace("_", " ").capitalize()
        node.get_logger().info(
            f"PASS: parked in {room_name} for {node.target_fruit}"
        )

        return 0
    except (OSError, RuntimeError, ValueError) as error:
        print(f"FAIL: autonomous mission: {error}", file=sys.stderr)

        return 1
    finally:
        if node is not None:
            node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
