"""Check the optional Explore launch exercise without starting nodes."""
from pathlib import Path
import runpy
import sys

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node

ROOT = Path(__file__).resolve().parents[1]


def main():
    path = ROOT / "exercises/explore_starter.launch.py"
    generate = runpy.run_path(str(path))["generate_launch_description"]
    description = generate()
    if not isinstance(description, LaunchDescription):
        raise TypeError("generate_launch_description() must return a LaunchDescription")

    arguments = {action.name for action in description.entities
                 if isinstance(action, DeclareLaunchArgument)}
    missing = {"robot_name", "use_sim_time"} - arguments
    if missing:
        print(f"TODO: declare these launch arguments: {', '.join(sorted(missing))}")
        return 1
    if not any(isinstance(action, Node) for action in description.entities):
        print("TODO: add a node action for explore_lite")
        return 1

    print("PASS: Explore launch has the required arguments and a node action")
    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except Exception as error:
        print(f"FAIL: {type(error).__name__}: {error}", file=sys.stderr)
        raise SystemExit(1)
