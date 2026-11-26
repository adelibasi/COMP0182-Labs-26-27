"""Supplied ROS handles and recorded poses for the Lab 7 mission."""

import math
from pathlib import Path
import re

from ament_index_python.packages import get_package_share_directory
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.node import Node
from tf2_ros import Buffer, TransformListener

from mission.marker_reader import MarkerReader
from sim_assets import finite_vector, load_task

MapPose = tuple[float, float, float]
VISIT_ORDER = ("room_1", "room_2", "room_3")


def mission_goals(config_dir: Path, *, sim: bool):
    """Load a mode's recorded map poses and camera settings."""
    import yaml  # rusty-linter: ignore

    path = config_dir / ("sim_mission.yaml" if sim else "real_mission.yaml")
    try:
        task = yaml.safe_load(path.read_text(encoding="utf-8"))
    except yaml.YAMLError as error:
        raise ValueError(f"Invalid YAML in {path}: {error}") from error
    if not isinstance(task, dict):
        raise ValueError(f"{path} must contain a YAML mapping")

    finite_vector(task.get("start_pose"), 3, "start_pose (A)")
    rooms = task.get("rooms")
    if not isinstance(rooms, dict) or rooms.keys() != set(VISIT_ORDER):
        raise ValueError("rooms must contain room_1, room_2 and room_3")

    view_poses, goal_poses = {}, {}
    for room in VISIT_ORDER:
        poses = rooms[room]
        if not isinstance(poses, dict):
            raise ValueError(f"{room} must contain view_pose and goal_pose")
        view = tuple(finite_vector(poses.get("view_pose"), 3, f"{room}.view_pose"))
        goal = tuple(finite_vector(poses.get("goal_pose"), 3, f"{room}.goal_pose"))
        if math.hypot(view[0] - goal[0], view[1] - goal[1]) < 0.3:
            raise ValueError(f"{room}.goal_pose must be at least 0.3 m from its view_pose")
        view_poses[room], goal_poses[room] = view, goal

    camera_settings = task
    if sim:
        installed_config = Path(get_package_share_directory("mission")) / "config"
        camera_settings = load_task(installed_config / "sim_task.yaml")
    labels = camera_settings.get("marker_id_to_fruit")
    target_fruit = task.get("target_fruit")
    if (
        not isinstance(target_fruit, str)
        or not isinstance(labels, dict)
        or target_fruit not in labels.values()
    ):
        raise ValueError(f"Set target_fruit in {path} to a configured fruit label")

    return target_fruit, view_poses, goal_poses, camera_settings.get("observation")


class MissionNode(Node):
    goal_timeout_s = 180.0
    arrival_timeout_s = 4.0
    position_tolerance_m = 0.25
    heading_tolerance_rad = 0.35

    def __init__(self):
        super().__init__("lab7_task2")
        robot_name = self.declare_parameter("robot_name", "").value
        if not re.fullmatch(r"tb_[A-Za-z0-9_]+", robot_name):
            raise ValueError("Set robot_name to the full tb_<NAME> namespace")

        sim = self.get_parameter("use_sim_time").value
        self.target_fruit, self.view_poses, self.goal_poses, observation = mission_goals(
            Path("config"), sim=sim
        )
        self.visit_order = list(VISIT_ORDER)

        self.action = ActionClient(self, NavigateToPose, "/navigate_to_pose")
        self.base_frame = f"{robot_name}/base_footprint"
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.marker_reader = MarkerReader(self, robot_name, observation)
