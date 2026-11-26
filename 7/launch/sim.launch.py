"""Lab 7 practice arena, temporary RGB Burger and visible ArUco boards."""

import importlib.util
import json
import os
import re
import tempfile
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
    RegisterEventHandler,
    SetEnvironmentVariable,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnShutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def _launch_nodes(context):
    name = LaunchConfiguration("robot_name").perform(context)
    if not re.fullmatch(r"tb_[A-Za-z0-9_]+", name):
        raise ValueError("robot_name must be tb_<NAME> (e.g. tb_ada)")

    lab_dir = Path(__file__).resolve().parents[1]
    world_path = lab_dir / "worlds/arena.world"
    if not world_path.is_file():
        raise FileNotFoundError("Missing arena world; run make arena")

    spec = importlib.util.spec_from_file_location("lab7_sim_assets", lab_dir / "scripts/sim_assets.py")
    assets = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(assets)
    geometry = json.loads((lab_dir / "config/static/arena.json").read_text())
    task = assets.load_task(Path(LaunchConfiguration("task_config").perform(context)))
    x, y, yaw = assets.arena_pose_to_world(geometry, task["start"]["pose_arena"])

    gazebo_ros = Path(get_package_share_directory("gazebo_ros"))
    burger = Path(get_package_share_directory("turtlebot3_gazebo"))
    camera_transport = LaunchConfiguration("camera_transport").perform(context)
    if camera_transport == "h264":
        get_package_share_directory("ffmpeg_image_transport")

    sdf, urdf = assets.prepare_robot(
        (burger / "models/turtlebot3_burger/model.sdf").read_text(),
        (burger / "urdf/turtlebot3_burger.urdf").read_text(), name,
        camera_transport=camera_transport,
    )
    world = assets.add_markers(world_path.read_text(), geometry, task, lab_dir / "models")
    model_dir = tempfile.TemporaryDirectory(prefix=f"{name}_")
    model_path = Path(model_dir.name) / "model.sdf"
    runtime_world = Path(model_dir.name) / "arena_with_markers.world"
    model_path.write_text(sdf, encoding="utf-8")
    runtime_world.write_text(world, encoding="utf-8")

    return [
        SetEnvironmentVariable(
            "GAZEBO_MODEL_PATH",
            ":".join(path for path in (str(lab_dir / "models"), str(burger / "models"),
                                      os.environ.get("GAZEBO_MODEL_PATH", "")) if path),
        ),
        RegisterEventHandler(
            OnShutdown(on_shutdown=lambda event, context: model_dir.cleanup())
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(gazebo_ros / "launch/gzserver.launch.py")),
            launch_arguments={"world": str(runtime_world)}.items(),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(gazebo_ros / "launch/gzclient.launch.py")),
            condition=IfCondition(LaunchConfiguration("gui")),
        ),
        Node(
            package="gazebo_ros",
            executable="spawn_entity.py",
            output="screen",
            arguments=[
                "-entity", name,
                "-file", str(model_path),
                "-robot_namespace", name,
                "-x", str(x),
                "-y", str(y),
                "-z", "0.01",
                "-Y", str(yaw),
                "-timeout", "180",
            ],
        ),
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            namespace=name,
            output="screen",
            parameters=[{
                "use_sim_time": True,
                "robot_description": ParameterValue(urdf, value_type=str),
                "frame_prefix": f"{name}/",
            }],
        ),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            "robot_name", description="Full namespace tb_<NAME> (e.g. tb_ada)"
        ),
        DeclareLaunchArgument(
            "gui", default_value="true", choices=["true", "false"],
            description="Show the Gazebo window",
        ),
        DeclareLaunchArgument(
            "camera_transport", default_value="h264", choices=["raw", "h264"],
            description="Gazebo camera output, use a separate decoder for H.264",
        ),
        DeclareLaunchArgument(
            "task_config", default_value=str(Path(__file__).resolve().parents[1] / "config/static/sim_task.yaml"),
            description="SIM practice marker placements and start pose",
        ),
        OpaqueFunction(function=_launch_nodes),
    ])
