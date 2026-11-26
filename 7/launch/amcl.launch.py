"""Localise one named robot against a saved occupancy grid."""

import json
import math
import re
import sys
from pathlib import Path

import yaml
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    OpaqueFunction,
    Shutdown,
)
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "scripts"))
from sim_assets import arena_pose_to_world, load_task


def _stop_session(event, context):
    if context.is_shutdown:
        return []

    if event.returncode != 0:
        raise RuntimeError(
            f"{event.process_name} exited with status {event.returncode}"
        )

    return [Shutdown(reason=f"{event.process_name} closed")]


def _tuning_parameters(path):
    settings = yaml.safe_load(path.read_text())
    fields = {
        "amcl_min_particles": ("min_particles", int, 1),
        "amcl_max_particles": ("max_particles", int, 1),
        "amcl_update_distance_m": ("update_min_d", float, 0.0),
        "amcl_update_angle_rad": ("update_min_a", float, 0.0),
        "amcl_scan_beams": ("max_beams", int, 2),
    }
    if not isinstance(settings, dict) or set(settings) != set(fields):
        raise ValueError(f"{path.name} must contain exactly: {', '.join(fields)}")

    result = {}
    for key, (parameter, kind, minimum) in fields.items():
        value = settings[key]
        valid_type = type(value) is int if kind is int else type(value) in (int, float)
        if not valid_type or not math.isfinite(value) or value < minimum:
            raise ValueError(
                f"{path.name}: {key} must be a finite {kind.__name__} >= {minimum}"
            )

        result[parameter] = kind(value)

    if result["min_particles"] > result["max_particles"]:
        raise ValueError(
            f"{path.name}: minimum particles cannot exceed maximum particles"
        )

    return result


def _launch_nodes(context):
    name = LaunchConfiguration("robot_name").perform(context)
    if not re.fullmatch(r"tb_[A-Za-z0-9_]+", name):
        raise ValueError("robot_name must be tb_<NAME> (e.g. tb_ada)")

    namespace = name

    filename = LaunchConfiguration("map_file").perform(context)
    if not filename:
        raise ValueError("map_file must name a saved occupancy-grid YAML file")

    path = Path(filename).expanduser().resolve()
    if not path.is_file():
        raise FileNotFoundError(f"Saved map does not exist: {path}")

    config = Path(__file__).resolve().parents[1] / "config"
    tuning = _tuning_parameters(config / "amcl_tuning.yaml")

    # in Lab 7, SIM starts at known A; REAL retains manual 2D Pose Estimate
    seed_start = LaunchConfiguration("seed_start").perform(context) == "true"
    if seed_start and LaunchConfiguration("use_sim_time").perform(context) != "true":
        raise ValueError("Never seed a physical robot using SIM practice A")

    initial_pose = {}
    if seed_start:
        task = load_task(ROOT / "config/static/sim_task.yaml")
        geometry = json.loads((ROOT / "config/static/arena.json").read_text())
        x, y, yaw = arena_pose_to_world(geometry, task["start"]["pose_arena"])
        initial_pose = {"set_initial_pose": True, "always_reset_initial_pose": True,
                        "initial_pose.x": x, "initial_pose.y": y,
                        "initial_pose.z": 0.0, "initial_pose.yaw": yaw}

    clock = {
        "use_sim_time": ParameterValue(
            LaunchConfiguration("use_sim_time"), value_type=bool
        )
    }

    return [
        Node(
            package="nav2_map_server",
            executable="map_server",
            name="map_server",
            parameters=[clock, {"yaml_filename": str(path), "frame_id": "map"}],
            output="screen",
            on_exit=_stop_session,
        ),
        Node(
            package="nav2_amcl",
            executable="amcl",
            name="amcl",
            namespace=namespace,
            parameters=[
                str(config / "static/amcl.yaml"),
                clock,
                {
                    "base_frame_id": f"{namespace}/base_footprint",
                    "odom_frame_id": f"{namespace}/odom",
                    "scan_topic": f"/{namespace}/scan",
                    "map_topic": "/map",
                },
                tuning,
                initial_pose,
            ],
            output="screen",
            on_exit=_stop_session,
        ),
        Node(
            package="nav2_lifecycle_manager",
            executable="lifecycle_manager",
            name="lifecycle_manager_localisation",
            parameters=[
                clock,
                {
                    "autostart": True,
                    # map server is global; amcl is namespaced, so monitor exits
                    "bond_timeout": 0.0,
                    "node_names": ["map_server", f"{namespace}/amcl"],
                    "attempt_respawn_reconnection": False,
                },
            ],
            output="screen",
            on_exit=_stop_session,
        ),
        # in REAL, wait for RViz initial pose; SIM may explicitly seed practice A
    ]


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "robot_name", description="Full namespace tb_<NAME> (e.g. tb_ada)"
            ),
            DeclareLaunchArgument(
                "map_file", description="Path to your saved map YAML"
            ),
            DeclareLaunchArgument(
                "use_sim_time",
                default_value="true",
                choices=["true", "false"],
                description="Use Gazebo time; set false for a physical robot",
            ),
            DeclareLaunchArgument(
                "seed_start", default_value="false", choices=["true", "false"],
                description="SIM only: initialise AMCL at the known practice A pose",
            ),
            OpaqueFunction(function=_launch_nodes),
        ]
    )
