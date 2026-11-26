"""Nav2 controller and a single bridge to the Python A*."""

import json
import math
import re
from pathlib import Path

import yaml
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    OpaqueFunction,
    SetLaunchConfiguration,
)
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterFile, ParameterValue


def _nodes(context):
    name = LaunchConfiguration("robot_name").perform(context)
    if not re.fullmatch(r"tb_[A-Za-z0-9_]+", name):
        raise ValueError("robot_name must be tb_<NAME> (e.g. tb_ada)")

    config = Path(__file__).resolve().parents[1] / "config"
    tuning = yaml.safe_load((config / "nav_tuning.yaml").read_text())
    if not isinstance(tuning, dict) or any(
        type(value) not in (int, float) or not math.isfinite(value) or value <= 0
        for value in tuning.values()
    ):
        raise ValueError("config/nav_tuning.yaml must contain positive finite numbers")

    radius = tuning["body_radius_m"] + tuning["body_clearance_m"]
    footprint = json.dumps(
        [[radius, radius], [radius, -radius], [-radius, -radius], [-radius, radius]]
    )
    use_sim_time = ParameterValue(LaunchConfiguration("use_sim_time"), value_type=bool)
    params = [
        ParameterFile(str(config / "static/nav2.yaml"), allow_substs=True),
        {"use_sim_time": use_sim_time},
    ]
    servers = [
        ("nav2_controller", "controller_server"),
        ("nav2_planner", "planner_server"),
        ("nav2_behaviors", "behavior_server"),
        ("nav2_velocity_smoother", "velocity_smoother"),
        ("nav2_collision_monitor", "collision_monitor"),
        ("nav2_bt_navigator", "bt_navigator"),
        ("nav2_waypoint_follower", "waypoint_follower"),
    ]
    actions = [
        SetLaunchConfiguration("agent_name", name),
        SetLaunchConfiguration("lab_config_dir", str(config / "static")),
        SetLaunchConfiguration("collision_footprint", footprint),
    ]
    actions.extend(SetLaunchConfiguration(key, str(value)) for key, value in tuning.items())
    actions.append(
        Node(
            package="planner",
            executable="planner_service.py",
            output="screen",
            parameters=[{
                "use_sim_time": use_sim_time,
                "exercise_file": ParameterValue(str(config.parent / "exercises/planner.py"), value_type=str),
            }],
        )
    )
    for package, executable in servers:
        actions.append(
            Node(
                package=package,
                executable=executable,
                name=executable,
                output="screen",
                parameters=params,
                remappings=(
                    [("cmd_vel", "cmd_vel_nav")]
                    if executable in ("controller_server", "behavior_server", "velocity_smoother")
                    else []
                ),
            )
        )

    actions.append(
        Node(
            package="nav2_lifecycle_manager",
            executable="lifecycle_manager",
            name="lifecycle_manager_navigation",
            output="screen",
            parameters=[
                {
                    "use_sim_time": ParameterValue(
                        LaunchConfiguration("use_sim_time"), value_type=bool
                    ),
                    "autostart": True,
                    "node_names": [executable for _, executable in servers],
                }
            ],
        )
    )

    return actions


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "robot_name", description="Full namespace tb_<NAME> (e.g. tb_ada)"
            ),
            DeclareLaunchArgument(
                "use_sim_time", default_value="true", choices=["true", "false"]
            ),
            OpaqueFunction(function=_nodes),
        ]
    )
