"""LiDAR RTAB-Map from Lab 6; works with either ROS 2 clock mode."""

import re
from pathlib import Path
from uuid import uuid4

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def _nodes(context):
    name = LaunchConfiguration("robot_name").perform(context)
    if not re.fullmatch(r"tb_[A-Za-z0-9_]+", name):
        raise ValueError("robot_name must be tb_<NAME> (e.g. tb_ada)")

    database = Path.home() / "lab7_maps" / f"{name}_{uuid4().hex}.db"
    database.parent.mkdir(parents=True, exist_ok=True)

    return [
        LogInfo(msg=f"RTAB-Map database: {database}"),
        Node(
            package="rtabmap_slam",
            executable="rtabmap",
            name="rtabmap",
            output="screen",
            parameters=[
                str(Path(__file__).resolve().parents[1] / "config/static/rtabmap.yaml"),
                {
                    "frame_id": f"{name}/base_footprint",
                    "odom_frame_id": f"{name}/odom",
                    "use_sim_time": ParameterValue(LaunchConfiguration("use_sim_time"), value_type=bool),
                    "database_path": str(database),
                },
            ],
            remappings=[("scan", f"/{name}/scan"), ("map", "/map")],
        ),
    ]


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "robot_name", description="Full namespace tb_<NAME> (e.g. tb_ada)"
            ),
            DeclareLaunchArgument(
                "use_sim_time", default_value="true", choices=["true", "false"],
                description="SIM true or approved real robot false",
            ),
            OpaqueFunction(function=_nodes),
        ]
    )
