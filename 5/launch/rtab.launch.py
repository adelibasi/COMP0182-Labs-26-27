"""Map one named robot's LiDAR scans using its wheel-odometry TF."""

import math
import re
from pathlib import Path
from uuid import uuid4

import yaml
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction, Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def _tuning_parameters(path):
    """Validate the four student settings, then pass RTAB's required strings."""
    settings = yaml.safe_load(path.read_text())
    bounds = {
        "Rtabmap/DetectionRate": (0.0, 30.0),
        "RGBD/LinearUpdate": (0.0, 2.0),
        "RGBD/AngularUpdate": (0.0, math.pi),
        "Grid/CellSize": (0.01, 0.5),
    }
    if not isinstance(settings, dict) or set(settings) != set(bounds):
        raise ValueError(f"{path.name} must contain exactly: {', '.join(bounds)}")

    result = {}
    for key, (minimum, maximum) in bounds.items():
        value = settings[key]
        if isinstance(value, bool):
            raise ValueError(f"{path.name}: {key} must be a number, not true/false")

        try:
            number = float(value)
        except (TypeError, ValueError) as error:
            raise ValueError(f"{path.name}: {key} must be a number") from error

        if not math.isfinite(number) or not minimum <= number <= maximum:
            raise ValueError(f"{path.name}: {key} must be between {minimum:g} and {maximum:g}")

        result[key] = str(number)
    return result


def _stop_session(event, context):
    if context.is_shutdown:
        return []

    if event.returncode != 0:
        raise RuntimeError(f"{event.process_name} exited with status {event.returncode}")

    return [Shutdown(reason=f"{event.process_name} closed")]


def _launch_nodes(context):
    name = LaunchConfiguration("robot_name").perform(context)
    if not re.fullmatch(r"[A-Za-z0-9_]+", name):
        raise ValueError(
            "robot_name must contain only letters, digits or underscores; "
            "for example: robot_name:=ada"
        )
    namespace = f"tb_{name}"
    use_mocap = LaunchConfiguration("use_mocap").perform(context) == "true"
    proximity = LaunchConfiguration("spatial_proximity").perform(context)
    if proximity not in ("true", "false"):
        raise ValueError("spatial_proximity must be true or false")

    config = Path(__file__).resolve().parents[1] / "config"
    tuning = _tuning_parameters(config / "rtabmap_tuning.yaml")

    data = Path.home() / "lab5_maps"
    data.mkdir(parents=True, exist_ok=True)
    database = data / f"rtabmap_{namespace}_{uuid4().hex}.db"

    return [
        LogInfo(msg=f"RTAB-Map database: {database}"),
        Node(
            package="rtabmap_slam",
            executable="rtabmap",
            name="rtabmap",
            output="screen",
            parameters=[
                str(config / "static/rtabmap.yaml"),
                {key: ParameterValue(value, value_type=str) for key, value in tuning.items()},
                {
                    "frame_id": f"{namespace}/base_footprint",
                    "odom_frame_id": f"{namespace}/odom",
                    "use_sim_time": ParameterValue(
                        LaunchConfiguration("use_sim_time"), value_type=bool
                    ),
                    "database_path": str(database),
                    "RGBD/ProximityBySpace": ParameterValue(proximity, value_type=str),
                    # Make the baseline independent of simulator ground truth.
                    "Optimizer/PriorsIgnored": ParameterValue(
                        "false" if use_mocap else "true", value_type=str
                    ),
                },
                {"Optimizer/Strategy": ParameterValue("1", value_type=str)} if use_mocap else {},
            ],
            remappings=[
                ("scan", f"/{namespace}/scan"),
                ("map", "/map"),
                # Disconnection as well as PriorsIgnored avoids even recording
                # privileged poses in the baseline database.
                (
                    "global_pose",
                    (
                        f"/{namespace}/mocap/pose"
                        if use_mocap
                        else f"/{namespace}/lab5_unused_global_pose"
                    ),
                ),
                ("imu", f"/{namespace}/lab5_unused_imu"),
            ],
            on_exit=_stop_session,
        ),
    ]


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "robot_name", description="Your robot's name, e.g. robot_name:=ada"
            ),
            DeclareLaunchArgument(
                "use_sim_time",
                default_value="true",
                choices=["true", "false"],
                description="Use Gazebo time; set false for a physical robot",
            ),
            DeclareLaunchArgument(
                "use_mocap",
                default_value="false",
                choices=["true", "false"],
                description="Optional privileged-pose experiment; requires g2o",
            ),
            DeclareLaunchArgument(
                "spatial_proximity",
                default_value="true",
                choices=["true", "false"],
                description="Enable spatial proximity matching; false for the A/B exercise",
            ),
            OpaqueFunction(function=_launch_nodes),
        ]
    )
