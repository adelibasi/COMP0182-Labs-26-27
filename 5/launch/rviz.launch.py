"""View one named robot's scans, map and localisation in RViz."""

import re
import tempfile
from pathlib import Path
from string import Template

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    OpaqueFunction,
    RegisterEventHandler,
    Shutdown,
)
from launch.event_handlers import OnShutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


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
    fixed_frame = LaunchConfiguration("fixed_frame").perform(context)
    if fixed_frame not in ("odom", "map"):
        raise ValueError("fixed_frame must be odom or map")

    fixed_frame = f"{namespace}/odom" if fixed_frame == "odom" else "map"
    # hardware descriptions already contain the namespace in their link names
    tf_prefix = namespace if LaunchConfiguration("use_sim_time").perform(context) == "true" else ""
    template = Path(__file__).resolve().parents[1] / "config/static/lab.rviz"
    rendered = Template(template.read_text()).substitute(
        agent_name=namespace, tf_prefix=tf_prefix, fixed_frame=fixed_frame
    )
    directory = tempfile.TemporaryDirectory(prefix=f"rviz_{namespace}_")
    config = Path(directory.name) / "lab.rviz"
    config.write_text(rendered)

    return [
        RegisterEventHandler(OnShutdown(on_shutdown=lambda event, context: directory.cleanup())),
        Node(
            package="rviz2",
            executable="rviz2",
            arguments=["-d", str(config)],
            parameters=[
                {
                    "use_sim_time": ParameterValue(
                        LaunchConfiguration("use_sim_time"), value_type=bool
                    )
                }
            ],
            output="screen",
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
                "fixed_frame",
                default_value="odom",
                choices=["odom", "map"],
                description="Start in the robot's odom frame, or map after a mapper/localiser starts",
            ),
            OpaqueFunction(function=_launch_nodes),
        ]
    )
