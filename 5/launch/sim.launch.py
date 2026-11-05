"""Run one named Burger in the Gazebo Classic TurtleBot3 demo world."""

import re
import sys
import tempfile
import xml.etree.ElementTree as ET
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    OpaqueFunction,
    RegisterEventHandler,
    Shutdown,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnShutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def _launch_nodes(context):
    name = LaunchConfiguration("robot_name").perform(context)
    if not re.fullmatch(r"[A-Za-z0-9_]+", name):
        raise ValueError(
            "robot_name must contain only letters, digits or underscores; "
            "for example: robot_name:=ada"
        )

    # The established namespace for all relevant topics; robot_name/odom, robot_name/scan
    namespace = f"tb_{name}"

    # configure ros topics against namespace
    gazebo_ros = Path(get_package_share_directory("gazebo_ros"))
    burger = Path(get_package_share_directory("turtlebot3_gazebo"))
    urdf = (burger / "urdf/turtlebot3_burger.urdf").read_text()
    model_dir = tempfile.TemporaryDirectory(prefix=f"{namespace}_")
    tree = ET.parse(burger / "models/turtlebot3_burger/model.sdf")

    # Prefix the existing frame, rather than replacing every sensor frame by
    # base_scan (which would incorrectly relabel an IMU as the laser frame).
    for tag in ("odometry_frame", "robot_base_frame", "frame_name"):
        for element in tree.getroot().iter(tag):
            frame = (element.text or "").strip().lstrip("/")
            if frame and frame != "world":
                element.text = f"{namespace}/{frame}"

    # use encoder odometry
    drive = tree.find(".//plugin[@filename='libgazebo_ros_diff_drive.so']")
    if drive is None:
        raise ValueError("TurtleBot3 differential-drive plugin not found")

    odom_source = drive.find("odometry_source")
    if odom_source is None:
        odom_source = ET.SubElement(drive, "odometry_source")

    odom_source.text = "0"

    # publish this robot's world pose directly
    model = tree.getroot().find("model")
    if model is None:
        raise ValueError("TurtleBot3 model not found")

    p3d = ET.SubElement(
        model, "plugin", name="mocap_ground_truth", filename="libgazebo_ros_p3d.so"
    )
    ros = ET.SubElement(p3d, "ros")
    ET.SubElement(ros, "remapping").text = "odom:=mocap/odom"
    ET.SubElement(p3d, "body_name").text = "base_footprint"
    ET.SubElement(p3d, "frame_name").text = "world"
    ET.SubElement(p3d, "update_rate").text = "30"
    ET.SubElement(p3d, "gaussian_noise").text = "0.0"

    model_path = Path(model_dir.name) / "model.sdf"
    tree.write(model_path, encoding="utf-8", xml_declaration=False)

    return [
        ExecuteProcess(
            cmd=[
                sys.executable,
                str(Path(__file__).resolve().parents[1] / "scripts/mocap_pose.py"),
                namespace,
                "--ros-args",
                "-p",
                "use_sim_time:=true",
            ],
            output="screen",
            on_exit=Shutdown(reason="mocap publisher exited"),
        ),
        RegisterEventHandler(OnShutdown(on_shutdown=lambda event, context: model_dir.cleanup())),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(gazebo_ros / "launch/gzserver.launch.py")),
            launch_arguments={"world": str(burger / "worlds/turtlebot3_world.world")}.items(),
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
                "-entity",
                namespace,
                "-file",
                str(model_path),
                "-robot_namespace",
                namespace,
                "-x",
                "-2.0",
                "-y",
                "-0.5",
                "-z",
                "0.01",
                "-Y",
                "0.0",
                "-timeout",
                "60",
            ],
        ),
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            namespace=namespace,
            output="screen",
            parameters=[
                {
                    "use_sim_time": True,
                    "robot_description": ParameterValue(urdf, value_type=str),
                    "frame_prefix": f"{namespace}/",
                }
            ],
        ),
    ]


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "robot_name", description="Your robot's name, e.g. robot_name:=ada"
            ),
            DeclareLaunchArgument(
                "gui",
                default_value="true",
                choices=["true", "false"],
                description="Show the Gazebo window",
            ),
            OpaqueFunction(function=_launch_nodes),
        ]
    )
