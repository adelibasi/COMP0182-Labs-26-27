"""Run the Lab 7 H.264 decoder and ArUco reader on the same host."""

from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction, Shutdown
from launch.substitutions import LaunchConfiguration


def _stop_session(event, context):
    if context.is_shutdown:
        return []

    if event.returncode:
        raise RuntimeError(f"{event.process_name} exited with status {event.returncode}")

    return [Shutdown(reason=f"{event.process_name} closed")]


def _launch_nodes(context):
    package = Path(__file__).resolve().parent
    lab = package.parents[1]
    robot = LaunchConfiguration("robot_name")
    use_sim_time = LaunchConfiguration("use_sim_time").perform(context)
    sim = use_sim_time == "true"
    input_topic = LaunchConfiguration("input_topic").perform(context)
    if not input_topic:
        input_topic = "camera/video_h264" if sim else "/camera/video_h264"

    input_localhost_only = LaunchConfiguration("input_localhost_only").perform(context)
    task_config = "config/static/sim_task.yaml" if sim else "config/real_mission.yaml"
    python = ["bash", str(lab / "scripts/python.sh")]

    return [
        ExecuteProcess(
            cmd=[*python, str(package / "decode_camera_h264.py"), "--robot-name", robot,
                 "--input-topic", input_topic, "--input-localhost-only", input_localhost_only],
            name="decode_camera_h264",
            output="screen",
            on_exit=_stop_session,
        ),
        ExecuteProcess(
            cmd=[*python, str(lab / "scripts/aruco.py"), "--robot-name", robot,
                 "--use-sim-time", use_sim_time, "--task-config", str(lab / task_config)],
            name="aruco",
            output="screen",
            on_exit=_stop_session,
        ),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("robot_name", description="Full robot name, e.g. tb_ada"),
        DeclareLaunchArgument("use_sim_time", default_value="true", choices=["true", "false"]),
        DeclareLaunchArgument("input_topic", default_value=""),
        DeclareLaunchArgument("input_localhost_only", default_value=LaunchConfiguration("use_sim_time"),
                              choices=["true", "false"]),
        OpaqueFunction(function=_launch_nodes),
    ])
