"""Connect explore_lite to the map, robot frame and Nav2 action."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("robot_name", description="Full robot namespace, e.g. tb_ada"),
            DeclareLaunchArgument("use_sim_time", default_value="true", choices=["true", "false"]),
            Node(
                package="explore_lite",
                executable="explore",
                name="explore_node",
                output="screen",
                parameters=[
                    {
                        "use_sim_time": LaunchConfiguration("use_sim_time"),
                        "costmap_topic": "TODO_MAP_TOPIC",
                        "costmap_updates_topic": "TODO_MAP_UPDATES_TOPIC",
                        "robot_base_frame": "TODO_ROBOT_FRAME",
                        "return_to_init": False,
                    }
                ],
                remappings=[("navigate_to_pose", "TODO_NAVIGATION_ACTION")],
            ),
        ]
    )
