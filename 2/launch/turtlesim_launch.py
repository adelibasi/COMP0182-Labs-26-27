#!/usr/bin/env python3
"""
Lab 2 — Launch file

Starts turtlesim and turtle_teleop_key together.
Run with: ros2 launch launch/turtlesim_launch.py
"""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='turtlesim',
            executable='turtlesim_node',
            name='turtlesim',
            output='screen'
        ),
        Node(
            package='turtlesim',
            executable='turtle_teleop_key',
            name='teleop',
            output='screen',
            prefix='xterm -e'  # opens teleop in a separate terminal (Linux/macOS)
        ),
    ])