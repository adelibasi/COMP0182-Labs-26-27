"""Convert simulated world odometry to RTAB-Map's mocap pose."""

import sys
from math import radians

import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from rclpy.qos import qos_profile_sensor_data
from rclpy.signals import SignalHandlerOptions


def global_pose(odometry, namespace):
    message = PoseWithCovarianceStamped()
    message.header.stamp = odometry.header.stamp
    # RTAB-Map's global_pose input treats header.frame_id as the tracked
    # sensor/body frame for sensor-to-base TF, NOT as the global reference
    # frame. Coordinates below remain in Gazebo's world frame. This adapter
    # is for that RTAB-Map input only; use mocap/odom for ordinary ground truth.
    message.header.frame_id = f"{namespace}/base_footprint"
    message.pose.pose = odometry.pose.pose
    # numerical weighting assumption, not a measured covariance
    for index in (0, 7, 14):
        message.pose.covariance[index] = 0.001**2
    for index in (21, 28, 35):
        message.pose.covariance[index] = radians(0.05) ** 2
    return message


def main():
    namespace = sys.argv[1]
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = rclpy.create_node("mocap", namespace=namespace)
    publisher = node.create_publisher(PoseWithCovarianceStamped, "mocap/pose", 1)
    node.create_subscription(
        Odometry,
        "mocap/odom",
        lambda message: publisher.publish(global_pose(message, namespace)),
        qos_profile_sensor_data,
    )
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
