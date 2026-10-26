#!/usr/bin/env python3
"""EXERCISE — Odometry monitor.

Goal: subscribe to /odom and print position, heading (degrees), and velocity.
Run it in its own terminal while driving the robot with square_driver.py.

Fill in each TODO.
"""
import math
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry


def quaternion_to_yaw(q):
    """Yaw (rotation about z), in radians, from a geometry_msgs/Quaternion."""
    # TODO 1: Calculate yaw (radians) from quaternion q.
    #         The zero return below is only a placeholder.
    return 0.0


class OdomMonitor(Node):
    def __init__(self):
        super().__init__('odom_monitor')
        # TODO 2: Subscribe to '/odom' (Odometry) using odom_callback; depth 10.
        self.get_logger().info('odom_monitor started')

    def odom_callback(self, msg):
        # TODO 3: Extract position, heading (degrees) and linear/angular
        #         velocity from msg.pose.pose and msg.twist.twist.
        #         Report them together on one readable line.
        pass


def main(args=None):
    rclpy.init(args=args)
    node = OdomMonitor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
