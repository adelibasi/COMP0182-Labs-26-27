#!/usr/bin/env python3
"""EXERCISE — Minimal ROS 2 publisher.

Goal: drive the robot forward by publishing a geometry_msgs/Twist on a timer.
Run (turtlesim):  python minimal_publisher.py --ros-args -r cmd_vel:=/turtle1/cmd_vel

Fill in each TODO.
"""
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist


class MinimalPublisher(Node):
    def __init__(self):
        super().__init__('minimal_publisher')
        # TODO 1: Create a Twist publisher on 'cmd_vel' (queue depth 10).
        # TODO 2: Make timer_callback run every 0.5 seconds.
        self.get_logger().info('minimal_publisher started')

    def timer_callback(self):
        msg = Twist()
        # TODO 3: Choose a small forward speed in msg.linear.x (m/s).
        # TODO 4: Send msg using your publisher; creating it alone sends nothing.
        pass


def main(args=None):
    rclpy.init(args=args)
    node = MinimalPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
