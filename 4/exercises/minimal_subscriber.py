#!/usr/bin/env python3
"""EXERCISE — Minimal ROS 2 subscriber.

Goal: print the turtlesim turtle's pose whenever a message arrives.
Run:  python minimal_subscriber.py   (with turtlesim running)

Fill in each TODO.
"""
import rclpy
from rclpy.node import Node
from turtlesim.msg import Pose


class MinimalSubscriber(Node):
    def __init__(self):
        super().__init__('minimal_subscriber')
        # TODO 1: Subscribe to '/turtle1/pose' (Pose) using pose_callback
        #         with a queue depth of 10.
        self.get_logger().info('minimal_subscriber started')

    def pose_callback(self, msg):
        # TODO 2: Report msg.x, msg.y and msg.theta (heading in radians).
        pass


def main(args=None):
    rclpy.init(args=args)
    node = MinimalSubscriber()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
