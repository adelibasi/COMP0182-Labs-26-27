#!/usr/bin/env python3
"""EXERCISE — Open-loop square driver.

Goal: drive a 1 m square by publishing Twist on a timer, with NO feedback.
A timed state machine: FORWARD for FORWARD_TIME seconds, then TURN for
TURN_TIME seconds, repeated four times, then stop.

Run (turtlesim):  python square_driver.py --ros-args -r cmd_vel:=/turtle1/cmd_vel
Run (robot):      python square_driver.py

Fill in each TODO.
"""
import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

SIDE_LENGTH = 1.0      # metres
LINEAR_SPEED = 0.2     # m/s
ANGULAR_SPEED = 0.5    # rad/s

# TODO 1: Calculate FORWARD_TIME and TURN_TIME in seconds.
#         A 90-degree turn must use radians with ANGULAR_SPEED.
# FORWARD_TIME = ...
# TURN_TIME = ...
CONTROL_PERIOD = 0.05  # seconds (20 Hz)


class SquareDriver(Node):
    def __init__(self):
        super().__init__('square_driver')
        self.publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        self.timer = self.create_timer(CONTROL_PERIOD, self.tick)
        self.phase = 'FORWARD'
        self.legs_done = 0
        self.phase_elapsed = 0.0
        self.get_logger().info('square_driver started')

    def tick(self):
        msg = Twist()

        if self.phase == 'FORWARD':
            # TODO 2: Command forward motion with no rotation.
            # TODO 3: Switch to TURN when the forward phase finishes.
            #         What must happen to phase_elapsed at a transition?
            pass

        elif self.phase == 'TURN':
            # TODO 4: Command one corner turn without forward motion.
            # TODO 5: Count a completed leg-and-turn cycle, then select
            #         FORWARD or DONE and start the next phase's timing.
            pass

        if self.phase == 'DONE':
            self.publisher.publish(Twist())
            self.timer.cancel()
            return

        self.publisher.publish(msg)
        self.phase_elapsed += CONTROL_PERIOD


def main(args=None):
    rclpy.init(args=args)
    node = SquareDriver()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.publisher.publish(Twist())  # safety: stop the robot
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
