#!/usr/bin/env python3
"""EXERCISE — Closed-loop square driver.

Goal: drive the same 1 m square as square_driver.py, but close the loop —
each leg ends when /odom says you've actually gone far enough or turned far
enough, not when a timer runs out.

Robot only: this needs nav_msgs/Odometry on /odom, which turtlesim doesn't
publish (it publishes turtlesim/msg/Pose on /turtle1/pose instead).

Run (robot):  python closed_loop_square.py

Fill in each TODO.
"""
import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

SIDE_LENGTH = 1.0                    # metres
LINEAR_SPEED = 0.2                   # m/s while driving a leg
ANGULAR_SPEED = 0.5                  # rad/s while turning a corner
DISTANCE_TOLERANCE = 0.02            # metres
ANGLE_TOLERANCE = math.radians(2)    # radians
CONTROL_PERIOD = 0.05                # seconds (20 Hz)


def quaternion_to_yaw(q):
    """Yaw (rotation about z), in radians, from a geometry_msgs/Quaternion.

    Same conversion as odom_monitor.py — reuse it here.
    """
    # TODO 1: Calculate yaw (radians) from q, as in odom_monitor.py.
    #         The zero return below is only a placeholder.
    return 0.0


class ClosedLoopSquare(Node):
    def __init__(self):
        super().__init__('closed_loop_square')
        self.publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        # TODO 2: Subscribe to '/odom' (Odometry) using odom_callback; depth 10.
        self.timer = self.create_timer(CONTROL_PERIOD, self.tick)

        self.pose = None            # (x, y, yaw) from the latest /odom message
        self.leg_start_pose = None  # pose recorded at the start of the current leg
        self.phase = 'FORWARD'
        self.legs_done = 0
        self.get_logger().info('closed_loop_square started')

    def odom_callback(self, msg):
        # TODO 3: Store estimated (x, y, yaw) from the /odom message in
        #         self.pose; obtain yaw through quaternion_to_yaw.
        pass

    def tick(self):
        if self.pose is None:
            return  # no odometry yet

        if self.leg_start_pose is None:
            self.leg_start_pose = self.pose

        msg = Twist()

        if self.phase == 'FORWARD':
            # TODO 4: Measure distance from self.leg_start_pose to self.pose.
            # TODO 5: Keep moving until remaining distance is within
            #         DISTANCE_TOLERANCE; then start TURN from a new pose.
            pass

        elif self.phase == 'TURN':
            # TODO 6: Measure heading change since self.leg_start_pose;
            #         handle wraparound between -pi and +pi.
            # TODO 7: Turn until within ANGLE_TOLERANCE of 90 degrees;
            #         count the cycle, reset the start pose and choose FORWARD/DONE.
            pass

        if self.phase == 'DONE':
            self.publisher.publish(Twist())
            self.timer.cancel()
            return

        self.publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = ClosedLoopSquare()
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
