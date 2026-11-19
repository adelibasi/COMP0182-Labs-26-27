"""Bounded, read-only check of a demonstrator-started Humble TurtleBot."""

import math
import re
import sys
import time

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import LaserScan
from tf2_ros import Buffer, TransformListener


def main():
    if len(sys.argv) != 2 or not re.fullmatch(r"tb_[A-Za-z0-9_]+", sys.argv[1]):
        print("robot_name must be tb_<NAME> (e.g. tb_ada)", file=sys.stderr)
        return 2
    name = sys.argv[1]
    rclpy.init()
    node = Node("lab6_hardware_check")
    seen = {"scan": None, "odom": None}
    node.create_subscription(
        LaserScan,
        f"/{name}/scan",
        lambda msg: seen.__setitem__("scan", msg),
        qos_profile_sensor_data,
    )
    node.create_subscription(
        Odometry,
        f"/{name}/odom",
        lambda msg: seen.__setitem__("odom", msg),
        qos_profile_sensor_data,
    )
    buffer = Buffer()
    listener = TransformListener(buffer, node)
    # stop waiting when a sensor or transform is missing
    deadline = time.monotonic() + 12
    try:
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
            if seen["scan"] and seen["odom"]:
                scan, odom = seen["scan"], seen["odom"]
                ready = scan.ranges and all(
                    math.isfinite(v)
                    for v in (odom.pose.pose.position.x, odom.pose.pose.position.y)
                )
                if (
                    ready
                    and buffer.can_transform(odom.header.frame_id, odom.child_frame_id, Time())
                    and buffer.can_transform(odom.child_frame_id, scan.header.frame_id, Time())
                ):
                    break

        problems = []
        scan, odom = seen["scan"], seen["odom"]
        if not scan or not scan.ranges:
            problems.append(f"no live LaserScan on /{name}/scan")

        if not odom:
            problems.append(f"no live Odometry on /{name}/odom")
        elif not all(
            math.isfinite(v) for v in (odom.pose.pose.position.x, odom.pose.pose.position.y)
        ):
            problems.append("odometry position is not finite")

        if scan and odom:
            if not buffer.can_transform(odom.header.frame_id, odom.child_frame_id, Time()):
                problems.append(f"missing TF {odom.header.frame_id} -> {odom.child_frame_id}")

            if not buffer.can_transform(odom.child_frame_id, scan.header.frame_id, Time()):
                problems.append(f"missing TF {odom.child_frame_id} -> {scan.header.frame_id}")

        subscribers = node.get_subscriptions_info_by_topic(f"/{name}/cmd_vel")
        if not any(endpoint.topic_type == "geometry_msgs/msg/Twist" for endpoint in subscribers):
            problems.append(f"/{name}/cmd_vel has no Humble geometry_msgs/msg/Twist subscriber")

        if problems:
            print("Hardware check failed: " + "; ".join(problems), file=sys.stderr)

            return 1

        print(f"Ready: /{name}/scan, /{name}/odom, TF and Twist.")

        return 0
    finally:
        del listener
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
