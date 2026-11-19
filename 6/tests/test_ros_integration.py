"""Run a bounded house-simulation check of Nav2 and the student planner."""

import argparse
import json
import math
import os
import re
import secrets
import shutil
import signal
import subprocess
import sys
import tempfile
import time
from pathlib import Path

import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped, Twist
from lifecycle_msgs.msg import State
from lifecycle_msgs.srv import GetState
from nav2_msgs.action import FollowPath, FollowWaypoints, NavigateToPose
from nav_msgs.msg import OccupancyGrid, Odometry
from rcl_interfaces.srv import GetParameters
from rclpy.action import ActionClient
from rclpy.qos import (
    DurabilityPolicy,
    QoSProfile,
    ReliabilityPolicy,
    qos_profile_sensor_data,
)
from rclpy.time import Time
from sensor_msgs.msg import LaserScan
from tf2_ros import Buffer, TransformListener

LAB = Path(__file__).resolve().parents[1]
GOAL = (-1.75, -0.5)
WAYPOINTS = [(-1.5, -0.5), (-1.5, -0.75)]


def pose(x, y):
    return {
        "header": {"frame_id": "map"},
        "pose": {"position": {"x": x, "y": y}, "orientation": {"w": 1.0}},
    }


def position(odom):
    point = odom.pose.pose.position
    return point.x, point.y


def distance(first, last):
    return math.dist(first, last)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--robot-name", required=True)
    parser.add_argument("--expect", choices=("stub", "working", "route"), default="stub")
    args = parser.parse_args()
    name = args.robot_name
    if not re.fullmatch(r"tb_[A-Za-z0-9_]+", name):
        parser.error("--robot-name must be tb_<NAME> (e.g. tb_ada)")
    lab = LAB
    staged_lab = None
    if args.expect == "route":
        staged_lab = tempfile.TemporaryDirectory(prefix="lab6-route-")
        lab = Path(staged_lab.name) / "lab_6"
        shutil.copytree(
            LAB,
            lab,
            ignore=shutil.ignore_patterns(
                "ros2_ws", "ros2_ws.ol", "*.answer*", "*.draft*", "__pycache__"
            ),
        )
        shutil.copyfile(lab / "tests/route_fixture.py", lab / "exercises/planner.py")
    processes = []
    logs = []
    log_dir = Path("/tmp/pgta_mas")
    log_dir.mkdir(exist_ok=True)
    # The headless fixture must not take over a student/physical robot domain.
    assigned_domain = os.environ.get("ROS_DOMAIN_ID")
    choices = [value for value in range(50, 100) if str(value) != assigned_domain]
    test_domain = secrets.choice(choices)
    os.environ["ROS_DOMAIN_ID"] = str(test_domain)
    os.environ["ROS_LOCALHOST_ONLY"] = "1"
    print(f"LAB6_CHECK_DOMAIN={test_domain}", flush=True)
    rclpy.init()
    node = rclpy.create_node("lab6_integration_check")
    seen = {"scan": None, "odom": None, "map": None, "max_cmd": 0.0}
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
    node.create_subscription(
        OccupancyGrid,
        "/map",
        lambda msg: seen.__setitem__("map", msg),
        QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
        ),
    )

    def record_command(msg):
        speed = max(abs(msg.linear.x), abs(msg.angular.z))
        seen["max_cmd"] = max(seen["max_cmd"], speed)

    node.create_subscription(Twist, f"/{name}/cmd_vel", record_command, 10)
    transform_buffer = Buffer()
    listener = TransformListener(transform_buffer, node)

    def wait_for(description, ready, seconds):
        def label():
            return description() if callable(description) else description

        deadline = time.monotonic() + seconds
        while time.monotonic() < deadline:
            for process, process_label in processes:
                if process.poll() is not None:
                    raise AssertionError(
                        f"{process_label} exited while waiting for {label()}"
                    )

            rclpy.spin_once(node, timeout_sec=0.2)
            if ready():
                return

        raise AssertionError(f"timed out waiting for {label()}")

    lifecycle_clients = {}
    lifecycle_futures = {}
    active_nodes = set()

    def lifecycle_active(target):
        if target in active_nodes:
            return True
        client = lifecycle_clients.get(target)
        if client is None:
            client = node.create_client(GetState, f"{target}/get_state")
            lifecycle_clients[target] = client
        if not client.service_is_ready():
            return False
        future = lifecycle_futures.get(target)
        if future is None:
            lifecycle_futures[target] = client.call_async(GetState.Request())
            return False
        if not future.done():
            return False
        lifecycle_futures.pop(target)
        response = future.result()
        if (
            response is not None
            and response.current_state.id == State.PRIMARY_STATE_ACTIVE
        ):
            active_nodes.add(target)
            return True
        return False

    def launch(label, filename, *launch_args):
        with tempfile.NamedTemporaryFile(
            mode="w+",
            prefix=f"lab6-{label}-",
            suffix=".log",
            dir=log_dir,
            delete=False,
        ) as log:
            logs.append(Path(log.name))
            process = subprocess.Popen(
                ["ros2", "launch", str(lab / "launch" / filename), *launch_args],
                stdout=log,
                stderr=subprocess.STDOUT,
                start_new_session=True,
            )

        processes.append((process, label))

    def send(action, action_type, payload):
        process = subprocess.Popen(
            ["ros2", "action", "send_goal", action, action_type, json.dumps(payload)],
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            text=True,
        )
        deadline = time.monotonic() + 45
        while process.poll() is None and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)

        if process.poll() is None:
            process.kill()
            raise AssertionError(f"{action} goal timed out")

        output = process.communicate()[0]
        if process.returncode:
            raise AssertionError(f"{action} command failed: {output[-500:]}")

        return output

    try:
        launch("sim", "sim.launch.py", f"robot_name:={name}", "gui:=false")
        wait_for("live scan and odometry", lambda: seen["scan"] and seen["odom"], 45)
        print("PASS: simulated scan and odometry")

        launch(
            "amcl",
            "amcl.launch.py",
            f"robot_name:={name}",
            f"map_file:={lab / 'maps/house.yaml'}",
            "use_sim_time:=true",
        )
        initial_pose = node.create_publisher(
            PoseWithCovarianceStamped, f"/{name}/initialpose", 10
        )
        wait_for(
            "AMCL initial-pose subscriber",
            lambda: initial_pose.get_subscription_count() > 0,
            20,
        )
        estimate = PoseWithCovarianceStamped()
        estimate.header.frame_id = "map"
        estimate.pose.pose.position.x = -2.0
        estimate.pose.pose.position.y = -0.5
        estimate.pose.pose.orientation.w = 1.0
        initial_pose.publish(estimate)

        wait_for("saved map", lambda: seen["map"] is not None, 20)
        grid = seen["map"]
        column = math.floor((-2.0 - grid.info.origin.position.x) / grid.info.resolution)
        row = math.floor((-0.5 - grid.info.origin.position.y) / grid.info.resolution)
        assert 0 <= column < grid.info.width and 0 <= row < grid.info.height
        spawn_cost = grid.data[row * grid.info.width + column]
        assert spawn_cost == 0, "spawn is not free in map"

        for target in ("/map_server", f"/{name}/amcl"):
            wait_for(
                f"{target} active",
                lambda target=target: lifecycle_active(target),
                20,
            )
        wait_for(
            "map to robot transform",
            lambda: transform_buffer.can_transform(
                "map", f"{name}/base_footprint", Time()
            ),
            20,
        )
        print("PASS: saved map, AMCL and map-to-robot transform")

        launch("nav", "nav.launch.py", f"robot_name:={name}", "use_sim_time:=true")
        nav_nodes = (
            "/planner_server",
            "/controller_server",
            "/behavior_server",
            "/bt_navigator",
            "/waypoint_follower",
        )
        action_clients = (
            (
                "/navigate_to_pose",
                ActionClient(node, NavigateToPose, "/navigate_to_pose"),
            ),
            (
                "/follow_waypoints",
                ActionClient(node, FollowWaypoints, "/follow_waypoints"),
            ),
            ("/follow_path", ActionClient(node, FollowPath, "/follow_path")),
        )
        pending = "Nav2 startup"

        def nav_ready():
            nonlocal pending
            for target in nav_nodes:
                pending = target
                if not lifecycle_active(target):
                    return False
            for action, client in action_clients:
                pending = action
                if not client.server_is_ready():
                    return False
            return True

        wait_for(lambda: f"{pending} ready", nav_ready, 20)
        planner_client = node.create_client(
            GetParameters, "/planner_server/get_parameters"
        )
        planner_future = None

        def student_planner_ready():
            nonlocal planner_future
            if not planner_client.service_is_ready():
                return False
            if planner_future is None:
                planner_future = planner_client.call_async(
                    GetParameters.Request(names=["planner_plugins"])
                )
                return False
            if not planner_future.done():
                return False
            response = planner_future.result()
            return (
                response is not None
                and len(response.values) == 1
                and list(response.values[0].string_array_value) == ["Student"]
            )

        wait_for(
            "Student planner plugin",
            student_planner_ready,
            10,
        )
        print("PASS: Nav2 active with only the Student planner")

        before = position(seen["odom"])
        seen["max_cmd"] = 0.0
        if args.expect in ("working", "route"):
            result = send(
                "/navigate_to_pose",
                "nav2_msgs/action/NavigateToPose",
                {"pose": pose(*GOAL)},
            )
            assert "status: SUCCEEDED" in result, result[-500:]
            assert distance(before, position(seen["odom"])) > 0.05
            before_waypoints = position(seen["odom"])
            result = send(
                "/follow_waypoints",
                "nav2_msgs/action/FollowWaypoints",
                {"poses": [pose(*point) for point in WAYPOINTS]},
            )
            assert "status: SUCCEEDED" in result and "missed_waypoints: []" in result
            moved = distance(before_waypoints, position(seen["odom"]))
            assert moved > 0.05, f"waypoints moved robot only {moved:.3f} m"
            print(f"PASS: navigation goal and two waypoints; waypoint travel {moved:.3f} m")
        else:
            result = send(
                "/navigate_to_pose",
                "nav2_msgs/action/NavigateToPose",
                {"pose": pose(*GOAL)},
            )
            assert "status: ABORTED" in result, result[-500:]
            wait_for("planner error", lambda: "Implement A*" in logs[-1].read_text(), 5)
            assert seen["max_cmd"] < 0.01, "stub produced a velocity command"
            assert distance(before, position(seen["odom"])) < 0.02
            print("PASS: unimplemented A* aborted without movement")

    except AssertionError as error:
        print(f"FAIL: {error}", file=sys.stderr)
        for log in logs:
            lines = log.read_text(errors="replace").splitlines()
            print(f"{log}:\n" + "\n".join(lines[-10:]), file=sys.stderr)

        return 1
    finally:
        for process, _ in reversed(processes):
            if process.poll() is None:
                os.killpg(process.pid, signal.SIGINT)

        for process, _ in reversed(processes):
            try:
                process.wait(timeout=15)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid, signal.SIGKILL)
                process.wait()

        del listener
        node.destroy_node()
        rclpy.shutdown()
        if staged_lab is not None:
            staged_lab.cleanup()

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
