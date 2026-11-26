"""Check the packaged mission against simulated action, TF and camera responses."""

from collections import deque
import json
import math
from pathlib import Path
import time
from types import SimpleNamespace
from unittest.mock import Mock, patch

from action_msgs.msg import GoalStatus
import rclpy
from rclpy.task import Future
from rclpy.time import Time
from tf2_ros import TransformException

from mission.autonomous_search import EXERCISE_FILE, load_exercise
from mission.node import mission_goals

ORDER = ["room_1", "room_2", "room_3"]
VIEW_POSES = {
    "room_1": (1.0, 2.0, math.pi - 0.1),
    "room_2": (2.0, 2.0, 1.0),
    "room_3": (3.0, 2.0, 2.0),
}
GOAL_POSES = {room: (x, 4.0, -yaw) for room, (x, _, yaw) in VIEW_POSES.items()}
SIGNS = ((7, "banana"), (23, "apple"), (42, "strawberry"))
NOW = Time(nanoseconds=10_000_000_000)


class SimulatedHandles:
    goal_timeout_s = 0.05
    arrival_timeout_s = 0.05
    base_frame = "tb_test/base_footprint"

    def __init__(self, signs, failure=None):
        self.signs, self.failure = signs, failure
        self.visit_order = ORDER.copy()
        self.view_poses = VIEW_POSES.copy()
        self.goal_poses = GOAL_POSES.copy()
        self.target_fruit = "apple"
        self.action = SimpleNamespace(
            wait_for_server=lambda **_: True,
            server_is_ready=lambda: True,
            send_goal_async=self.send_goal,
        )
        self.tf_buffer = SimpleNamespace(lookup_transform=self.lookup_transform)
        self.marker_reader = SimpleNamespace(read_marker=self.read_marker)
        self.events = []
        self.replies = deque()
        self.idle_spins = 0

    def get_clock(self):
        return SimpleNamespace(now=lambda: NOW)

    def get_logger(self):
        return Mock()

    def reply(self, response):
        future = Future()
        self.replies.append((future, response))
        return future

    def spin_once(self, node, *, timeout_sec=None, executor=None):
        if self.replies:
            future, response = self.replies.popleft()
            future.set_result(response)
        else:
            self.idle_spins += 1
            if self.idle_spins > 50:
                raise AssertionError("Wait did not finish within the test time limit")
            time.sleep(0.01)

    def spin_until_future_complete(self, node, future, executor=None, timeout_sec=None):
        if timeout_sec is None or not 0 <= timeout_sec < math.inf:
            raise AssertionError("Use a finite timeout when waiting for Nav2")
        while not future.done() and self.replies:
            self.spin_once(node)
        if not future.done():
            self.spin_once(node)
            time.sleep(min(timeout_sec, self.goal_timeout_s))

    def send_goal(self, goal, **_):
        pose = goal.pose
        if pose.header.frame_id != "map" or pose.header.stamp != NOW.to_msg():
            raise AssertionError(
                "Stamp the Nav2 goal with the ROS clock in the map frame"
            )
        stops = VIEW_POSES | {
            f"{room}.goal_pose": pose for room, pose in GOAL_POSES.items()
        }
        for stop, point in stops.items():
            if (pose.pose.position.x, pose.pose.position.y) == point[:2]:
                break
        else:
            raise AssertionError(f"Unexpected Nav2 goal position {pose.pose.position}")
        rotation = pose.pose.orientation
        yaw = 2.0 * math.atan2(rotation.z, rotation.w)
        if (
            rotation.x != 0.0
            or rotation.y != 0.0
            or not math.isclose(math.hypot(rotation.z, rotation.w), 1.0)
            or not math.isclose(
                math.remainder(yaw - point[2], math.tau), 0.0, abs_tol=1e-6
            )
        ):
            raise AssertionError(
                f"{stop}: goal orientation does not use the supplied heading"
            )
        self.stop, self.pose = stop, point
        self.events.append(("go", stop))
        self.result = None
        rejected = stop == "room_2" and self.failure == "rejected"
        return self.reply(
            SimpleNamespace(
                accepted=not rejected,
                get_result_async=self.goal_result,
                cancel_goal_async=self.cancel_goal,
            )
        )

    def goal_result(self):
        failure = self.failure if self.stop == "room_2" else None
        if failure == "rejected":
            raise AssertionError("Check goal acceptance before requesting the result")
        status = (
            GoalStatus.STATUS_ABORTED
            if failure == "aborted"
            else GoalStatus.STATUS_SUCCEEDED
        )
        self.result = (
            Future()
            if failure == "timeout"
            else self.reply(SimpleNamespace(status=status))
        )
        return self.result

    def cancel_goal(self):
        self.events.append(("cancel", self.stop))
        return self.reply(SimpleNamespace(return_code=0, goals_canceling=[object()]))

    def lookup_transform(self, target_frame, source_frame, time, timeout=None):
        if (
            self.result is None
            or not self.result.done()
            or self.result.result().status != GoalStatus.STATUS_SUCCEEDED
        ):
            raise AssertionError(
                "Wait for a successful Nav2 result before checking arrival"
            )
        if (target_frame, source_frame) != ("map", self.base_frame):
            raise AssertionError(
                "Verify arrival using map to the robot's base_footprint"
            )
        event = ("tf", self.stop)
        if self.events[-1] != event:
            self.events.append(event)
        x, y, yaw = self.pose
        if self.stop == "room_1":
            yaw = -math.pi + 0.1
        if self.stop == "room_2" and self.failure == "heading":
            yaw += 1.0
        if (self.stop, self.failure) in (("room_2", "position"), ("room_2.goal_pose", "parking")):
            x += 1.0
        if self.stop == "room_2" and self.failure == "missing TF":
            raise TransformException("No transform yet")
        return SimpleNamespace(
            transform=SimpleNamespace(
                translation=SimpleNamespace(x=x, y=y, z=0.0),
                rotation=SimpleNamespace(
                    x=0.0, y=0.0, z=math.sin(yaw / 2), w=math.cos(yaw / 2)
                ),
            )
        )

    def read_marker(self, timeout_s=None):
        self.events.append(("read", self.stop))
        if self.stop == "room_2" and self.failure == "unreadable":
            return None
        index = ORDER.index(self.stop)
        return (*self.signs[index], NOW.nanoseconds + index)


def check_recorded_poses():
    rooms = {
        room: {"view_pose": VIEW_POSES[room], "goal_pose": GOAL_POSES[room]}
        for room in ORDER
    }
    recorded = {
        "target_fruit": "apple",
        "start_pose": [1.0, 0.5, 0.0],
        "rooms": rooms,
    }

    config_dir = Path("config")
    sim_settings = {
        "marker_id_to_fruit": {23: "apple"},
        "observation": {"consistent_frames": 3},
    }
    real = recorded | {"target_fruit": "pear", "marker_id_to_fruit": {11: "pear"}}

    def load(sim, data):
        with (
            patch.object(Path, "read_text", autospec=True, return_value=json.dumps(data)) as read,
            patch("mission.node.get_package_share_directory", return_value="share/mission") as share,
            patch("mission.node.load_task", return_value=sim_settings) as fixture,
        ):
            result = mission_goals(config_dir, sim=sim)
        filename = "sim_mission.yaml" if sim else "real_mission.yaml"
        read.assert_called_once_with(config_dir / filename, encoding="utf-8")
        if sim:
            share.assert_called_once_with("mission")
            fixture.assert_called_once_with(Path("share/mission/config/sim_task.yaml"))
        else:
            share.assert_not_called()
            fixture.assert_not_called()
        return result

    room_3 = rooms["room_3"]
    invalid_configs = (
        ({"start_pose": [None, None, None]}, "start_pose"),
        ({"rooms": {"room_1": rooms["room_1"]}}, "room_1, room_2 and room_3"),
        ({"rooms": rooms | {"room_3": None}}, "room_3"),
        ({"rooms": rooms | {"room_3": room_3 | {"view_pose": [None, None, None]}}}, "room_3.view_pose"),
        ({"rooms": rooms | {"room_3": room_3 | {"view_pose": [0, 0, True]}}}, "room_3.view_pose"),
        ({"rooms": rooms | {"room_3": {"view_pose": VIEW_POSES["room_3"]}}}, "room_3.goal_pose"),
        ({"rooms": rooms | {"room_3": room_3 | {"goal_pose": [None, None, None]}}}, "room_3.goal_pose"),
        ({"rooms": rooms | {"room_3": room_3 | {"goal_pose": [2, 4]}}}, "room_3.goal_pose"),
        ({"rooms": rooms | {"room_3": room_3 | {"goal_pose": VIEW_POSES["room_3"]}}}, "at least 0.3 m"),
        ({"target_fruit": "kiwi"}, "target_fruit"),
    )
    for sim, data, observation in ((True, recorded, sim_settings["observation"]), (False, real, None)):
        expected = data["target_fruit"], VIEW_POSES, GOAL_POSES, observation
        if load(sim, data) != expected:
            raise AssertionError("The mission must use that mode's recorded poses and fruit")

        for invalid, message in invalid_configs:
            try:
                load(sim, data | invalid)
            except ValueError as error:
                if message not in str(error):
                    raise AssertionError(f"Expected {message}, got {error}") from error
            else:
                raise AssertionError(f"Expected the mission to reject invalid {message}")


def check(node_type=None):
    check_recorded_poses()
    if node_type is None:
        node_type = load_exercise(Path(__file__).resolve().parents[1] / EXERCISE_FILE)

    class SimulatedSearch(SimulatedHandles, node_type):
        pass

    def run(robot):
        with patch.multiple(
            rclpy,
            ok=lambda: True,
            spin_once=robot.spin_once,
            spin_until_future_complete=robot.spin_until_future_complete,
        ):
            return robot.run_mission()

    views = [(event, room) for room in ORDER for event in ("go", "tf", "read")]
    for signs, expected in ((SIGNS, "room_2"), ((SIGNS[1], SIGNS[0], SIGNS[2]), "room_1")):
        robot = SimulatedSearch(signs)
        chosen = run(robot)
        goal = f"{expected}.goal_pose"
        if chosen != expected or robot.events != views + [("go", goal), ("tf", goal)]:
            raise AssertionError(
                f"Expected verified sign readings and {goal}, got {chosen} and {robot.events}"
            )

    for failure, signs, events in (
        ("rejected", SIGNS, views[:4]),
        ("aborted", SIGNS, views[:4]),
        ("timeout", SIGNS, views[:4] + [("cancel", "room_2")]),
        ("heading", SIGNS, views[:5]),
        ("position", SIGNS, views[:5]),
        ("missing TF", SIGNS, views[:5]),
        ("unreadable", SIGNS, views[:6]),
        ("ambiguous", (SIGNS[1], SIGNS[1], SIGNS[2]), views),
        ("no match", (SIGNS[0], SIGNS[0], SIGNS[2]), views),
        ("parking", SIGNS, views + [("go", "room_2.goal_pose"), ("tf", "room_2.goal_pose")]),
    ):
        robot = SimulatedSearch(signs, failure)
        try:
            run(robot)
        except NotImplementedError:
            raise
        except (RuntimeError, TimeoutError):
            pass
        else:
            raise AssertionError(f"Mission must fail on {failure}")
        if robot.events != events:
            raise AssertionError(f"Unexpected actions on {failure}: {robot.events}")


if __name__ == "__main__":
    try:
        check()
    except NotImplementedError as error:
        raise SystemExit(f"TODO: {error}")
    except AssertionError as error:
        raise SystemExit(f"FAIL: {error}")
    print(
        "PASS: stamped goals, cancellations, arrival checks and sign-based selection."
    )
