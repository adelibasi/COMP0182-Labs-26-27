"""Read fresh, consistent ArUco observations from the vision topic."""

import json
import math
import time


def observation_time_ns(observation):
    """Allow the camera stamp to lead the publisher's /clock by up to 0.5 seconds."""
    now = observation.get("clock_now_ns")
    if type(now) is not int or now <= 0:
        return None

    stamp = observation.get("source_stamp_ns")
    if (observation.get("status") in ("LIVE", "NO_MARKER") and type(stamp) is int
            and 0 < stamp - now <= 500_000_000):
        return stamp

    return now


def _fresh(observation, max_age_s):
    stamp = observation.get("source_stamp_ns")
    now = observation.get("clock_now_ns")
    age = observation.get("received_age_s")

    return (observation.get("status") in ("LIVE", "NO_MARKER")
            and type(stamp) is int and type(now) is int
            and 0 < stamp <= now and now - stamp <= max_age_s * 1e9
            and type(age) in (int, float) and 0 <= age <= max_age_s)


def _confirmed_marker(observations, after_ns, required_frames, max_age_s):
    if after_ns <= 0:
        return None

    last_stamp = after_ns
    marker, streak = None, 0
    for observation in observations:
        stamp = observation.get("source_stamp_ns")
        live = observation.get("status") == "LIVE"
        if live and type(stamp) is int and stamp <= last_stamp:
            continue

        ids, labels = observation.get("ids"), observation.get("labels")
        if not (live and _fresh(observation, max_age_s)
                and isinstance(ids, list) and len(ids) == 1 and type(ids[0]) is int
                and ids[0] >= 0 and isinstance(labels, list) and len(labels) == 1
                and isinstance(labels[0], str) and labels[0].strip()):
            marker, streak = None, 0
            continue

        last_stamp = stamp
        seen = ids[0], labels[0]
        streak = streak + 1 if seen == marker else 1
        marker = seen
        if streak >= required_frames:
            return seen[0], seen[1], stamp

    return None


class MarkerReader:
    """Read from the running ArUco node. Call from the mission loop after a stop."""

    def __init__(self, node, robot_name, settings=None):
        from rclpy.qos import QoSProfile, ReliabilityPolicy  # rusty-linter: ignore
        from std_msgs.msg import String  # rusty-linter: ignore

        settings = settings or {}
        self._required_frames = settings.get("consistent_frames", 3)
        self._max_age_s = settings.get("max_image_age_s", 1.0)
        self._timeout_s = settings.get("window_seconds", 10.0)
        if (type(self._required_frames) is not int or self._required_frames < 1
                or not math.isfinite(self._max_age_s) or self._max_age_s <= 0
                or not math.isfinite(self._timeout_s) or self._timeout_s <= 0):
            raise ValueError("Marker frame count, age limit and timeout must be positive")

        self._node = node
        self._clock = "ros_sim" if node.get_parameter("use_sim_time").value else "ros_system"
        self._latest = None
        self._subscription = node.create_subscription(
            String, f"/{robot_name}/vision/observations", self._receive,
            QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE))

    def _receive(self, message):
        try:
            observation = json.loads(message.data)
        except ValueError as error:
            self._node.get_logger().warning(f"Unusable camera evidence: {error}")

            return

        if isinstance(observation, dict) and observation.get("clock") == self._clock:
            camera_now = observation_time_ns(observation)
            if camera_now is not None:
                observation["clock_now_ns"] = max(
                    camera_now, self._node.get_clock().now().nanoseconds)
                self._latest = observation

    def _observations(self, timeout_s):
        import rclpy  # rusty-linter: ignore

        if not math.isfinite(timeout_s) or timeout_s <= 0:
            raise ValueError("Camera timeout must be positive and finite")

        deadline = time.monotonic() + timeout_s
        previous = self._latest
        while rclpy.ok() and (remaining := deadline - time.monotonic()) > 0:
            rclpy.spin_once(self._node, timeout_sec=min(0.10, remaining))
            if self._latest is not previous:
                previous = self._latest
                yield previous

    def wait_for_camera(self, timeout_s=60.0):
        topic = self._subscription.topic_name
        self._node.get_logger().info(f"Waiting for fresh frames on {topic}")
        if not any(_fresh(frame, self._max_age_s) for frame in self._observations(timeout_s)):
            raise TimeoutError(f"No fresh frames on {topic} after {timeout_s:g}s. Start make decode.")

    def read_marker(self, timeout_s: float | None = None) -> tuple[int, str, int] | None:
        """Return (ID, label, source stamp in ns) from new frames, or None on timeout."""
        timeout_s = self._timeout_s if timeout_s is None else timeout_s
        camera_now = self._latest["clock_now_ns"] if self._latest is not None else 0
        after_ns = max(self._node.get_clock().now().nanoseconds, camera_now)
        observations = self._observations(timeout_s)
        if after_ns <= 0:
            first = next(observations, None)
            if first is None:
                return None

            after_ns = first["clock_now_ns"]

        return _confirmed_marker(
            observations, after_ns, self._required_frames, self._max_age_s)
