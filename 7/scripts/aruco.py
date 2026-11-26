#!/usr/bin/env python3
"""Lab 7 annotated image publisher and bounded camera check. Publishes no motion."""

import argparse
from collections import deque
import importlib.util
import json
import os
from pathlib import Path
import re
import sys
import time

# the Make launcher uses Python -I -S. Import only this trusted lab's helpers
SCRIPTS = Path(__file__).resolve().parent
sys.path.insert(0, str(SCRIPTS))
from vision_core import ArucoReader, ObservationTracker


def load_student_aruco_detector():
    """Load student detection without importing ROS or giving it a reference answer."""
    path = SCRIPTS.parent / "exercises/aruco_reader.py"
    if not path.is_file():
        raise FileNotFoundError(f"Student ArUco exercise missing: {path}")

    spec = importlib.util.spec_from_file_location("lab7_student_aruco", path)
    if spec is None or spec.loader is None:
        raise RuntimeError(f"Cannot load student ArUco exercise: {path}")

    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    if not callable(getattr(module, "detect_markers", None)):
        raise ValueError("exercises/aruco_reader.py must define detect_markers")

    return module.detect_markers


def arguments(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--robot-name", required=True)
    parser.add_argument("--use-sim-time", choices=("true", "false"), default="true")
    parser.add_argument("--task-config", type=Path,
                        default=SCRIPTS.parent / "config/static/sim_task.yaml")
    parser.add_argument("--image-topic", help="Default: /<robot-name>/camera/video_raw")
    parser.add_argument("--check", action="store_true",
                        help="No GUI: require three usable images, then exit")
    parser.add_argument("--expected-id", type=int,
                        help="In --check mode, require this ID in all three images")
    parser.add_argument("--timeout", type=float, default=60.0,
                        help="Camera startup deadline in seconds (maximum 60)")
    args = parser.parse_args(argv)
    if not re.fullmatch(r"tb_[A-Za-z0-9_]+", args.robot_name):
        parser.error("--robot-name must be tb_<NAME>")

    if not 0 < args.timeout <= 60:
        parser.error("--timeout must be greater than zero and at most 60 seconds")

    if args.expected_id is not None and not args.check:
        parser.error("--expected-id is for --check")

    args.use_sim_time = args.use_sim_time == "true"
    args.image_topic = args.image_topic or f"/{args.robot_name}/camera/video_raw"
    if not args.image_topic.startswith("/"):
        parser.error("--image-topic must be an absolute ROS topic name")

    return args


def run(args, task_config):
    # match the decoder's local participant and ignore custom network transports
    os.environ["ROS_LOCALHOST_ONLY"] = "1"
    os.environ["RMW_IMPLEMENTATION"] = "rmw_fastrtps_cpp"
    os.environ["SKIP_DEFAULT_XML"] = "1"
    for key in ("FASTRTPS_DEFAULT_PROFILES_FILE", "FASTDDS_DEFAULT_PROFILES_FILE",
                "FASTDDS_ENVIRONMENT_FILE"):
        os.environ.pop(key, None)

    # load ROS and OpenCV dependencies after configuring DDS
    import numpy as np  # rusty-linter: ignore
    import rclpy  # rusty-linter: ignore
    from cv_bridge import CvBridge  # rusty-linter: ignore
    from rclpy.executors import ExternalShutdownException  # rusty-linter: ignore
    from rclpy.node import Node  # rusty-linter: ignore
    from rclpy.parameter import Parameter  # rusty-linter: ignore
    from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy  # rusty-linter: ignore
    from sensor_msgs.msg import Image  # rusty-linter: ignore
    from std_msgs.msg import String  # rusty-linter: ignore

    reader = None if args.check and args.expected_id is None else ArucoReader(task_config)
    student_detector = load_student_aruco_detector() if not args.check else None
    if args.expected_id is not None and not 0 <= args.expected_id < reader.marker_count:
        raise ValueError("--expected-id is outside the configured marker dictionary")

    class Vision(Node):
        def __init__(self):
            super().__init__("lab7_camera_check" if args.check else "lab7_aruco",
                             namespace=f"/{args.robot_name}",
                             parameter_overrides=[Parameter(
                                 "use_sim_time", Parameter.Type.BOOL, args.use_sim_time)])
            self.bridge, self.tracker = CvBridge(), ObservationTracker()
            self.started = time.monotonic()
            self.last_emit = 0.0
            self.last_console = None
            self.done, self.passed, self.failed = False, False, False
            self.check_frames = deque(maxlen=3)
            self.annotated_frame = None
            self.student_aruco_todo = False
            image_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT,
                                   durability=DurabilityPolicy.VOLATILE)
            self.publisher = self.annotated = None
            if not args.check:
                self.publisher = self.create_publisher(
                    String, "vision/observations", QoSProfile(
                        depth=1, reliability=ReliabilityPolicy.RELIABLE,
                        durability=DurabilityPolicy.VOLATILE))
                self.annotated = self.create_publisher(Image, "camera/video_annotated", image_qos)

            self.subscription = self.create_subscription(
                Image, args.image_topic, self.receive, image_qos)
            self.get_logger().info(
                f"Reading {args.image_topic}; source stamps use "
                f"{'simulation' if args.use_sim_time else 'system'} time. "
                "Receipt timeout: 2 seconds. IDs do not identify robot pose.")

        def publish_image(self, frame, header):
            image = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
            image.header = header
            self.annotated.publish(image)

        def clear_image(self):
            if self.annotated_frame is not None:
                shape, header = self.annotated_frame
                self.publish_image(np.zeros(shape, dtype=np.uint8), header)
                self.annotated_frame = None

        def emit(self, observation, now_ns, now_mono):
            observation["clock"] = "ros_sim" if args.use_sim_time else "ros_system"
            # use one ROS clock for the image and its acceptance window
            # a remote subscriber's /clock callback can lag this publisher
            observation["clock_now_ns"] = now_ns
            if self.publisher is not None:
                self.publisher.publish(String(data=json.dumps(observation, allow_nan=False)))

            self.last_emit = now_mono
            state = (observation["status"], tuple(observation["ids"]),
                     tuple(observation["labels"]), observation["reason"])
            if state != self.last_console:
                detail = ", ".join(f"{i}: {label or 'unknown'}"
                                   for i, label in zip(observation["ids"], observation["labels"]))
                self.get_logger().info(f"{observation['status']}: {detail or observation['reason']}")
                self.last_console = state

        def receive(self, message):
            arrived_at = time.monotonic()
            stamp = message.header.stamp.sec * 10**9 + message.header.stamp.nanosec
            ids, labels, corners, annotated, error = [], [], [], None, None
            try:
                image = self.bridge.imgmsg_to_cv2(message, desired_encoding="bgr8")
                if image.shape != (message.height, message.width, 3) or image.dtype != np.uint8:
                    raise ValueError("decoded image dimensions or encoding do not match the message")
                if reader is not None and student_detector is None:
                    ids, labels, corners = reader.detect(image)
                elif student_detector is not None:
                    try:
                        detected = student_detector(image, reader.dictionary_name)
                    except NotImplementedError:
                        if not self.student_aruco_todo:
                            self.get_logger().warning(
                                "Complete exercises/aruco_reader.py and restart make decode.")
                            self.student_aruco_todo = True

                        ids, labels, corners = [], [], []
                    else:
                        self.student_aruco_todo = False
                        if (not isinstance(detected, (tuple, list))
                                or len(detected) != 2):
                            raise ValueError("Student detector must return (ids, corners)")

                        ids, corners = detected
                        if (not isinstance(ids, list) or not isinstance(corners, list)
                                or len(ids) != len(corners)
                                or any(type(i) is not int or i < 0 or i >= reader.marker_count
                                       for i in ids)):
                            raise ValueError("Student detector returned invalid IDs/corners")

                        if len(set(ids)) != len(ids):
                            raise ValueError("Student detector returned duplicate marker IDs")

                        pairs = sorted(zip(ids, corners), key=lambda pair: pair[0])
                        ids = [pair[0] for pair in pairs]
                        corners = [pair[1] for pair in pairs]
                        labels = [reader.marker_map.get(i) for i in ids]
                if self.annotated is not None:
                    annotated = reader.annotate(image, ids, labels, corners)
            except Exception as caught:
                error = f"image conversion/detection failed: {type(caught).__name__}: {caught}"

            now_mono, now_ns = time.monotonic(), self.get_clock().now().nanoseconds
            accepted = self.tracker.update(
                source_stamp_ns=stamp, frame_id=message.header.frame_id,
                width=message.width, height=message.height, ids=ids, labels=labels,
                now_mono=arrived_at, now_ns=now_ns, error=error)
            observation = self.tracker.snapshot(now_mono, now_ns)
            if accepted and observation["status"] != "STALE":
                if annotated is not None:
                    self.publish_image(annotated, message.header)
                    self.annotated_frame = (annotated.shape, message.header)

                if self.check_frames and (stamp <= self.check_frames[-1][0] or
                                          message.header.frame_id != self.check_frames[-1][3]):
                    self.check_frames.clear()

                self.check_frames.append((stamp, arrived_at, list(ids), message.header.frame_id))
            elif observation["status"] == "STALE":
                self.clear_image()
                self.check_frames.clear()
            else:
                # duplicate/old timestamps cannot renew or republish IDs
                return

            if reader is not None or observation["status"] == "STALE":
                self.emit(observation, now_ns, now_mono)

            if not self.passed and len(self.check_frames) == 3:
                recent = now_mono - self.check_frames[0][1] <= 2.0
                expected = (args.expected_id is None or
                            all(args.expected_id in entry[2] for entry in self.check_frames))
                if recent and expected:
                    self.passed = True
                    if args.check:
                        self.done = True
                        suffix = f"; marker {args.expected_id} detected" if args.expected_id is not None else ""
                        print(f"PASS: three advancing image stamps; {message.width}x{message.height}; "
                              f"frame {message.header.frame_id}{suffix}", flush=True)

        def poll(self):
            # poll from the executor loop, not a simulation-clock timer. Pausing
            # the simulator must not pause the monotonic no-image watchdog or check deadline
            now_mono, now_ns = time.monotonic(), self.get_clock().now().nanoseconds
            observation = self.tracker.snapshot(now_mono, now_ns)
            if observation["status"] == "STALE":
                self.clear_image()
                self.check_frames.clear()
                changed = self.last_console is None or self.last_console[0] != "STALE"
                if changed or now_mono - self.last_emit >= 1.0:
                    self.emit(observation, now_ns, now_mono)

            if not self.passed and now_mono - self.started >= args.timeout:
                if observation["status"] == "STALE":
                    source = ("Start make sim and make decode."
                              if args.use_sim_time else "Start make decode and check robot bringup.")
                    detail = f"{observation['reason']}. {source}"
                elif args.expected_id is not None:
                    detail = f"marker {args.expected_id} not present in three recent images"
                else:
                    detail = "fewer than three usable images"

                print(f"FAIL: camera on {args.image_topic} timed out after {args.timeout:g}s. {detail}",
                      file=sys.stderr, flush=True)
                self.done = self.failed = True

    node = None
    rclpy.init(args=[])
    try:
        node = Vision()
        while rclpy.ok() and not node.done:
            rclpy.spin_once(node, timeout_sec=0.05)
            node.poll()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()

    if args.check:
        return 0 if node is not None and node.passed else 1

    return 1 if node is not None and node.failed else 0


def main(argv=None):
    args = arguments(argv)
    try:
        if args.check and args.expected_id is None:
            return run(args, {})

        # load task settings only for detection or the expected-ID probe
        import yaml  # rusty-linter: ignore

        task_config = yaml.safe_load(args.task_config.read_text())
        if not isinstance(task_config, dict):
            raise ValueError("The task config must contain a YAML mapping")
        return run(args, task_config)
    except Exception as error:
        print(f"FAIL: {type(error).__name__}: {error}", file=sys.stderr)

        return 1


if __name__ == "__main__":
    raise SystemExit(main())
