#!/usr/bin/env python3
"""Passively record simulator truth, wheel odometry and a localiser for Lab 5.

Run while stationary. Follow KEEP STILL / DRIVE / STOP in this terminal.
Every run gets a new output folder. Interrupted or malformed captures are kept
but marked incomplete/invalid; compare_poses.py refuses to report their RMSE.

AMCL note: this records amcl_pose, whose Humble header is the original scan
timestamp, NOT the future-dated map->odom TF timestamp. AMCL often publishes
only after movement. If waiting for a fresh pose, set 2D Pose Estimate in RViz
while stationary. This recorder never requests filter updates or moves the robot.
Source: https://api.nav2.org/nav2-humble/html/amcl__node_8cpp_source.html
"""

import argparse
import csv
import json
import math
import os
import re
import sys
from datetime import datetime, timezone
from pathlib import Path
from time import monotonic
from uuid import uuid4

from compare_poses import CSV_FIELDS, NS, Series, parse_row

MAPPER_PARAMETERS = (
    "Optimizer/PriorsIgnored",
    "RGBD/ProximityBySpace",
    "RGBD/ProximityByTime",
    "RGBD/NeighborLinkRefining",
    "Rtabmap/DetectionRate",
    "Grid/CellSize",
    "Reg/Strategy",
    "frame_id",
    "odom_frame_id",
    "database_path",
)


def decode_parameters(names, values):
    """Decode the scalar parameters requested by this read-only snapshot."""
    if len(names) != len(values):
        raise ValueError("Parameter response count mismatch")
    output = {}
    # rcl_interfaces/msg/ParameterType constants: not set=0, bool=1,
    # integer=2, double=3, string=4. Keep this helper importable without ROS.
    attributes = {1: "bool_value", 2: "integer_value", 3: "double_value", 4: "string_value"}
    for name, value in zip(names, values):
        if value.type == 0:
            output[name] = None
        elif value.type in attributes:
            data = getattr(value, attributes[value.type])
            if isinstance(data, float) and not math.isfinite(data):
                raise ValueError("Non-finite parameter value")
            output[name] = data
        else:
            raise ValueError("Unexpected non-scalar mapper parameter")
    return output


def message_row(message, child=""):
    stamp = message.header.stamp
    if not (0 <= stamp.nanosec < NS) or stamp.sec < 0:
        raise ValueError("Malformed ROS message timestamp")
    nanoseconds = int(stamp.sec) * NS + int(stamp.nanosec)
    pose, cov = message.pose.pose, message.pose.covariance
    if len(cov) != 36 or not all(math.isfinite(value) for value in cov):
        raise ValueError("Pose covariance must contain 36 finite values")
    return {
        "stamp_ns": nanoseconds,
        "t_sim_s": f"{nanoseconds/NS:.9f}",
        "frame_id": message.header.frame_id,
        "child_frame_id": child,
        "x": pose.position.x,
        "y": pose.position.y,
        "z": pose.position.z,
        "qx": pose.orientation.x,
        "qy": pose.orientation.y,
        "qz": pose.orientation.z,
        "qw": pose.orientation.w,
        "cxx": cov[0],
        "cxy": cov[1],
        "cyx": cov[6],
        "cyy": cov[7],
        "cyaw": cov[35],
    }


class RecordingState:
    """Small ROS-independent state machine; times are native simulation stamps."""

    def __init__(self, duration):
        self.duration_ns = round(duration * NS)
        self.samples = {name: [] for name in ("ground_truth", "odometry", "estimator")}
        self.start_ns = None
        self.driving = False
        self.stopping = False
        self.finished = False
        self.last_progress = 0

    @property
    def timing(self):
        if self.start_ns is None:
            return {}
        return {
            "start_ns": self.start_ns,
            "calibration_end_ns": self.start_ns + 8 * NS,
            "stop_ns": self.start_ns + self.duration_ns - 5 * NS,
            "end_ns": self.start_ns + self.duration_ns,
        }

    def add(self, name, sample):
        stream = self.samples[name]
        if stream and sample.stamp_ns <= stream[-1].stamp_ns:
            raise ValueError(
                f"{name}: duplicate/backwards timestamp; do not reset Gazebo during recording"
            )
        stream.append(sample)
        messages = []
        if self.start_ns is None and all(self.samples.values()):
            candidate = self.samples["estimator"][-1].stamp_ns
            truth = Series(self.samples["ground_truth"], "ground_truth")
            odom = Series(self.samples["odometry"], "odometry")
            # Wait for an actual overlapping fresh pose, not a stale latched pose.
            if (
                0 <= truth.stamps[-1] - candidate <= NS
                and truth.at(candidate) is not None
                and odom.at(candidate) is not None
            ):
                self.start_ns = candidate
                messages.extend(
                    ["READY — RECORDING STARTED", "KEEP STILL for the first 8 simulation seconds."]
                )
        if self.start_ns is None:
            return messages
        now = self.samples["ground_truth"][-1].stamp_ns
        elapsed = (now - self.start_ns) / NS
        if elapsed >= 8 and not self.driving:
            self.driving = True
            messages.append("DRIVE — move slowly, turn and revisit a scanned area; avoid walls.")
        if now >= self.timing["stop_ns"] and not self.stopping:
            self.stopping = True
            messages.append(
                "STOP — stop the robot with teleop and keep still until CAPTURE COMPLETE."
            )
        # Two seconds of extra truth allow delayed native estimator messages to arrive.
        if now >= self.timing["end_ns"] + 2 * NS:
            self.finished = True
        progress = int(max(0, elapsed) // 10)
        if progress > self.last_progress and not self.finished:
            self.last_progress = progress
            messages.append(
                f"Recorded {elapsed:.0f}/{self.duration_ns/NS:.0f} simulation seconds."
            )
        return messages


def create_run(out_dir, name, estimator):
    out_dir.mkdir(parents=True, exist_ok=True)
    run = out_dir / (
        f"run_{datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%SZ')}"
        f"_{name}_{estimator}_{uuid4().hex[:8]}"
    )
    run.mkdir()
    return run


def save_metadata(run, metadata):
    temporary = run / "metadata.pending.json"
    with temporary.open("w", encoding="utf-8") as handle:
        json.dump(metadata, handle, indent=2, allow_nan=False)
        handle.write("\n")
    temporary.replace(run / "metadata.json")


def cli():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--robot-name", required=True)
    parser.add_argument("--estimator", choices=("rtabmap", "amcl"), default="rtabmap")
    parser.add_argument(
        "--duration",
        type=float,
        default=120,
        help="Simulation seconds, including 8 s initial / 5 s final stillness",
    )
    parser.add_argument("--out-dir", type=Path, default=Path("output/runs"))
    parser.add_argument(
        "--label", default="", help="Optional experiment note (e.g. proximity off)"
    )
    parser.add_argument(
        "--assisted",
        action="store_true",
        help="Label a truth/mocap-assisted experiment; does not change mapper settings",
    )
    args = parser.parse_args()
    if not re.fullmatch(r"[A-Za-z0-9_]+", args.robot_name):
        parser.error("robot-name must contain only letters, digits or underscores")
    if not math.isfinite(args.duration) or not 30 <= args.duration <= 3600:
        parser.error("duration must be between 30 and 3600 simulation seconds")
    return args


def run_recording(args):
    # Defer ROS imports so helpers and tests remain available without ROS.
    import rclpy
    from geometry_msgs.msg import PoseWithCovarianceStamped
    from nav_msgs.msg import Odometry
    from rcl_interfaces.srv import GetParameters
    from rclpy.executors import ExternalShutdownException
    from rclpy.node import Node
    from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

    ns = f"tb_{args.robot_name}"
    topics = {
        "ground_truth": f"/{ns}/mocap/odom",
        "odometry": f"/{ns}/odom",
        "estimator": "/localization_pose" if args.estimator == "rtabmap" else f"/{ns}/amcl_pose",
    }
    frames = {
        "ground_truth": ("world", {"base_footprint", f"{ns}/base_footprint"}),
        "odometry": (f"{ns}/odom", {f"{ns}/base_footprint"}),
        "estimator": ("map", {""}),
    }
    run = create_run(args.out_dir, args.robot_name, args.estimator)
    state = RecordingState(args.duration)
    metadata = {
        "schema_version": 1,
        "status": "waiting",
        "robot_name": args.robot_name,
        "estimator": args.estimator,
        "label": args.label,
        "assisted": args.assisted,
        "topics": topics,
        "duration_s": args.duration,
        "calibration_s": 8,
        "final_still_s": 5,
        "ros_domain_id": os.environ.get("ROS_DOMAIN_ID", ""),
        "created_utc": datetime.now(timezone.utc).isoformat(),
        "timestamp_policy": "native pose header stamps, never future-dated TF",
        "ground_truth_usage": "This recorder is passive. Check mapper settings separately for truth-assisted runs.",
        "mapper_snapshot": {
            "status": "pending" if args.estimator == "rtabmap" else "not_applicable"
        },
    }
    save_metadata(run, metadata)
    handles, writers = {}, {}
    node = None
    error = None
    started_wall, last_truth_wall = monotonic(), monotonic()
    print("Output:", run, flush=True)
    print(
        "WAITING — keep the robot still; waiting for fresh truth, odometry and estimator poses.",
        flush=True,
    )
    if args.estimator == "amcl":
        print(
            "AMCL: if waiting, set 2D Pose Estimate in RViz while the robot is stationary.",
            flush=True,
        )

    class Recorder(Node):
        def __init__(self):
            super().__init__("lab5_pose_recorder")
            sensor_qos = QoSProfile(
                depth=200,
                reliability=ReliabilityPolicy.BEST_EFFORT,
                durability=DurabilityPolicy.VOLATILE,
            )
            pose_qos = QoSProfile(
                depth=100,
                reliability=ReliabilityPolicy.RELIABLE,
                durability=DurabilityPolicy.VOLATILE,
            )
            self.create_subscription(
                Odometry,
                topics["ground_truth"],
                lambda m: self.receive("ground_truth", m),
                sensor_qos,
            )
            self.create_subscription(
                Odometry, topics["odometry"], lambda m: self.receive("odometry", m), sensor_qos
            )
            self.create_subscription(
                PoseWithCovarianceStamped,
                topics["estimator"],
                lambda m: self.receive("estimator", m),
                pose_qos,
            )
            self.parameter_client = (
                self.create_client(GetParameters, "/rtabmap/get_parameters")
                if args.estimator == "rtabmap"
                else None
            )
            self.parameter_future = None
            self.parameter_deadline = monotonic() + 5.0

        def poll_parameters(self):
            """Nonblocking, bounded read-only audit; capture never depends on it."""
            if metadata["mapper_snapshot"]["status"] != "pending":
                return
            try:
                if self.parameter_future is None and self.parameter_client.service_is_ready():
                    request = GetParameters.Request()
                    request.names = list(MAPPER_PARAMETERS)
                    self.parameter_future = self.parameter_client.call_async(request)
                if self.parameter_future is not None and self.parameter_future.done():
                    values = self.parameter_future.result().values
                    metadata["mapper_snapshot"] = {
                        "status": "captured",
                        "parameters": decode_parameters(MAPPER_PARAMETERS, values),
                        "captured_utc": datetime.now(timezone.utc).isoformat(),
                        "scope": "single read-only startup snapshot, not continuous monitoring",
                    }
                    save_metadata(run, metadata)
                elif monotonic() > self.parameter_deadline:
                    metadata["mapper_snapshot"] = {
                        "status": "unavailable",
                        "reason": "read-only parameter service timed out after 5 wall seconds",
                    }
                    save_metadata(run, metadata)
            except Exception as exc:
                metadata["mapper_snapshot"] = {"status": "unavailable", "reason": str(exc)}
                save_metadata(run, metadata)

        def receive(self, name, message):
            nonlocal last_truth_wall, error
            if error is not None or state.finished:
                return
            try:
                row = message_row(message, getattr(message, "child_frame_id", ""))
                sample = parse_row(row, *frames[name])
                messages = state.add(name, sample)
                writers[name].writerow(row)
                if name == "ground_truth":
                    last_truth_wall = monotonic()
                if state.start_ns is not None and metadata["status"] == "waiting":
                    metadata.update(state.timing, status="recording")
                    save_metadata(run, metadata)
                for message in messages:
                    print(message, flush=True)
            except (ValueError, OSError) as exc:
                error = str(exc)

    try:
        for name in topics:
            handles[name] = (run / f"{name}.csv").open("x", newline="", encoding="utf-8")
            writers[name] = csv.DictWriter(handles[name], fieldnames=CSV_FIELDS)
            writers[name].writeheader()
        rclpy.init()
        node = Recorder()
        while rclpy.ok() and not state.finished and error is None:
            rclpy.spin_once(node, timeout_sec=0.2)
            node.poll_parameters()
            if state.start_ns is None and monotonic() - started_wall > 60:
                error = "No fresh overlapping streams after 60 s. Check topics, initialise the estimator and retry."
            elif monotonic() - last_truth_wall > 30:
                error = "No simulator ground truth for 30 s. Is Gazebo paused or stopped?"
        if not state.finished and error is None:
            error = "Recording interrupted before CAPTURE COMPLETE"
    except (KeyboardInterrupt, ExternalShutdownException):
        error = "Recording interrupted before CAPTURE COMPLETE"
    except Exception as exc:
        error = str(exc)
    finally:
        for handle in handles.values():
            handle.close()
        if node is not None:
            node.destroy_node()
        rclpy.try_shutdown()
        metadata.update(
            state.timing,
            status="complete" if state.finished and error is None else "incomplete",
            stream_counts={name: len(samples) for name, samples in state.samples.items()},
        )
        if error is not None:
            metadata["error"] = error
        save_metadata(run, metadata)
    if error is not None:
        print("STOP:", error, file=sys.stderr)
        print("Partial data preserved:", run, file=sys.stderr)
        return 1
    print("CAPTURE COMPLETE — keep the robot stopped. Run the independent checks next.")
    print(f"python3 scripts/compare_poses.py {str(run)!r}")
    return 0


if __name__ == "__main__":
    try:
        sys.exit(run_recording(cli()))
    except (OSError, ValueError, ModuleNotFoundError) as exc:
        print("STOP:", exc, file=sys.stderr)
        sys.exit(1)
