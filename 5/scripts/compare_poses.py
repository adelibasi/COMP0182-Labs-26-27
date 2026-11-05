#!/usr/bin/env python3
"""Evaluate a Lab 5 recording without ROS.

Compare odometry and a localiser at the localiser's native message timestamps.
Only the high-rate simulator truth and wheel odometry are interpolated, across
short gaps. A separate, fixed SE(2) frame alignment for each estimate is fitted
using the first eight stationary seconds, never the held-out journey.

These are initial-pose-aligned trajectory errors, not absolute map accuracy:
initial pose bias can be absorbed by alignment. Covariance is reported
uncertainty, not measured error or a guaranteed confidence radius.
"""

import argparse
import csv
import json
import math
import re
import sys
from bisect import bisect_left
from dataclasses import dataclass
from datetime import datetime, timezone
from pathlib import Path
from uuid import uuid4

NS = 1_000_000_000
MAX_GAP_NS = 150_000_000
CSV_FIELDS = (
    "stamp_ns",
    "t_sim_s",
    "frame_id",
    "child_frame_id",
    "x",
    "y",
    "z",
    "qx",
    "qy",
    "qz",
    "qw",
    "cxx",
    "cxy",
    "cyx",
    "cyy",
    "cyaw",
)


def wrap(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


@dataclass(frozen=True)
class Sample:
    stamp_ns: int
    pose: tuple
    covariance: tuple  # Cxx, Cxy, Cyx, Cyy, Cyaw


def parse_row(row, expected_frame, allowed_children):
    """Validate native timestamps, frame identity, pose and covariance."""
    try:
        stamp = int(row["stamp_ns"])
        values = [
            float(row[key])
            for key in CSV_FIELDS
            if key not in ("stamp_ns", "frame_id", "child_frame_id")
        ]
        if stamp <= 0 or not all(math.isfinite(value) for value in values):
            raise ValueError("non-finite values or non-positive timestamp")
        if abs(float(row["t_sim_s"]) - stamp / NS) > 1e-7:
            raise ValueError("timestamp columns disagree")
        if row["frame_id"].lstrip("/") != expected_frame:
            raise ValueError(f"expected frame {expected_frame!r}, got {row['frame_id']!r}")
        if row["child_frame_id"].lstrip("/") not in allowed_children:
            raise ValueError(f"unexpected body frame {row['child_frame_id']!r}")
        qx, qy, qz, qw = (float(row[k]) for k in ("qx", "qy", "qz", "qw"))
        norm = math.sqrt(qx * qx + qy * qy + qz * qz + qw * qw)
        if not 0.99 <= norm <= 1.01:
            raise ValueError("orientation quaternion is not normalised")
        qx, qy, qz, qw = (q / norm for q in (qx, qy, qz, qw))
        yaw = math.atan2(2 * (qw * qz + qx * qy), 1 - 2 * (qy * qy + qz * qz))
        cov = tuple(float(row[k]) for k in ("cxx", "cxy", "cyx", "cyy", "cyaw"))
        cxx, cxy, cyx, cyy, cyaw = cov
        tolerance = 1e-9 + 1e-6 * max(cxx, cyy, abs(cxy), abs(cyx))
        if min(cxx, cyy, cyaw) < 0 or abs(cxy - cyx) > tolerance:
            raise ValueError("invalid position/heading covariance")
        if cxx * cyy - cxy * cyx < -tolerance:
            raise ValueError("position covariance is not positive semidefinite")
        return Sample(stamp, (float(row["x"]), float(row["y"]), yaw), cov)
    except (KeyError, TypeError, OverflowError) as exc:
        raise ValueError("incomplete or malformed pose row") from exc


class Series:
    def __init__(self, samples, label):
        self.samples = list(samples)
        self.stamps = [sample.stamp_ns for sample in self.samples]
        self.label = label
        if not self.stamps or any(t <= 0 for t in self.stamps):
            raise ValueError(f"{label}: empty stream or invalid timestamps")
        if any(b <= a for a, b in zip(self.stamps, self.stamps[1:])):
            raise ValueError(f"{label}: duplicate or backwards timestamps; was Gazebo reset?")
        if any(not all(math.isfinite(v) for v in sample.pose) for sample in self.samples):
            raise ValueError(f"{label}: non-finite pose")

    def at(self, stamp, max_gap_ns=MAX_GAP_NS):
        """Exact pose, or short-gap interpolation; never extrapolate."""
        i = bisect_left(self.stamps, stamp)
        if i < len(self.stamps) and self.stamps[i] == stamp:
            return self.samples[i].pose
        if i == 0 or i == len(self.stamps):
            return None
        a, b = self.samples[i - 1], self.samples[i]
        gap = b.stamp_ns - a.stamp_ns
        if gap > max_gap_ns:
            return None
        f = (stamp - a.stamp_ns) / gap
        return (
            a.pose[0] + f * (b.pose[0] - a.pose[0]),
            a.pose[1] + f * (b.pose[1] - a.pose[1]),
            wrap(a.pose[2] + f * wrap(b.pose[2] - a.pose[2])),
        )


def read_series(path, frame, children):
    with path.open(newline="", encoding="utf-8") as handle:
        reader = csv.DictReader(handle)
        if tuple(reader.fieldnames or ()) != CSV_FIELDS:
            raise ValueError(f"{path.name}: unexpected CSV columns")
        samples = []
        for line, row in enumerate(reader, start=2):
            try:
                samples.append(parse_row(row, frame, children))
            except ValueError as exc:
                raise ValueError(f"{path.name}:{line}: {exc}") from exc
    return Series(samples, path.name)


def fit_alignment(pairs):
    """Estimate world -> estimate SE(2) from stationary pose correspondences."""
    if not pairs:
        raise ValueError("No fresh estimator pose in the stationary calibration interval")
    s = sum(math.sin(e[2] - g[2]) for g, e in pairs)
    c = sum(math.cos(e[2] - g[2]) for g, e in pairs)
    if math.hypot(s, c) / len(pairs) < 0.9:
        raise ValueError(
            "Calibration headings disagree; initialise the localiser before recording"
        )
    angle = math.atan2(s, c)
    c, s = math.cos(angle), math.sin(angle)
    tx = sum(e[0] - c * g[0] + s * g[1] for g, e in pairs) / len(pairs)
    ty = sum(e[1] - s * g[0] - c * g[1] for g, e in pairs) / len(pairs)
    return angle, tx, ty


def aligned_truth(pose, alignment):
    angle, tx, ty = alignment
    c, s = math.cos(angle), math.sin(angle)
    return (c * pose[0] - s * pose[1] + tx, s * pose[0] + c * pose[1] + ty, wrap(pose[2] + angle))


def pose_metrics(pairs, alignment):
    position, heading = [], []
    for ground, estimate in pairs:
        ground = aligned_truth(ground, alignment)
        position.append(100 * math.hypot(estimate[0] - ground[0], estimate[1] - ground[1]))
        heading.append(math.degrees(abs(wrap(estimate[2] - ground[2]))))
    if not position:
        raise ValueError("No held-out poses to evaluate")
    return {
        "position_rmse_cm": math.sqrt(sum(x * x for x in position) / len(position)),
        "position_p95_cm": sorted(position)[math.ceil(0.95 * len(position)) - 1],
        "position_max_cm": max(position),
        "yaw_rmse_deg": math.sqrt(sum(x * x for x in heading) / len(heading)),
        "yaw_mean_absolute_deg": sum(heading) / len(heading),
    }


def motion_in_window(truth, start, end):
    """Maximum displacement/yaw from the start, using every truth sample."""
    poses = [sample.pose for sample in truth.samples if start <= sample.stamp_ns <= end]
    first, last = truth.at(start), truth.at(end)
    if first is None or last is None:
        raise ValueError("Missing ground truth at a coverage-window boundary")
    poses = [first, *poses, last]
    displacement = max(math.hypot(p[0] - first[0], p[1] - first[1]) for p in poses)
    rotation = max(abs(wrap(p[2] - first[2])) for p in poses)
    return displacement, rotation


def stationary_window(truth, start, end):
    count = sum(start <= sample.stamp_ns <= end for sample in truth.samples)
    if count < 8:
        raise ValueError("Missing ground truth across the initial eight stationary seconds")
    displacement, rotation = motion_in_window(truth, start, end)
    if displacement > 0.02 or rotation > math.radians(2):
        raise ValueError(
            "Robot moved during calibration (>2 cm or >2 degrees); record a fresh run"
        )
    return {"max_translation_m": displacement, "max_rotation_deg": math.degrees(rotation)}


def provenance(metadata):
    """A startup parameter snapshot is evidence, not proof of independence."""
    snapshot = metadata.get("mapper_snapshot", {})
    parameters = snapshot.get("parameters", {}) if isinstance(snapshot, dict) else {}
    prior = parameters.get("Optimizer/PriorsIgnored")
    ignored = str(prior).lower() == "true" if prior is not None else None
    if metadata.get("assisted") or ignored is False:
        status = "assisted_or_prior_inputs_enabled"
        note = (
            "This run is labelled assisted or RTAB prior inputs were enabled. "
            "Truth-assisted error is not independent validation of LiDAR/odometry-only SLAM."
        )
    elif snapshot.get("status") == "captured" and ignored is True:
        status = "priors_ignored_at_capture_start"
        note = (
            "RTAB global priors were ignored in the startup snapshot; this snapshot "
            "does not establish the mapper's earlier history or later configuration."
        )
    else:
        status = "unverified"
        note = "Mapper assistance/configuration is unverified; PASS certifies data checks only."
    return {"status": status, "note": note, "mapper_snapshot": snapshot}


def analyse(metadata, truth, odometry, estimator):
    if metadata.get("schema_version") != 1 or metadata.get("status") != "complete":
        raise ValueError("Recording is incomplete or invalid; no RMSE will be reported")
    start = metadata.get("start_ns")
    cal_end, stop, end = (metadata.get(key) for key in ("calibration_end_ns", "stop_ns", "end_ns"))
    if not all(isinstance(t, int) for t in (start, cal_end, stop, end)):
        raise ValueError("Missing recording time window")
    if cal_end - start != 8 * NS or end - stop != 5 * NS or stop - cal_end < 17 * NS:
        raise ValueError("Unexpected recording/calibration time windows")
    if truth.stamps[0] > start or truth.stamps[-1] < end:
        raise ValueError("Ground truth does not cover the complete recording")
    relevant_truth = [s for s in truth.samples if start <= s.stamp_ns <= end]
    if any(
        b.stamp_ns - a.stamp_ns > MAX_GAP_NS for a, b in zip(relevant_truth, relevant_truth[1:])
    ):
        raise ValueError("Ground-truth gaps exceed 0.15 s; do not interpolate across missing data")
    stationary = stationary_window(truth, start, cal_end)
    calibration, heldout, candidates = [], [], 0
    for sample in estimator.samples:
        if not start <= sample.stamp_ns <= stop:
            continue
        is_calibration = sample.stamp_ns < cal_end
        if not is_calibration:
            candidates += 1
        ground, odom = truth.at(sample.stamp_ns), odometry.at(sample.stamp_ns)
        if ground is None or odom is None:
            continue
        item = (sample, ground, odom)
        (calibration if is_calibration else heldout).append(item)
    if not calibration:
        raise ValueError(
            "No fresh common stationary pose; initialise the estimator while recording waits"
        )
    if len(heldout) < 20 or candidates == 0 or len(heldout) / candidates < 0.95:
        raise ValueError("Insufficient matched held-out poses (need 20 and at least 95% coverage)")
    # AMCL is motion-gated: a long silence while the robot is demonstrably still
    # is normal. Never excuse a gap using endpoint equality alone: a robot can
    # leave a location and return, so inspect the full high-rate truth interval.
    long_gaps = []
    if heldout[0][0].stamp_ns > cal_end + 5 * NS:
        long_gaps.append((cal_end, heldout[0][0].stamp_ns))
    if heldout[-1][0].stamp_ns < stop - 5 * NS:
        long_gaps.append((heldout[-1][0].stamp_ns, stop))
    long_gaps.extend(
        (a[0].stamp_ns, b[0].stamp_ns)
        for a, b in zip(heldout, heldout[1:])
        if b[0].stamp_ns - a[0].stamp_ns > 10 * NS
    )
    for begin, finish in long_gaps:
        displacement, rotation = motion_in_window(truth, begin, finish)
        if displacement > 0.02 or rotation > math.radians(2):
            raise ValueError(
                "Estimator poses do not span moving parts of the journey; check update settings"
            )
    journey = [s.pose for s in truth.samples if cal_end <= s.stamp_ns <= stop]
    distance = sum(math.hypot(b[0] - a[0], b[1] - a[1]) for a, b in zip(journey, journey[1:]))
    if distance < 0.25:
        raise ValueError("Too little travel: drive at least 0.25 m during the measured journey")
    estimate_alignment = fit_alignment([(g, sample.pose) for sample, g, _ in calibration])
    odom_alignment = fit_alignment([(g, o) for _, g, o in calibration])
    scales = [math.sqrt(s.covariance[0] + s.covariance[3]) for s, _, _ in heldout]
    warnings = []
    if len(calibration) == 1:
        warnings.append(
            "Only one fresh calibration pose; alignment uses that pose's position and yaw"
        )
    if len(heldout) != candidates:
        warnings.append(
            f"{candidates-len(heldout)} estimator poses omitted: no short-gap truth/odom match"
        )
    if long_gaps:
        warnings.append(
            f"{len(long_gaps)} long estimator gap(s) accepted only because truth confirms the robot stayed still"
        )
    source = provenance(metadata)
    warnings.append(source["note"])
    return {
        "status": "PASS",
        "checks_passed": "recording, coverage and alignment checks only; not estimator independence or performance",
        "method": "initial-pose-aligned trajectory error, not absolute map accuracy",
        "estimator": metadata["estimator"],
        "label": metadata.get("label", ""),
        "calibration_samples": len(calibration),
        "calibration_stationarity": stationary,
        "heldout_samples": len(heldout),
        "heldout_candidates": candidates,
        "paired_coverage": len(heldout) / candidates,
        "distance_travelled_m": distance,
        "driving_window_s": [(cal_end - start) / NS, (stop - start) / NS],
        "sampling": "native estimator timestamps; truth/odom interpolated only across gaps <=0.15 s",
        "stationary_estimator_gaps": len(long_gaps),
        "provenance": source,
        "alignment_world_to_estimator": estimate_alignment,
        "alignment_world_to_odometry": odom_alignment,
        "odometry": pose_metrics([(g, o) for _, g, o in heldout], odom_alignment),
        "estimator_metrics": pose_metrics(
            [(g, sample.pose) for sample, g, _ in heldout], estimate_alignment
        ),
        "reported_uncertainty_scale_m": {
            "definition": "sqrt(Cxx+Cyy); not actual error or a guaranteed confidence radius",
            "first": scales[0],
            "last": scales[-1],
            "minimum": min(scales),
            "maximum": max(scales),
        },
        "warnings": warnings,
    }


def evaluate_run(run):
    with (run / "metadata.json").open(encoding="utf-8") as handle:
        metadata = json.load(handle)
    name = metadata.get("robot_name", "")
    if not isinstance(name, str) or not re.fullmatch(r"[A-Za-z0-9_]+", name):
        raise ValueError("Invalid robot name in recording metadata")
    if metadata.get("estimator") not in ("rtabmap", "amcl"):
        raise ValueError("Unknown estimator in recording metadata")
    ns = f"tb_{name}"
    truth = read_series(
        run / "ground_truth.csv", "world", {"base_footprint", f"{ns}/base_footprint"}
    )
    odometry = read_series(run / "odometry.csv", f"{ns}/odom", {f"{ns}/base_footprint"})
    estimator = read_series(run / "estimator.csv", "map", {""})
    return analyse(metadata, truth, odometry, estimator)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("run", type=Path, help="Run folder printed by record_poses.py")
    args = parser.parse_args()
    report = evaluate_run(args.run)
    name = f"comparison_{datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%SZ')}_{uuid4().hex[:8]}.json"
    destination = args.run / name
    with destination.open("x", encoding="utf-8") as handle:
        json.dump(report, handle, indent=2, allow_nan=False)
        handle.write("\n")
    print(
        "PASS — recording/alignment checks; initial-pose-aligned trajectory error (not absolute map accuracy)"
    )
    print(
        f"Calibration: {report['calibration_samples']} poses; held-out: {report['heldout_samples']} "
        f"({100*report['paired_coverage']:.1f}% matched); travel: {report['distance_travelled_m']:.2f} m"
    )
    for label, key in (
        ("Odometry", "odometry"),
        (report["estimator"].upper(), "estimator_metrics"),
    ):
        metrics = report[key]
        print(
            f"{label}: position RMSE {metrics['position_rmse_cm']:.2f} cm; "
            f"yaw RMSE {metrics['yaw_rmse_deg']:.2f} deg"
        )
    scale = report["reported_uncertainty_scale_m"]
    print(
        f"Reported sqrt(Cxx+Cyy): {scale['first']:.3f} -> {scale['last']:.3f} m "
        "(uncertainty, not measured error)"
    )
    for warning in report["warnings"]:
        print("NOTE:", warning)
    print("Report:", destination)
    return 0


if __name__ == "__main__":
    try:
        sys.exit(main())
    except (OSError, ValueError, KeyError, TypeError) as exc:
        print("STOP:", exc, file=sys.stderr)
        sys.exit(1)
