"""Real OpenCV image checks plus clock semantics; no ROS/Gazebo claim."""

import json
from pathlib import Path
import sys
import unittest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "scripts"))
from vision_core import ArucoReader, ObservationTracker

try:
    import cv2
    import numpy as np
except ImportError:
    cv2 = np = None


TASK = {"marker_dictionary": "DICT_4X4_50",
        "marker_id_to_fruit": {7: "banana", 23: "apple", 42: "strawberry"}}


@unittest.skipUnless(cv2 is not None and hasattr(cv2, "aruco"),
                     "OpenCV ArUco is required for real image tests")
class MarkerImageTests(unittest.TestCase):
    def setUp(self):
        self.reader = ArucoReader(TASK)

    def image(self, marker_ids, side=100):
        image = np.full((480, 640, 3), 255, dtype=np.uint8)
        for index, marker_id in enumerate(marker_ids):
            if hasattr(cv2.aruco, "drawMarker"):
                marker = cv2.aruco.drawMarker(self.reader.dictionary, marker_id, side)
            else:
                marker = cv2.aruco.generateImageMarker(self.reader.dictionary, marker_id, side)
            x, y = 35 + index * 180, 100
            image[y:y + side, x:x + side] = cv2.cvtColor(marker, cv2.COLOR_GRAY2BGR)
        return image

    def test_supplied_practice_ids_decode_to_configured_labels(self):
        # Order in the picture differs from sorted JSON order.
        ids, labels, corners = self.reader.detect(self.image([42, 7, 23]))
        self.assertEqual(ids, [7, 23, 42])
        self.assertEqual(labels, ["banana", "apple", "strawberry"])
        self.assertEqual(len(corners), 3)

    def test_unknown_marker_is_reported_without_inventing_a_fruit(self):
        ids, labels, _ = self.reader.detect(self.image([13]))
        self.assertEqual(ids, [13])
        self.assertEqual(labels, [None])
        self.assertEqual(json.loads(json.dumps(labels)), [None])

    def test_small_marker_decodes_in_an_ideal_image(self):
        # About 40 pixels across at the practice observation distance. This
        # checks dictionary decoding, not Gazebo lighting or optical quality.
        ids, labels, _ = self.reader.detect(self.image([23], side=40))
        self.assertEqual((ids, labels), ([23], ["apple"]))

    def test_blank_image_does_not_reuse_the_previous_detection(self):
        self.assertEqual(self.reader.detect(self.image([7]))[0], [7])
        self.assertEqual(self.reader.detect(self.image([])), ([], [], []))

    def test_annotation_preserves_the_input_image(self):
        image = self.image([7])
        before = image.copy()
        ids, labels, corners = self.reader.detect(image)
        annotated = self.reader.annotate(image, ids, labels, corners)
        self.assertTrue(np.array_equal(image, before))
        self.assertFalse(np.array_equal(image, annotated))


class ObservationFreshnessTests(unittest.TestCase):
    def setUp(self):
        self.tracker = ObservationTracker()

    def record(self, **changes):
        values = dict(source_stamp_ns=10_000_000_000,
                      frame_id="tb_test/camera_optical_frame", width=640, height=480,
                      ids=[7], labels=["banana"], now_mono=100.0, now_ns=10_100_000_000)
        values.update(changes)
        return self.tracker.update(**values)

    def test_new_blank_image_clears_the_previous_id_immediately(self):
        self.record()
        self.record(source_stamp_ns=10_200_000_000, now_ns=10_300_000_000,
                    now_mono=100.2, ids=[], labels=[])
        observation = self.tracker.snapshot(100.2, 10_300_000_000)
        self.assertEqual(observation["status"], "NO_MARKER")
        self.assertEqual(observation["ids"], [])
        self.assertEqual(observation["source_stamp_ns"], 10_200_000_000)

    def test_paused_simulation_does_not_pause_the_receipt_timeout(self):
        self.record()
        observation = self.tracker.snapshot(102.01, 10_100_000_000)
        self.assertEqual(observation["status"], "STALE")
        self.assertEqual((observation["ids"], observation["labels"]), ([], []))
        self.assertEqual(observation["source_age_s"], 0.1)
        self.assertEqual(observation["source_stamp_ns"], 10_000_000_000)

    def test_duplicate_or_older_image_stamps_do_not_extend_freshness(self):
        self.record()
        self.assertFalse(self.record(now_mono=101.5, now_ns=10_200_000_000))
        self.assertFalse(self.record(source_stamp_ns=9_900_000_000,
                                     now_mono=101.9, now_ns=10_300_000_000))
        observation = self.tracker.snapshot(102.1, 10_300_000_000)
        self.assertEqual(observation["status"], "STALE")
        self.assertEqual(observation["received_count"], 3)
        self.assertEqual(observation["received_age_s"], 2.1)

    def test_old_or_future_source_images_are_not_usable(self):
        self.assertFalse(self.record(source_stamp_ns=1_000_000_000))
        self.assertEqual(self.tracker.snapshot(100, 10_100_000_000)["status"], "STALE")
        self.assertFalse(self.record(source_stamp_ns=20_000_000_000))
        observation = self.tracker.snapshot(100, 10_100_000_000)
        self.assertEqual(observation["ids"], [])
        self.assertIn("ahead", observation["reason"])

    def test_clock_reset_clears_previous_run_and_accepts_new_timestamps(self):
        self.record()
        reset = self.tracker.snapshot(100.2, 500_000_000)
        self.assertEqual(reset["status"], "STALE")
        self.assertIsNone(reset["source_stamp_ns"])
        self.assertTrue(self.record(source_stamp_ns=600_000_000, now_ns=700_000_000,
                                    now_mono=100.3, ids=[23], labels=["apple"]))
        observation = self.tracker.snapshot(100.3, 700_000_000)
        self.assertEqual((observation["status"], observation["ids"]), ("LIVE", [23]))

    def test_failed_image_conversion_clears_visible_ids(self):
        self.record()
        self.record(source_stamp_ns=10_200_000_000, now_ns=10_300_000_000,
                    now_mono=100.2, error="image conversion failed")
        observation = self.tracker.snapshot(100.2, 10_300_000_000)
        self.assertEqual(observation["status"], "STALE")
        self.assertEqual(observation["ids"], [])

    def test_zero_stamp_and_missing_frame_id_fail_validation(self):
        self.assertFalse(self.record(source_stamp_ns=0))
        self.assertFalse(self.record(frame_id=""))
        self.assertEqual(self.tracker.snapshot(100, 10_100_000_000)["status"], "STALE")

    def test_json_keeps_unknown_label_and_contains_no_room_or_pose(self):
        self.record(ids=[13], labels=[None])
        observation = self.tracker.snapshot(100.1, 10_200_000_000)
        encoded = json.loads(json.dumps(observation, allow_nan=False))
        self.assertEqual(encoded["labels"], [None])
        self.assertNotIn("station", encoded)
        self.assertNotIn("pose", encoded)
        observation["ids"].append(42)
        self.assertEqual(self.tracker.snapshot(100.1, 10_200_000_000)["ids"], [13])



class ViewerModeTests(unittest.TestCase):
    def test_make_decode_uses_the_lab_camera_launcher(self):
        import subprocess

        recipe = subprocess.check_output(
            ["make", "--dry-run", "--no-print-directory", "decode",
             "ROBOT_NAME=tb_alpha", "ROS_DOMAIN_ID=36"], cwd=ROOT, text=True)
        self.assertIn("./packages/camera_decoder/decode_camera.launch.py", recipe)
        self.assertIn('robot_name:=$ROBOT_NAME', recipe)
        self.assertIn('use_sim_time:=$USE_SIM_TIME', recipe)

    def test_rviz_preset_displays_the_namespaced_image(self):
        from string import Template
        import yaml

        source = (ROOT / "config/static/lab.rviz").read_text()
        config = yaml.safe_load(Template(source).substitute(
            agent_name="tb_alpha", tf_prefix="tb_alpha", fixed_frame="map"))
        manager = config["Visualization Manager"]
        display, = [item for item in manager["Displays"]
                    if item["Class"] == "rviz_default_plugins/Image"]
        self.assertTrue(display["Enabled"])
        self.assertEqual(display["Topic"]["Value"], "/tb_alpha/camera/video_annotated")
        self.assertEqual(display["Topic"]["Reliability Policy"], "Best Effort")
        self.assertNotIn("nav2_rviz_plugins/GoalTool", [tool["Class"] for tool in manager["Tools"]])


class PreflightTests(unittest.TestCase):
    def test_sim_guard_and_turtlebot_smoke_are_separate(self):
        import os
        import subprocess
        from tempfile import TemporaryDirectory

        make = ["make", "--dry-run", "--no-print-directory",
                "-f", str(ROOT / "Makefile"),
                "ROBOT_NAME=tb_alpha", "ROS_DOMAIN_ID=36"]
        env = os.environ.copy()
        env.pop("USE_SIM_TIME", None)
        with TemporaryDirectory() as directory:
            for use_sim_time in (None, "true", "false"):
                with self.subTest(use_sim_time=use_sim_time):
                    mode = [] if use_sim_time is None else [f"USE_SIM_TIME={use_sim_time}"]
                    recipe = subprocess.check_output(
                        make + mode + ["check"], cwd=directory, env=env, text=True)
                    self.assertIn('--use-sim-time "$USE_SIM_TIME"', recipe)
                    self.assertEqual("For SIM, set USE_SIM_TIME" in recipe, use_sim_time != "false")
                    self.assertNotIn("scripts/check_hardware.py", recipe)

            smoke = subprocess.check_output(
                make + ["check-hardware"], cwd=directory, env=env, text=True)
        self.assertIn("scripts/check_hardware.py", smoke)
        self.assertNotIn("scripts/aruco.py", smoke)


if __name__ == "__main__":
    unittest.main()
