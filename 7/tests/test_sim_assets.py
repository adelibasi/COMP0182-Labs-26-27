"""Offline camera/marker contracts; these do not claim live Gazebo rendering."""

import copy
import json
import math
from pathlib import Path
import sys
import unittest
import xml.etree.ElementTree as ET

import cv2
import numpy as np

LAB = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(LAB / "scripts"))
import sim_assets as assets


STOCK_SDF = """<sdf version="1.5"><model name="turtlebot3_burger">
 <link name="base_footprint"/><link name="base_link"><collision name="base">
  <geometry><box><size>0.14 0.14 0.14</size></box></geometry></collision></link>
 <link name="base_scan"><sensor name="laser" type="ray"><always_on>true</always_on>
  <visualize>true</visualize><update_rate>5</update_rate><ray><range><min>0.12</min>
  <max>3.5</max></range></ray><plugin name="laser" filename="libgazebo_ros_ray_sensor.so">
  <ros/><frame_name>base_scan</frame_name></plugin></sensor></link>
 <plugin name="drive" filename="libgazebo_ros_diff_drive.so"><ros/>
  <wheel_separation>0.160</wheel_separation><wheel_diameter>0.066</wheel_diameter>
  <odometry_frame>odom</odometry_frame><robot_base_frame>base_footprint</robot_base_frame>
 </plugin></model></sdf>"""
STOCK_URDF = """<robot name="burger"><link name="base_footprint"/>
 <link name="base_link"/><joint name="base_joint" type="fixed">
 <parent link="base_footprint"/><child link="base_link"/>
 <origin xyz="0 0 0.01" rpy="0 0 0"/></joint></robot>"""


def canonical(element):
    return (element.tag, tuple(sorted(element.attrib.items())), (element.text or "").strip(),
            tuple(canonical(item) for item in element))


def detected(image):
    dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
    create = getattr(cv2.aruco, "DetectorParameters_create", None) or cv2.aruco.DetectorParameters
    _, ids, _ = cv2.aruco.detectMarkers(image, dictionary, parameters=create())
    return [] if ids is None else sorted(int(value) for value in ids.flatten())


class SimAssetTests(unittest.TestCase):
    def test_spawn_xml_has_no_encoding_declaration(self):
        # ROS Humble's spawn_entity.py rejects Unicode XML with a prolog.
        sdf, urdf = assets.prepare_robot(STOCK_SDF, STOCK_URDF, "tb_alpha")
        self.assertTrue(sdf.startswith("<sdf "))
        self.assertTrue(urdf.startswith("<robot "))
        self.assertNotIn("<?xml", sdf)
        self.assertNotIn("<?xml", urdf)
        self.assertEqual(ET.fromstring(sdf).tag, "sdf")
        self.assertEqual(ET.fromstring(urdf).tag, "robot")

    @classmethod
    def setUpClass(cls):
        cls.task = assets.load_task(LAB / "config/static/sim_task.yaml")
        cls.geometry = json.loads((LAB / "config/static/arena.json").read_text())

    def test_camera_topics_frame_and_real_sensor_settings(self):
        sdf, _ = assets.prepare_robot(STOCK_SDF, STOCK_URDF, "tb_alpha")
        root = ET.fromstring(sdf)
        sensor = root.find("model/link[@name='camera_link']/sensor")
        self.assertEqual(sensor.get("type"), "camera")
        self.assertEqual(sensor.findtext("update_rate"), "15")
        self.assertEqual(sensor.findtext("camera/image/width"), "640")
        self.assertEqual(sensor.findtext("camera/image/height"), "480")
        self.assertEqual(sensor.findtext("camera/image/format"), "R8G8B8")
        plugin = sensor.find("plugin")
        self.assertEqual(plugin.get("filename"), "libgazebo_ros_camera.so")
        self.assertEqual(plugin.findtext("ros/namespace"), "/tb_alpha")
        self.assertEqual(plugin.findtext("ros/remapping"), "camera/image_raw:=camera/video_raw")
        self.assertIsNone(plugin.find("ros/parameters"))
        self.assertEqual(plugin.findtext("camera_name"), "camera")
        self.assertEqual(plugin.findtext("frame_name"), "tb_alpha/camera_optical_frame")
        self.assertAlmostEqual(float(sensor.findtext("camera/horizontal_fov")), math.pi / 3)

    def test_h264_camera_routes_ffmpeg_packets_to_video_h264(self):
        import yaml

        sdf, _ = assets.prepare_robot(STOCK_SDF, STOCK_URDF, "tb_alpha", camera_transport="h264")
        plugin = ET.fromstring(sdf).find("model/link[@name='camera_link']/sensor/plugin")
        self.assertEqual(plugin.findtext("ros/remapping"), "camera/image_raw/ffmpeg:=camera/video_h264")
        self.assertEqual(plugin.findtext("frame_name"), "tb_alpha/camera_optical_frame")
        params_path = Path(plugin.findtext("ros/parameters"))
        self.assertEqual(params_path, LAB / "config/static/sim_camera.yaml")
        params = yaml.safe_load(params_path.read_text())["/**"]["ros__parameters"]
        self.assertEqual(params["camera.image_raw.enable_pub_plugins"], ["image_transport/ffmpeg"])
        self.assertEqual(params["camera.image_raw.ffmpeg.max_b_frames"], 0)

    def test_existing_collisions_and_lidar_are_preserved(self):
        sdf, _ = assets.prepare_robot(STOCK_SDF, STOCK_URDF, "tb_alpha")
        before, after = ET.fromstring(STOCK_SDF), ET.fromstring(sdf)
        self.assertEqual([canonical(e) for e in before.findall(".//collision")],
                         [canonical(e) for e in after.findall(".//collision")])
        self.assertEqual(after.findtext(".//sensor[@name='laser']/visualize"), "false")
        self.assertEqual(after.findtext(".//sensor[@name='laser']/always_on"), "true")
        self.assertEqual(after.findtext(".//sensor[@name='laser']/update_rate"), "5")
        self.assertEqual(after.findtext(".//sensor[@name='laser']/ray/range/max"), "3.5")
        self.assertEqual(after.findtext(".//plugin[@name='drive']/wheel_diameter"), "0.066")
        self.assertEqual(after.findtext(".//plugin[@name='drive']/odometry_frame"), "tb_alpha/odom")

    def test_urdf_and_sdf_mount_and_optical_axes_agree(self):
        sdf, urdf = assets.prepare_robot(STOCK_SDF, STOCK_URDF, "tb_alpha")
        sdf, urdf = ET.fromstring(sdf), ET.fromstring(urdf)
        pose = sdf.find("model/link[@name='camera_link']/pose")
        self.assertEqual(pose.get("relative_to"), "base_link")
        sdf_xyz = list(map(float, pose.text.split()[:3]))
        urdf_xyz = list(map(float, urdf.find("joint[@name='camera_joint']/origin").get("xyz").split()))
        np.testing.assert_allclose(sdf_xyz, urdf_xyz, atol=1e-12)
        origin = urdf.find("joint[@name='camera_optical_joint']/origin")
        roll, pitch, yaw = map(float, origin.get("rpy").split())
        rx = np.array([[1, 0, 0], [0, math.cos(roll), -math.sin(roll)], [0, math.sin(roll), math.cos(roll)]])
        rz = np.array([[math.cos(yaw), -math.sin(yaw), 0], [math.sin(yaw), math.cos(yaw), 0], [0, 0, 1]])
        self.assertEqual(pitch, 0)
        np.testing.assert_allclose(rz @ rx, [[0, 0, 1], [-1, 0, 0], [0, -1, 0]], atol=1e-12)
        optical = sdf.find("model/frame[@name='camera_optical_frame']/pose")
        self.assertEqual(sdf.find("model/frame[@name='camera_optical_frame']").get("attached_to"), "camera_link")
        self.assertEqual(optical.get("relative_to"), "camera_link")
        np.testing.assert_allclose(list(map(float, optical.text.split()[3:])), [roll, pitch, yaw])

    def test_model_failures_are_explicit(self):
        for name in ("alpha", "tb_", "tb-with-dash"):
            with self.assertRaises(ValueError):
                assets.prepare_robot(STOCK_SDF, STOCK_URDF, name)
        camera_sdf, camera_urdf = assets.prepare_robot(STOCK_SDF, STOCK_URDF, "tb_alpha")
        with self.assertRaisesRegex(ValueError, "already has camera_link"):
            assets.prepare_robot(camera_sdf, camera_urdf, "tb_alpha")

    def test_textured_models_use_exact_source_pngs_and_ogre_material(self):
        for marker_id in (7, 23, 42):
            name = assets.marker_model_name(marker_id)
            directory = LAB / "models" / name
            image_bytes = (LAB / "markers" / f"aruco_{marker_id}.png").read_bytes()
            self.assertEqual(
                (directory / "materials/textures" / f"aruco_{marker_id}.png").read_bytes(),
                image_bytes)
            image = cv2.imdecode(np.frombuffer(image_bytes, dtype=np.uint8),
                                 cv2.IMREAD_GRAYSCALE)
            self.assertEqual(image.shape, (800, 800))
            self.assertTrue(np.all(image[:100] == 255) and np.all(image[-100:] == 255))
            self.assertEqual(detected(image), [marker_id])
            sdf = ET.fromstring((directory / "model.sdf").read_bytes())
            self.assertEqual(sdf.findall(".//collision"), [])
            self.assertEqual(sdf.findall(".//geometry/box"), [])
            visuals = sdf.findall("model/link/visual")
            self.assertEqual(len(visuals), 1)
            visual = visuals[0]
            self.assertEqual(visual.get("name"), "single_textured_face")
            self.assertEqual(visual.findtext("geometry/plane/normal"), "0 0 1")
            np.testing.assert_allclose(
                list(map(float, visual.findtext("geometry/plane/size").split())),
                [self.task["markers"]["board_size_m"]] * 2)
            self.assertEqual(visual.findtext("cast_shadows"), "false")
            script = visual.find("material/script")
            self.assertEqual([uri.text for uri in script.findall("uri")],
                             [f"model://{name}/materials/scripts",
                              f"model://{name}/materials/textures"])
            self.assertEqual(script.findtext("name"),
                             f"Lab7Textured/ArUco{marker_id}")
            material = (
                directory / "materials/scripts" / f"aruco_{marker_id}.material"
            ).read_text(encoding="utf-8")
            self.assertIn(f"material Lab7Textured/ArUco{marker_id}", material)
            self.assertIn(f"texture aruco_{marker_id}.png", material)
            self.assertIn("filtering none", material)
            self.assertIn("lighting off", material)

    def test_textured_plane_orientation_matches_hp_passed_upright_probe(self):
        # For rear marker yaw=-pi/2, the combined rotation must reproduce
        # the Phase 2/3 sign: plane X right, plane Y up, plane normal facing -Y.
        for marker_id in (7, 23, 42):
            name = assets.marker_model_name(marker_id)
            sdf = (LAB / "models" / name / "model.sdf").read_bytes()
            pose = ET.fromstring(sdf).findtext("model/link/visual/pose")
            x, y, z, roll, pitch, yaw = map(float, pose.split())
            self.assertEqual([x, y, z], [0, 0, 0])
            self.assertAlmostEqual(pitch, 0)
            cr, sr = math.cos(roll), math.sin(roll)
            cy, sy = math.cos(yaw), math.sin(yaw)
            rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])
            rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]])
            r = rz @ rx
            np.testing.assert_allclose(r @ [0, 0, 1], [1, 0, 0], atol=1e-12)
            np.testing.assert_allclose(r @ [1, 0, 0], [0, 1, 0], atol=1e-12)
            np.testing.assert_allclose(r @ [0, 1, 0], [0, 0, 1], atol=1e-12)
            arena_yaw = self.task["markers"]["placements"]["B2"]["facing_yaw_arena_rad"]
            c, s = math.cos(arena_yaw), math.sin(arena_yaw)
            model_rotation = np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])
            expected_probe_rotation = np.array([[1, 0, 0], [0, 0, -1], [0, 1, 0]])
            np.testing.assert_allclose(
                model_rotation @ r, expected_probe_rotation, atol=1e-12)

    def test_runtime_world_preserves_arena_and_places_textured_models(self):
        base = (LAB / "worlds/arena.world").read_text()
        updated = assets.add_markers(base, self.geometry, self.task, LAB / "models")
        before, after = ET.fromstring(base), ET.fromstring(updated)
        for selector in ("world/gui", "world/physics", "world/model[@name='map_walls']"):
            self.assertEqual(canonical(before.find(selector)), canonical(after.find(selector)))
        includes = after.findall("world/include")
        self.assertEqual(len(includes), 3)
        self.assertEqual([m for m in after.findall("world/model")
                          if m.get("name", "").startswith("lab7_marker_")], [])
        for (station, placement), include in zip(
                self.task["markers"]["placements"].items(), includes):
            name = assets.marker_model_name(placement["id"])
            self.assertEqual(include.findtext("uri"), f"model://{name}")
            self.assertEqual(include.findtext("name"), f"lab7_marker_{station}")
            values = list(map(float, include.findtext("pose").split()))
            expected = assets.arena_pose_to_world(
                self.geometry, [*placement["position_arena_m"][:2],
                                placement["facing_yaw_arena_rad"]])
            np.testing.assert_allclose(
                [values[0], values[1], values[2], values[5]],
                [expected[0], expected[1], placement["position_arena_m"][2], expected[2]])
            self.assertEqual(values[3:5], [0, 0])
            # The textured plane's normal points at the labelled observation pose.
            obs = self.task["stations"][station]["observation_pose_arena"]
            dx = obs[0] - placement["position_arena_m"][0]
            dy = obs[1] - placement["position_arena_m"][1]
            expected_dot = (math.cos(placement["facing_yaw_arena_rad"]) * dx +
                            math.sin(placement["facing_yaw_arena_rad"]) * dy)
            self.assertGreater(expected_dot, 0)

    def test_bad_marker_config_is_rejected(self):
        bad = copy.deepcopy(self.task)
        bad["markers"]["placements"]["B2"]["id"] = 7
        with self.assertRaisesRegex(ValueError, "unique"):
            assets.validate_task(bad)


if __name__ == "__main__":
    unittest.main()
