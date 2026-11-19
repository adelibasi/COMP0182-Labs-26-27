"""Checks that also work on a machine without ROS installed."""

import importlib.util
import os
import re
import shutil
import subprocess
import tempfile
import unittest
from pathlib import Path
from types import SimpleNamespace as Record


class Lab6Tests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        source = Path(__file__).resolve().parents[1]
        cls.temp = tempfile.TemporaryDirectory()
        cls.lab = Path(cls.temp.name) / "lab_6"
        shutil.copytree(
            source,
            cls.lab,
            ignore=shutil.ignore_patterns(
                "build",
                "install",
                "log",
                "__pycache__",
                "*.answer*",
                "*.draft*",
                "m-explore-ros2",
            ),
        )
        spec = importlib.util.spec_from_file_location(
            "path_adapter", cls.lab / "scripts/path_adapter.py"
        )
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)
        cls.poses_to_cells = staticmethod(module.poses_to_cells)
        cls.validate_path = staticmethod(module.validate_path)

    @classmethod
    def tearDownClass(cls):
        cls.temp.cleanup()

    def fixture(self, blocked=()):
        data = [0] * 9
        for column, row in blocked:
            data[row * 3 + column] = 254
        map_ = Record(
            header=Record(frame_id="map"),
            metadata=Record(
                size_x=3,
                size_y=3,
                resolution=1.0,
                origin=Record(position=Record(x=0.0, y=0.0)),
            ),
            data=data,
        )

        def pose(x, y):
            return Record(
                header=Record(frame_id="map"), pose=Record(position=Record(x=x, y=y))
            )

        return map_, pose(0.5, 0.5), pose(2.5, 0.5)

    def test_valid_route_and_rejected_routes(self):
        map_, start, goal = self.fixture()
        route = [(0, 0), (1, 0), (2, 0)]
        start_cell, goal_cell = self.poses_to_cells(map_, start, goal)
        self.validate_path(map_, start_cell, goal_cell, route)
        for invalid in ([], [(0, 0), (2, 0)], [(0, 0), (1, 0)], [(0, 0), (3, 0)]):
            with self.subTest(invalid=invalid), self.assertRaises(ValueError):
                self.validate_path(map_, start_cell, goal_cell, invalid)
        map_, start, goal = self.fixture({(1, 0)})
        with self.assertRaises(ValueError):
            self.validate_path(map_, start_cell, goal_cell, route)

    def test_rejects_malformed_costmap(self):
        map_, start, goal = self.fixture()
        map_.data = [0, 0]
        with self.assertRaises(ValueError):
            self.poses_to_cells(map_, start, goal)

    def test_rejects_invalid_endpoints(self):
        for endpoint in ("start", "goal"):
            for axis in ("x", "y"):
                for value in (-0.5, 3.0, float("nan"), float("inf")):
                    map_, start, goal = self.fixture()
                    pose = start if endpoint == "start" else goal
                    setattr(pose.pose.position, axis, value)
                    with self.subTest(endpoint=endpoint, axis=axis, value=value):
                        with self.assertRaises(ValueError):
                            self.poses_to_cells(map_, start, goal)

            map_, start, goal = self.fixture()
            pose = start if endpoint == "start" else goal
            pose.header.frame_id = "odom"
            with self.subTest(endpoint=endpoint, frame="odom"):
                with self.assertRaises(ValueError):
                    self.poses_to_cells(map_, start, goal)

            for cost in (253, 254, 255):
                map_, start, goal = self.fixture()
                map_.data[0 if endpoint == "start" else 2] = cost
                with self.subTest(endpoint=endpoint, cost=cost):
                    with self.assertRaises(ValueError):
                        self.poses_to_cells(map_, start, goal)

    def test_extracted_package_has_map_and_planner_bridge(self):
        map_yaml = (self.lab / "maps/house.yaml").read_text()
        self.assertIn("image: house.pgm", map_yaml)
        self.assertTrue((self.lab / "maps/house.pgm").is_file())
        nav = (self.lab / "config/static/nav2.yaml").read_text()
        self.assertIn("planner_plugins: [Student]", nav)
        self.assertNotIn("NavfnPlanner", nav)
        self.assertTrue((self.lab / "exercises/planner.py").is_file())
        self.assertFalse((self.lab / "exercises/planner.answer.py").exists())
        self.assertFalse((self.lab / "ros2_ws/src/m-explore-ros2").exists())
        for source in (
            "packages/lab6_planner/CMakeLists.txt",
            "packages/lab6_planner/package.xml",
            "packages/lab6_planner/plugin.xml",
            "packages/lab6_planner/src/student_planner.cpp",
            "packages/lab6_planner_interfaces/CMakeLists.txt",
            "packages/lab6_planner_interfaces/package.xml",
            "packages/lab6_planner_interfaces/srv/PlanPath.srv",
        ):
            self.assertTrue((self.lab / source).is_file(), source)
        self.assertIn(
            "TODO_MAP_TOPIC",
            (self.lab / "exercises/explore_starter.launch.py").read_text(),
        )

    def test_make_env_fallback_and_robot_guard(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            shutil.copyfile(self.lab / "Makefile", root / "Makefile")
            env = os.environ.copy()
            for name in ("ROBOT_NAME", "ROS_DOMAIN_ID", "ROS_LOCALHOST_ONLY", "USE_SIM_TIME", "GUI", "MAP", "TURTLEBOT3_MODEL"):
                env.pop(name, None)
            env.pop("MAKEFLAGS", None)
            env.pop("MAKEOVERRIDES", None)

            def check(*args):
                return subprocess.run(
                    ["make", "--no-print-directory", "require-robot", *args],
                    cwd=root, env=env, capture_output=True, text=True,
                )

            def heading(*args):
                result = subprocess.run(
                    ["make", "--no-print-directory", "mast-head", *args],
                    cwd=root, env=env, capture_output=True, text=True,
                )
                self.assertEqual(result.returncode, 0, result.stderr)
                return re.sub(r"\x1b\[[0-9;]*m", "", result.stdout)

            self.assertIn("Set ROBOT_NAME", check().stderr)
            self.assertIn("ROBOT_NAME: <unset> (source: unset)", heading())
            for invalid in ("ada", "tb_", "tb_ada/scan"):
                env["ROBOT_NAME"] = invalid
                self.assertIn("tb_<NAME>", check().stderr)
            env.pop("ROBOT_NAME")
            self.assertIn("USE_SIM_TIME: true (source: Makefile default)", heading())
            self.assertIn("TURTLEBOT3_MODEL: burger (source: Makefile default)", heading())
            env["TURTLEBOT3_MODEL"] = "waffle"
            self.assertIn("TURTLEBOT3_MODEL: waffle (source: shell)", heading())
            self.assertIn("TURTLEBOT3_MODEL: pancake (source: command line)", heading("TURTLEBOT3_MODEL=pancake"))
            env.pop("TURTLEBOT3_MODEL")
            env["USE_SIM_TIME"] = ""
            self.assertIn("USE_SIM_TIME: <unset> (source: shell)", heading())
            env.pop("USE_SIM_TIME")
            env["ROBOT_NAME"] = "tb_from_session"
            env["TURTLEBOT3_MODEL"] = "pancake"
            (root / ".env").write_text("ROBOT_NAME=tb_from_file\nROS_DOMAIN_ID=42\nTURTLEBOT3_MODEL=waffle\n")
            self.assertEqual(check().returncode, 0)
            self.assertIn("ROBOT_NAME: tb_from_file (source: .env)", heading())
            self.assertIn("ROS_DOMAIN_ID: 42 (source: .env)", heading())
            self.assertIn("TURTLEBOT3_MODEL: waffle (source: .env)", heading())
            self.assertIn("ROBOT_NAME: tb_from_cli (source: command line)", heading("ROBOT_NAME=tb_from_cli"))
            (root / ".env").write_text("ROBOT_NAME=ada\n")
            self.assertIn("tb_<NAME>", check().stderr)
            self.assertEqual(check("ROBOT_NAME=tb_from_cli").returncode, 0)
            (root / ".env").write_text("ROBOT_NAME=\n")
            self.assertIn("Set ROBOT_NAME", check().stderr)
            self.assertIn("ROBOT_NAME: <unset> (source: .env)", heading())
            (root / ".env").unlink()
            self.assertEqual(check().returncode, 0)
            self.assertIn("ROBOT_NAME: tb_from_session (source: shell)", heading())

            command = ["make", "--no-print-directory", "-n", "nav"]
            (root / ".env").write_text("ROBOT_NAME=tb_ada\n")
            default = subprocess.run(command, cwd=root, env=env, capture_output=True, text=True)
            self.assertEqual(default.returncode, 0, default.stderr)
            self.assertNotIn("scripts/check_hardware.py", default.stdout)
            (root / ".env").write_text("ROBOT_NAME=tb_ada\nUSE_SIM_TIME=false\n")
            physical = subprocess.run(command, cwd=root, env=env, capture_output=True, text=True)
            self.assertEqual(physical.returncode, 0, physical.stderr)
            self.assertIn("scripts/check_hardware.py", physical.stdout)
            env["USE_SIM_TIME"] = "true"
            simulation = subprocess.run(command, cwd=root, env=env, capture_output=True, text=True)
            self.assertEqual(simulation.returncode, 0, simulation.stderr)
            self.assertIn("scripts/check_hardware.py", simulation.stdout)
            (root / ".env").write_text("ROBOT_NAME=tb_ada\n")
            simulation = subprocess.run(command, cwd=root, env=env, capture_output=True, text=True)
            self.assertEqual(simulation.returncode, 0, simulation.stderr)
            self.assertNotIn("scripts/check_hardware.py", simulation.stdout)


if __name__ == "__main__":
    unittest.main()
