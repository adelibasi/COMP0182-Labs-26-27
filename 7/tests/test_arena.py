"""Geometric/map checks only: no Gazebo, ROS or robot execution is claimed."""

from collections import deque
import json
import math
from pathlib import Path
import sys
import unittest
import xml.etree.ElementTree as ET

import cv2
import yaml

LAB = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(LAB / "scripts"))
from sim_assets import arena_pose_to_world


def parsed_collision_boxes(xml):
    """Read collision boxes from the shipped Gazebo world."""
    model = ET.fromstring(xml).find("world/model[@name='map_walls']")
    tx, ty, _, roll, pitch, yaw = map(float, model.findtext("pose").split())
    if roll or pitch:
        raise AssertionError("Unexpected tilted arena")
    boxes = []
    for link in model.findall("link"):
        x, y, z, r, p, a = map(float, link.findtext("pose").split())
        if r or p or a:
            raise AssertionError("Unexpected locally rotated wall")
        width, depth, height = map(float, link.findtext("collision/geometry/box/size").split())
        cx = tx + math.cos(yaw) * x - math.sin(yaw) * y
        cy = ty + math.sin(yaw) * x + math.cos(yaw) * y
        boxes.append((link.get("name"), cx, cy, width / 2, depth / 2, yaw, z - height / 2, z + height / 2))
    return boxes


def point_in_box(x, y, box):
    _, cx, cy, hx, hy, yaw, _, _ = box
    dx, dy = x - cx, y - cy
    u = math.cos(yaw) * dx + math.sin(yaw) * dy
    v = -math.sin(yaw) * dx + math.cos(yaw) * dy
    return abs(u) <= hx + 1e-9 and abs(v) <= hy + 1e-9


def cell_index(meta, x, y):
    col = math.floor((x - meta["origin"][0]) / meta["resolution"])
    row = math.floor((y - meta["origin"][1]) / meta["resolution"])
    if not (0 <= col < meta["width"] and 0 <= row < meta["height"]):
        raise AssertionError("Probe outside raster")
    return col, row


def pixel(meta, pixels, col, row):
    return pixels[(meta["height"] - 1 - row) * meta["width"] + col]


def flood(meta, pixels, start):
    """Treat unknown as traversable so it cannot conceal a perimeter hole."""
    queue, visited = deque([start]), {start}
    while queue:
        x, y = queue.popleft()
        for nx, ny in ((x - 1, y), (x + 1, y), (x, y - 1), (x, y + 1)):
            if 0 <= nx < meta["width"] and 0 <= ny < meta["height"] and (nx, ny) not in visited:
                if pixel(meta, pixels, nx, ny) != 0:
                    visited.add((nx, ny))
                    queue.append((nx, ny))
    return visited


def reaches_edge(meta, cells):
    return any(x in (0, meta["width"] - 1) or y in (0, meta["height"] - 1) for x, y in cells)


class ArenaGeometryTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.geometry = json.loads((LAB / "config/static/arena.json").read_text())
        map_yaml = LAB / "maps/arena.yaml"
        cls.meta = yaml.safe_load(map_yaml.read_text())
        image_path = map_yaml.parent / cls.meta["image"]
        image = cv2.imread(str(image_path), cv2.IMREAD_GRAYSCALE)
        if image is None:
            raise AssertionError(f"Cannot read arena occupancy map: {image_path}")
        cls.meta["height"], cls.meta["width"] = image.shape
        cls.pixels = image.ravel()
        cls.world = (LAB / "worlds/arena.world").read_bytes()
        cls.boxes = parsed_collision_boxes(cls.world)
        sx, sy, _ = arena_pose_to_world(cls.geometry, cls.geometry["spawn_pose_arena"])
        cls.spawn_cell = cell_index(cls.meta, sx, sy)
        cls.reachable = flood(cls.meta, cls.pixels, cls.spawn_cell)

    def test_shipped_world_has_expected_wall_collisions(self):
        self.assertEqual(len(self.boxes), 13)
        self.assertEqual(ET.fromstring(self.world).findall("world/plugin"), [])
        self.assertTrue(all(abs(b[6]) < 1e-9 and abs(b[7] - 0.3) < 1e-9 for b in self.boxes))

    def test_world_transform_retains_source_alignment(self):
        pose = arena_pose_to_world(self.geometry, self.geometry["spawn_pose_arena"])
        self.assertAlmostEqual(pose[0], -1.1053642608597487, places=10)
        self.assertAlmostEqual(pose[1], 0.19616169579085252, places=10)
        self.assertAlmostEqual(pose[2], 0.7461, places=12)

    def test_continuous_perimeter_has_no_escape_in_rotated_raster(self):
        self.assertEqual(pixel(self.meta, self.pixels, *self.spawn_cell), 254)
        self.assertFalse(reaches_edge(self.meta, self.reachable))
        width, depth = self.geometry["arena_size_m"]
        for i in range(201):
            f = i / 200
            for x, y in ((f * width, 0.15), (f * width, depth - 0.15), (0.15, f * depth), (width - 0.15, f * depth)):
                wx, wy, _ = arena_pose_to_world(self.geometry, (x, y, 0.0))
                self.assertTrue(any(point_in_box(wx, wy, box) for box in self.boxes))

    def test_all_intended_bays_connect_to_the_central_corridor(self):
        # Neutral probes, not approved robot goals or official coursework rooms.
        probes = ((0.8, 0.8), (3.16, 0.9), (5.6, 0.8),
                  (0.8, 5.0), (3.16, 5.0), (5.6, 5.0),
                  (0.6, 2.4), (1.9, 3.5), (4.6, 3.0))
        for local in probes:
            wx, wy, _ = arena_pose_to_world(self.geometry, (*local, 0.0))
            cell = cell_index(self.meta, wx, wy)
            self.assertEqual(pixel(self.meta, self.pixels, *cell), 254, local)
            self.assertIn(cell, self.reachable, local)

    def test_inward_dividers_and_L_are_continuous_but_corridor_is_open(self):
        occupied_probes = ((2.5, 0.8), (2.5, 1.5), (2.5, 4.2), (2.5, 4.9),
                           (3.85, 0.8), (3.85, 4.9), (1.25, 1.9), (1.25, 2.5))
        for x, y in occupied_probes:
            wx, wy, _ = arena_pose_to_world(self.geometry, (x, y, 0.0))
            self.assertTrue(any(point_in_box(wx, wy, box) for box in self.boxes))
            self.assertEqual(pixel(self.meta, self.pixels, *cell_index(self.meta, wx, wy)), 0)
        for x in (1.7, 2.5, 3.2, 3.85, 4.8, 5.7):
            wx, wy, _ = arena_pose_to_world(self.geometry, (x, 3.4, 0.0))
            self.assertEqual(pixel(self.meta, self.pixels, *cell_index(self.meta, wx, wy)), 254)

    def test_map_conservatively_covers_collision_samples_with_correct_row_order(self):
        # compare occupancy against the actual Gazebo collision boxes
        res = self.meta["resolution"]
        ox, oy, _ = self.meta["origin"]
        for row in range(self.meta["height"]):
            for col in range(self.meta["width"]):
                value = pixel(self.meta, self.pixels, col, row)
                cx, cy = ox + (col + 0.5) * res, oy + (row + 0.5) * res
                centre_hits = any(point_in_box(cx, cy, box) for box in self.boxes)
                if centre_hits:
                    self.assertEqual(value, 0, (col, row, "wall centre marked free/unknown"))
                if value != 0:
                    for fx, fy in ((-0.499, -0.499), (-0.499, 0.499), (0.499, -0.499), (0.499, 0.499)):
                        self.assertFalse(any(point_in_box(cx + fx * res, cy + fy * res, b) for b in self.boxes), (col, row, "wall crossing in unoccupied cell"))


if __name__ == "__main__":
    unittest.main()
