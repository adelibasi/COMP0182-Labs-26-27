#!/usr/bin/env python3
"""Show unexplored frontiers on a live ROS 2 occupancy grid.

A frontier is a free map cell next to unknown space. This node suggests
areas to explore; it never sends velocity commands or navigation goals.
"""

from collections import deque
from math import atan2, cos, floor, isfinite, sin, sqrt
from time import monotonic

import rclpy
from geometry_msgs.msg import Point, PointStamped
from nav_msgs.msg import GridCells, OccupancyGrid
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

FREE_MAX = 20  # Occupancy values 0..20 count as known free space.
MIN_FRONTIER_CELLS = 5  # Ignore isolated, tiny frontier patches.


def adjacent4(index, width, height):
    """Indices of the four neighbouring cells; don't wrap across map rows."""
    x, y = index % width, index // width
    if x > 0:
        yield index - 1
    if x + 1 < width:
        yield index + 1
    if y > 0:
        yield index - width
    if y + 1 < height:
        yield index + width


def adjacent8(index, width, height):
    """Indices of the eight surrounding cells for grouping a frontier."""
    x, y = index % width, index // width
    for dy in (-1, 0, 1):
        for dx in (-1, 0, 1):
            if dx == 0 and dy == 0:
                continue
            nx, ny = x + dx, y + dy
            if 0 <= nx < width and 0 <= ny < height:
                yield ny * width + nx


def find_frontiers(data, width, height):
    """Return frontier groups, largest first, from a flat occupancy grid."""
    if width <= 0 or height <= 0 or len(data) != width * height:
        return []

    frontier_cells = {
        index
        for index, value in enumerate(data)
        if 0 <= value <= FREE_MAX and any(data[n] == -1 for n in adjacent4(index, width, height))
    }

    groups = []
    while frontier_cells:
        seed = frontier_cells.pop()
        group = [seed]
        queue = deque([seed])
        while queue:
            for neighbor in adjacent8(queue.popleft(), width, height):
                if neighbor in frontier_cells:
                    frontier_cells.remove(neighbor)
                    queue.append(neighbor)
                    group.append(neighbor)
        if len(group) >= MIN_FRONTIER_CELLS:
            groups.append(group)

    return sorted(groups, key=len, reverse=True)


def origin_yaw(info):
    """Yaw of the occupancy grid's origin in the map frame."""
    q = info.origin.orientation
    return atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def map_cell_position(index, width, info):
    """Centre of a grid cell in map coordinates, including rotated origins."""
    col, row = index % width, index // width
    x = (col + 0.5) * info.resolution
    y = (row + 0.5) * info.resolution
    yaw = origin_yaw(info)
    return (
        info.origin.position.x + x * cos(yaw) - y * sin(yaw),
        info.origin.position.y + x * sin(yaw) + y * cos(yaw),
    )


def map_point_cell(x, y, info):
    """Grid index for a map-frame point; None if outside the known grid."""
    if not (isfinite(x) and isfinite(y)) or info.resolution <= 0:
        return None
    dx = x - info.origin.position.x
    dy = y - info.origin.position.y
    yaw = origin_yaw(info)
    col = floor((dx * cos(yaw) + dy * sin(yaw)) / info.resolution)
    row = floor((-dx * sin(yaw) + dy * cos(yaw)) / info.resolution)
    if 0 <= col < info.width and 0 <= row < info.height:
        return row * info.width + col
    return None


def representative_cell(group, width):
    """Choose a real frontier cell nearest the group's grid-space centre."""
    mx = sum(i % width for i in group) / len(group)
    my = sum(i // width for i in group) / len(group)
    return min(group, key=lambda i: (i % width - mx) ** 2 + (i // width - my) ** 2)


def closest_frontier(groups, width, info, x, y):
    """Closest frontier cell to a clicked map point (straight-line distance)."""
    best = None
    for group in groups:
        for cell in group:
            fx, fy = map_cell_position(cell, width, info)
            distance_sq = (fx - x) ** 2 + (fy - y) ** 2
            if best is None or distance_sq < best[0]:
                best = (distance_sq, fx, fy, len(group))
    if best is None:
        return None
    d2, fx, fy, group_size = best
    return fx, fy, sqrt(d2), group_size


class FrontierExplorer(Node):
    def __init__(self):
        super().__init__("frontier_explorer")
        qos = QoSProfile(depth=1)
        qos.reliability = ReliabilityPolicy.RELIABLE
        qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.create_subscription(OccupancyGrid, "/map", self.on_map, qos)
        self.create_subscription(PointStamped, "/clicked_point", self.on_click, 10)
        self.publisher = self.create_publisher(GridCells, "/frontiers", qos)
        self.suggestion_pub = self.create_publisher(GridCells, "/suggested_frontier", qos)

        self.latest_map = None
        self.latest_groups = []
        self.target = None
        self.target_was_unknown = False
        self.last_report = -10.0
        self.last_suggestion_report = -10.0
        self.get_logger().info(
            "Waiting for /map. Click an unknown area with RViz Publish Point; "
            "no robot movement commands are sent."
        )

    def grid_cells(self, message, scale=1.0):
        display = GridCells()
        display.header = message.header
        display.cell_width = message.info.resolution * scale
        display.cell_height = message.info.resolution * scale
        return display

    def on_map(self, message):
        self.latest_map = message
        width, height = message.info.width, message.info.height
        groups = find_frontiers(message.data, width, height)
        self.latest_groups = groups

        display = self.grid_cells(message)
        for group in groups:
            for index in group:
                x, y = map_cell_position(index, width, message.info)
                display.cells.append(Point(x=x, y=y, z=0.03))
        self.publisher.publish(display)

        now = monotonic()
        if now - self.last_report >= 5.0:
            self.last_report = now
            if not groups:
                self.get_logger().info("No frontier regions found on the current map.")
            else:
                regions = []
                for number, group in enumerate(groups[:3], start=1):
                    cell = representative_cell(group, width)
                    x, y = map_cell_position(cell, width, message.info)
                    regions.append(f"{number}: ({x:.2f}, {y:.2f}) m, {len(group)} cells")
                self.get_logger().info(
                    f"{len(groups)} frontier region(s). Largest: " + "; ".join(regions)
                )

        self.update_suggestion()

    def on_click(self, click):
        if self.latest_map is None:
            self.get_logger().info("Wait for a /map message, then click again.")
            return
        map_frame = self.latest_map.header.frame_id.lstrip("/")
        click_frame = click.header.frame_id.lstrip("/")
        if click_frame != map_frame:
            self.get_logger().warning(
                f'Clicked point is in {click_frame or "an empty frame"}, '
                f"not {map_frame}; set RViz Fixed Frame to map."
            )
            return
        self.target = (click.point.x, click.point.y)
        index = map_point_cell(click.point.x, click.point.y, self.latest_map.info)
        self.target_was_unknown = (
            index is not None
            and index < len(self.latest_map.data)
            and self.latest_map.data[index] == -1
        )
        self.last_suggestion_report = -10.0
        self.update_suggestion(force_report=True)

    def update_suggestion(self, force_report=False):
        """Refresh the highlight whenever the map or clicked point changes."""
        if self.latest_map is None:
            return
        message = self.latest_map
        display = self.grid_cells(message, scale=3.0)

        if self.target is None:
            self.suggestion_pub.publish(display)
            return

        x, y = self.target
        index = map_point_cell(x, y, message.info)
        if index is None:
            self.get_logger().info(
                "Clicked point is outside the map grid; click inside the grey map area."
            )
            self.target = None
            self.target_was_unknown = False
        elif index >= len(message.data):
            self.get_logger().warning("Map data is incomplete; wait for another map update.")
        elif message.data[index] != -1:
            value = message.data[index]
            if self.target_was_unknown:
                self.get_logger().info(
                    f"Selected area ({x:.2f}, {y:.2f}) m is now observed "
                    f"(occupancy={value}); suggestion cleared."
                )
            elif 0 <= value <= FREE_MAX:
                self.get_logger().info(
                    "Selected area is already mapped as free. Choose an unknown grey cell."
                )
            else:
                self.get_logger().info(
                    "Selected area is already mapped or occupied. Choose an unknown grey cell."
                )
            self.target = None
            self.target_was_unknown = False
        else:
            suggestion = closest_frontier(
                self.latest_groups, message.info.width, message.info, x, y
            )
            if suggestion is not None:
                fx, fy, distance, group_size = suggestion
                display.cells.append(Point(x=fx, y=fy, z=0.07))
                now = monotonic()
                if force_report or now - self.last_suggestion_report >= 5.0:
                    self.last_suggestion_report = now
                    self.get_logger().info(
                        f"Unknown point ({x:.2f}, {y:.2f}) m; "
                        f"suggested frontier ({fx:.2f}, {fy:.2f}) m "
                        f"({distance:.2f} m straight-line, region {group_size} cells). "
                        "Check obstacles; drive manually."
                    )
            elif force_report:
                self.get_logger().info("No frontier region available near the selected area.")

        # Publishing empty GridCells clears any previous suggestion in RViz.
        self.suggestion_pub.publish(display)


def main():
    rclpy.init()
    node = FrontierExplorer()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        # ROS 2 Humble may already have shut down the context on Ctrl+C.
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
