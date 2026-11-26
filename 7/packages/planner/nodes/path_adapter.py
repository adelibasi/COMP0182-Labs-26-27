"""Convert start and goal positions to grid cells and check A* routes."""

import math


def cell_is_blocked(costmap, point):
    # nav2 costs 253–255 are impassable, including unknown cells
    column, row = point

    return costmap.data[row * costmap.metadata.size_x + column] >= 253


def poses_to_cells(costmap, start, goal):
    """Convert start and goal positions in metres to (column, row) cells.

    Subtract the costmap origin, divide by metres per cell, then round down.
    """
    meta = costmap.metadata
    width, height, resolution = meta.size_x, meta.size_y, meta.resolution
    if (
        width <= 0
        or height <= 0
        or not math.isfinite(resolution)
        or resolution <= 0
        or len(costmap.data) != width * height
    ):
        raise ValueError("Global costmap is missing or malformed")

    if (
        start.header.frame_id != costmap.header.frame_id
        or goal.header.frame_id != costmap.header.frame_id
    ):
        raise ValueError("Start and goal must be in the global costmap frame")

    def cell(pose):
        x, y = pose.pose.position.x, pose.pose.position.y
        ox, oy = meta.origin.position.x, meta.origin.position.y
        if not all(math.isfinite(value) for value in (x, y, ox, oy)):
            raise ValueError("Non-finite map position")

        point = (math.floor((x - ox) / resolution), math.floor((y - oy) / resolution))
        if not 0 <= point[0] < width or not 0 <= point[1] < height:
            raise ValueError("Start or goal lies outside the global costmap")

        return point

    start_cell, goal_cell = cell(start), cell(goal)
    if cell_is_blocked(costmap, start_cell) or cell_is_blocked(costmap, goal_cell):
        raise ValueError("Start or goal is blocked or unknown")

    return start_cell, goal_cell


def validate_path(costmap, start_cell, goal_cell, path):
    """Check that a route starts and ends correctly and takes valid grid steps."""
    width, height = costmap.metadata.size_x, costmap.metadata.size_y

    if not isinstance(path, list) or not path or len(path) > width * height:
        raise ValueError("A* must return a non-empty list of cells")

    if path[0] != start_cell or path[-1] != goal_cell:
        raise ValueError("A* path must include start and goal cells")

    previous = None
    for point in path:
        if (
            not isinstance(point, tuple)
            or len(point) != 2
            or any(type(value) is not int for value in point)
            or not 0 <= point[0] < width
            or not 0 <= point[1] < height
            or cell_is_blocked(costmap, point)
        ):
            raise ValueError("A* path contains a malformed, off-map or blocked cell")

        if previous is not None:
            dx, dy = point[0] - previous[0], point[1] - previous[1]
            if max(abs(dx), abs(dy)) != 1:
                raise ValueError("A* path is discontinuous")

            # keep diagonal paths clear of adjacent obstacles
            if (
                dx
                and dy
                and (
                    cell_is_blocked(costmap, (previous[0] + dx, previous[1]))
                    or cell_is_blocked(costmap, (previous[0], previous[1] + dy))
                )
            ):
                raise ValueError("A* path cuts an obstacle corner")

        previous = point
