"""Try your A* on small maps before driving a robot.

Each picture below is a map. Read it as it appears on the page:

    S  start                 G  goal
    .  free cell            #  wall
    ?  unknown cell         I  inflated obstacle boundary
    ~  costly but passable cell
"""

import unittest
from collections import deque
from textwrap import dedent
from types import SimpleNamespace

from path_adapter import poses_to_cells, validate_path
from planner import a_star_impl


class AStarTests(unittest.TestCase):
    def test_crosses_an_open_hallway(self):
        """An open hallway needs a direct route from left to right."""
        picture = """
            S . . . G
        """
        self.check_route(picture)

    def test_can_travel_from_right_to_left(self):
        """The planner must also work when the goal is to the left."""
        picture = """
            G . . . S
        """
        self.check_route(picture)

    def test_can_travel_up(self):
        """A one-cell-wide map leaves only an upward route."""
        picture = """
            G
            .
            .
            S
        """
        self.check_route(picture)

    def test_can_travel_down(self):
        """The same narrow map must also work in the other direction."""
        picture = """
            S
            .
            .
            G
        """
        self.check_route(picture)

    def test_goes_around_a_wall(self):
        """The direct route is blocked, but the outer edge is open."""
        picture = """
            S . . .
            . # # .
            . # # .
            . . . G
        """
        self.check_route(picture)

    def test_finds_the_gap_at_the_bottom(self):
        """The robot has to go down to pass the wall."""
        picture = """
            S . # . G
            . . # . .
            . . . . .
        """
        self.check_route(picture)

    def test_goes_around_unknown_space(self):
        """An unknown cell is not a shortcut through the map."""
        picture = """
            S ? G
            . . .
        """
        self.check_route(picture)

    def test_goes_around_an_inflated_obstacle(self):
        """The safety boundary around an obstacle is also blocked."""
        picture = """
            S I G
            . . .
        """
        self.check_route(picture)

    def test_can_cross_costly_but_passable_cells(self):
        """A high cost is different from a wall when it is the only route."""
        picture = """
            # # # # #
            S ~ ~ ~ G
            # # # # #
        """
        self.check_route(picture)

    def test_prefers_a_free_route_to_an_equally_long_costly_one(self):
        """Two corridors go round the wall; the lower one has no high costs."""
        picture = """
            . ~ ~ ~ .
            S # # # G
            . . . . .
        """
        grid, route = self.check_route(picture)
        for column, row in route:
            cost = grid.data[row * grid.metadata.size_x + column]
            self.assertLess(cost, 200, "The free lower corridor should be preferred")

    def test_stays_on_the_map_at_its_edge(self):
        """The route follows the left and bottom edges without leaving the map."""
        picture = """
            S # . .
            . # . .
            . # . .
            . . . G
        """
        self.check_route(picture)

    def test_finds_a_route_through_several_turns(self):
        """A larger map requires turns and cannot be solved by going straight."""
        picture = """
            S . # . . . G
            # . # . # # .
            # . . . # . .
            # # # . # . #
            . . . . . . .
        """
        self.check_route(picture)

    def test_returns_no_route_when_a_wall_separates_start_and_goal(self):
        """A complete wall means there is no path to return."""
        picture = """
            S # G
            # # #
            . . .
        """
        grid, start, goal = map_from_picture(picture)
        self.assertEqual(
            a_star_impl(nav2_cost_map=grid, start_point=start, goal_point=goal), []
        )

    def test_does_not_squeeze_diagonally_between_walls(self):
        """The two free cells touch only at a blocked corner."""
        picture = """
            S #
            # G
        """
        grid, start, goal = map_from_picture(picture)
        self.assertEqual(
            a_star_impl(nav2_cost_map=grid, start_point=start, goal_point=goal), []
        )

    def test_start_already_at_the_goal(self):
        """A one-cell route still includes its start and goal cell."""
        picture = """
            S
        """
        grid, start, _ = map_from_picture(picture)
        self.assertEqual(
            a_star_impl(nav2_cost_map=grid, start_point=start, goal_point=start),
            [start],
        )

    def check_route(self, picture):
        """Check every step and reject a route longer than a simple detour."""
        grid, start, goal = map_from_picture(picture)
        start_cell, goal_cell = poses_to_cells(grid, pose_at(start), pose_at(goal))
        route = a_star_impl(
            nav2_cost_map=grid, start_point=start_cell, goal_point=goal_cell
        )
        validate_path(grid, start_cell, goal_cell, route)
        fewest_steps = shortest_four_way_route(grid, start, goal)
        self.assertIsNotNone(fewest_steps, "The test map should have a route")
        self.assertLessEqual(len(route) - 1, fewest_steps)
        return grid, route


CELL_COST = {"S": 0, "G": 0, ".": 0, "~": 200, "I": 253, "#": 254, "?": 255}


def map_from_picture(picture):
    """Build the small costmap shown in a test; ROS is not needed here."""
    rows = [line.split() for line in dedent(picture).strip().splitlines()]
    width = len(rows[0])
    if any(len(row) != width for row in rows):
        raise ValueError("Every row in a test map must have the same width")

    data = []
    start = goal = None
    for row_number, row in enumerate(reversed(rows)):
        for column, symbol in enumerate(row):
            data.append(CELL_COST[symbol])
            if symbol == "S":
                start = (column, row_number)
            elif symbol == "G":
                goal = (column, row_number)

    # nav2 stores the bottom row first, unlike the pictures above
    grid = SimpleNamespace(
        header=SimpleNamespace(frame_id="map"),
        metadata=SimpleNamespace(
            size_x=width,
            size_y=len(rows),
            resolution=1.0,
            origin=SimpleNamespace(position=SimpleNamespace(x=0.0, y=0.0)),
        ),
        data=data,
    )
    return grid, start, goal


def pose_at(cell):
    """Place a test pose in the middle of a map cell."""
    column, row = cell
    return SimpleNamespace(
        header=SimpleNamespace(frame_id="map"),
        pose=SimpleNamespace(position=SimpleNamespace(x=column + 0.5, y=row + 0.5)),
    )


def shortest_four_way_route(grid, start, goal):
    """Find the shortest route using only up, down, left and right moves."""
    queue = deque([(start, 0)])
    seen = {start}
    while queue:
        (column, row), steps = queue.popleft()
        if (column, row) == goal:
            return steps

        for dx, dy in ((1, 0), (-1, 0), (0, 1), (0, -1)):
            next_column, next_row = column + dx, row + dy
            next_cell = (next_column, next_row)
            if (
                0 <= next_column < grid.metadata.size_x
                and 0 <= next_row < grid.metadata.size_y
                and next_cell not in seen
                and grid.data[next_row * grid.metadata.size_x + next_column] < 253
            ):
                seen.add(next_cell)
                queue.append((next_cell, steps + 1))
    return None


if __name__ == "__main__":
    unittest.main()
