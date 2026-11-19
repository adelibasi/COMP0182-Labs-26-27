"""Check the worksheet's A* contract on small costmaps."""

import unittest
from types import SimpleNamespace as N

from planner import a_star_impl
from path_adapter import validate_path


def grid(width, height, costs):
    return N(metadata=N(size_x=width, size_y=height), data=costs)


class StudentContractTests(unittest.TestCase):
    def assert_route_score(self, g, start, goal, expected):
        path = a_star_impl(nav2_cost_map=g, start_point=start, goal_point=goal)
        validate_path(g, start, goal, path)
        for a, b in zip(path, path[1:]):
            self.assertEqual(abs(a[0] - b[0]) + abs(a[1] - b[1]), 1)
        width = g.metadata.size_x
        score = (len(path) - 1, sum(g.data[y * width + x] for x, y in path[1:]))
        self.assertEqual(score, expected)

    def test_shortest_steps_override_high_cost(self):
        w, h = 7, 5
        costs = [0] * (w * h)
        for y in range(1, 4):
            for x in range(1, 6):
                costs[y * w + x] = 252
        self.assert_route_score(grid(w, h, costs), (0, 2), (6, 2), (6, 5 * 252))

    def test_no_diagonal_shortcuts(self):
        self.assert_route_score(grid(3, 3, [0] * 9), (0, 0), (2, 2), (4, 0))

    def test_cost_breaks_only_equal_length_ties(self):
        w, h = 5, 3
        cells = [0] * (w * h)
        for x in (1, 2, 3):
            cells[1 * w + x] = 254
            cells[2 * w + x] = 200
        self.assert_route_score(grid(w, h, cells), (0, 1), (4, 1), (6, 0))

    def test_avoids_blocked_and_unknown_cells(self):
        for blocked in (253, 254, 255):
            with self.subTest(blocked=blocked):
                g = grid(3, 2, [0, blocked, 0, 0, 0, 0])
                self.assert_route_score(g, (0, 0), (2, 0), (4, 0))


if __name__ == "__main__":
    unittest.main()
