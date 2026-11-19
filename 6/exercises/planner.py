"""Your A* planner for the Nav2 global costmap."""

from __future__ import annotations

from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from nav2_msgs.msg import Costmap


def a_star_impl(
    *,
    nav2_cost_map: Costmap,
    start_point: tuple[int, int],
    goal_point: tuple[int, int],
) -> list[tuple[int, int]]:
    """Find a route across Nav2's current global costmap.

    Inputs:
        ``nav2_cost_map`` is the live costmap, not the saved house PGM.
        ``start_point`` and ``goal_point`` are (column, row) cell indices.
        The adapter validates the map and endpoints before calling A*.
        Assume a well-formed map and in-bounds, traversable endpoints.

    Grid layout:
        ``metadata.size_x`` and ``metadata.size_y`` give its width and height.
        ``data[row * metadata.size_x + column]`` gives one cell's cost.
        Keep both indices within the grid.

    Costs:
        0 is free; 1–252 increase nearer obstacles. Nav2 uses 253 for the
        inflated obstacle boundary, 254 for an obstacle and 255 for unknown
        space. Do not enter cells with costs 253–255.
        Minimise steps first, then the sum of entered-cell costs
        (excluding the start cell) among equally short routes.

    Return:
        Four-connected (column, row) cells, including start and goal.
        Diagonal movement is outside the core assessed implementation.
        Return [] if no route exists; [start_point] if start equals goal.
    """
    # expand the lowest estimated-cost cell from a priority queue

    # record each usable neighbour's best cost and predecessor

    # reconstruct the route when the goal is reached
    raise NotImplementedError("Implement A* in exercises/planner.py")
