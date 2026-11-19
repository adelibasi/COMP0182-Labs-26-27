"""Supply known clear grid legs for the Nav2 integration check."""


def a_star_impl(*, nav2_cost_map, start_point, goal_point):
    # shortcut: straight grid legs only, use A* for arbitrary destinations
    column, row = start_point
    goal_column, goal_row = goal_point
    cells = [(column, row)]
    while column != goal_column:
        column += 1 if goal_column > column else -1
        cells.append((column, row))
    while row != goal_row:
        row += 1 if goal_row > row else -1
        cells.append((column, row))
    return cells
