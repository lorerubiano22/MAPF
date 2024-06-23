from heapq import heappop, heappush

# Define the grid
# grid = [
#    [1, 1, 0, 1, 1, 1, 1, 1, 1],
#    [0, 0, 0, 1, 1, 1, 1, 1, 1],
#    [0, 0, 0, 0, 0, 0, 0, 0, 0],
#     [0, 0, 0, 0, 0, 0, 0, 0, 0],
# ]


grid = [
    [0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0],
    [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
    [0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0],
    [0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0],
    [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
]


# Define the start and goal positions
start_blue = (1, 0)
goal_blue = (1, 12)
start_red = (2, 12)
goal_red = (2, 0)

# Define directions for movement (left, right, stay)
directions = [(0, -1), (0, 1), (0, 0), (-1, 0), (1, 0)]  # left, right, stay


def heuristic(pos, goal):
    return abs(pos[0] - goal[0]) + abs(pos[1] - goal[1])


def a_star_simultaneous(start_blue, goal_blue, start_red, goal_red, grid):
    # Priority queue for A*
    open_set = []
    heappush(open_set, (0, start_blue, start_red, [start_blue], [start_red]))

    # Visited set to avoid processing the same state
    visited = set([(start_blue, start_red)])

    while open_set:
        _, current_blue, current_red, path_blue, path_red = heappop(open_set)

        # Check if both goals are reached
        if current_blue == goal_blue and current_red == goal_red:
            return path_blue, path_red

        for dx_blue, dy_blue in directions:
            for dx_red, dy_red in directions:
                # Blue's new position
                next_blue = (current_blue[0] + dx_blue, current_blue[1] + dy_blue)
                # Red's new position
                next_red = (current_red[0] + dx_red, current_red[1] + dy_red)

                if (
                    0 <= next_blue[0] < len(grid)
                    and 0 <= next_blue[1] < len(grid[0])
                    and grid[next_blue[0]][next_blue[1]] != 1
                    and 0 <= next_red[0] < len(grid)
                    and 0 <= next_red[1] < len(grid[0])
                    and grid[next_red[0]][next_red[1]] != 1
                ):

                    # Check if positions collide or cross each other
                    if (
                        next_blue != next_red
                        and next_blue != current_red
                        and next_red != current_blue
                    ):
                        new_state = (next_blue, next_red)

                        if new_state not in visited:
                            visited.add(new_state)
                            g_cost = len(path_blue) + 1  # each move costs 1
                            f_cost = (
                                g_cost
                                + heuristic(next_blue, goal_blue)
                                + heuristic(next_red, goal_red)
                            )
                            heappush(
                                open_set,
                                (
                                    f_cost,
                                    next_blue,
                                    next_red,
                                    path_blue + [next_blue],
                                    path_red + [next_red],
                                ),
                            )

    return None, None


# Find the simultaneous paths
path_blue, path_red = a_star_simultaneous(
    start_blue, goal_blue, start_red, goal_red, grid
)
print("Path for blue:", path_blue)
print("Path for red:", path_red)
