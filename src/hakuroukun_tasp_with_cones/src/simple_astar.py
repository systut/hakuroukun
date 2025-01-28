#!/usr/bin/env python3
import heapq
import rospy

class SimpleOccupancyGrid:
    """
    A lightweight wrapper for storing map info. Typically, you'd get these fields from:
      - occupancy_grid.info.width
      - occupancy_grid.info.height
      - occupancy_grid.info.resolution
      - occupancy_grid.info.origin.position.x
      - occupancy_grid.info.origin.position.y
      - occupancy_grid.data (list of int: 0=free, 100=occ, -1=unknown)
    """
    def __init__(self, width, height, resolution, origin_x, origin_y, data):
        self.width = width
        self.height = height
        self.resolution = resolution
        self.origin_x = origin_x
        self.origin_y = origin_y
        self.data = data  # A list of length width*height

def world_to_grid(ogrid, wx, wy):
    """
    Convert world coordinates (wx, wy) to grid indices (gx, gy).
    Return None if out of bounds.
    """
    gx = int((wx - ogrid.origin_x) / ogrid.resolution)
    gy = int((wy - ogrid.origin_y) / ogrid.resolution)
    if gx < 0 or gx >= ogrid.width or gy < 0 or gy >= ogrid.height:
        return None  # out of map
    return (gx, gy)

def grid_to_world(ogrid, gx, gy):
    """
    Convert grid coords (gx, gy) back to world (wx, wy).
    """
    wx = ogrid.origin_x + (gx + 0.5) * ogrid.resolution
    wy = ogrid.origin_y + (gy + 0.5) * ogrid.resolution
    return (wx, wy)

def is_occupied(ogrid, gx, gy, treat_unknown_as_occupied=True):
    """
    Return True if cell (gx, gy) is occupied or out of bounds.
    If 'treat_unknown_as_occupied=True', we consider -1 as occupied.
    """
    if gx < 0 or gx >= ogrid.width or gy < 0 or gy >= ogrid.height:
        return True
    idx = gy * ogrid.width + gx
    val = ogrid.data[idx]  # 0=free, 100=occupied, -1=unknown
    if val < 0:
        return treat_unknown_as_occupied
    return (val >= 50)  # e.g., treat >=50 as occupied

def get_neighbors(gx, gy, connectivity=8):
    """
    Return neighbor offsets. For connectivity=8, includes diagonals.
    For connectivity=4, only up/down/left/right.
    """
    if connectivity == 4:
        return [(gx-1, gy), (gx+1, gy), (gx, gy-1), (gx, gy+1)]
    else:  # 8-connect
        return [
            (gx-1, gy), (gx+1, gy), (gx, gy-1), (gx, gy+1),
            (gx-1, gy-1), (gx-1, gy+1), (gx+1, gy-1), (gx+1, gy+1)
        ]

def heuristic(a, b, diag_ok=True):
    """
    Heuristic for A*: either Chebyshev distance for 8-connect
    or Manhattan for 4-connect.
    """
    (ax, ay) = a
    (bx, by) = b
    dx = abs(ax - bx)
    dy = abs(ay - by)
    if diag_ok:
        # Chebyshev distance
        return max(dx, dy)
    else:
        # Manhattan distance
        return dx + dy

def astar_plan(ogrid, wx_start, wy_start, wx_goal, wy_goal, connectivity=8):
    """
    Simple A* from a start in world coords -> goal in world coords.
    Returns a list of (wx, wy) if path found, else empty list.

    connectivity: 4 or 8
    """

    # 1. Convert world -> grid
    start_g = world_to_grid(ogrid, wx_start, wy_start)
    goal_g  = world_to_grid(ogrid, wx_goal, wy_goal)
    if not start_g or not goal_g:
        rospy.logwarn("[A*] Start or goal out of map bounds.")
        return []

    if is_occupied(ogrid, start_g[0], start_g[1]):
        rospy.logwarn("[A*] Start cell is occupied!")
        return []

    if is_occupied(ogrid, goal_g[0], goal_g[1]):
        rospy.logwarn("[A*] Goal cell is occupied!")
        return []

    # 2. Setup open/closed sets
    open_set = []
    heapq.heappush(open_set, (0, start_g))
    came_from = {}  # maps cell -> parent_cell
    g_score = {start_g: 0}
    visited = set()

    while open_set:
        # 3. Pop the lowest-f node
        _, current = heapq.heappop(open_set)
        if current in visited:
            continue
        visited.add(current)

        # 4. If we reached goal
        if current == goal_g:
            # Reconstruct path
            return reconstruct_path(came_from, current, ogrid)

        # 5. Check neighbors
        for n in get_neighbors(current[0], current[1], connectivity):
            if is_occupied(ogrid, n[0], n[1]):
                continue  # skip occupied
            if n in visited:
                continue
            # Move cost depends on diagonal or not
            cost_move = 1.0 if (abs(n[0] - current[0]) + abs(n[1] - current[1]) == 1) else 1.4
            tentative_g = g_score[current] + cost_move
            if n not in g_score or tentative_g < g_score[n]:
                came_from[n] = current
                g_score[n] = tentative_g
                h = heuristic(n, goal_g, diag_ok=(connectivity==8))
                f = tentative_g + h
                heapq.heappush(open_set, (f, n))

    # 6. No path found
    print("[A*] No path found.")
    return []

def reconstruct_path(came_from, current, ogrid):
    """
    Reconstruct path from 'current' back to start.
    Convert grid coords to world coords.
    """
    path = [current]
    while current in came_from:
        current = came_from[current]
        path.append(current)
    path.reverse()
    # convert grid -> world
    world_path = []
    for (gx, gy) in path:
        wx, wy = grid_to_world(ogrid, gx, gy)
        world_path.append((wx, wy))
    return world_path
