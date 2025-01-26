#!/usr/bin/env python3
import math

class TASPPathPlanner:
    def __init__(self, tasp_cell_size=1.0):
        """
        tasp_cell_size: The step (in meters) when exploring neighboring cells.
        """
        self.tasp_cell_size = tasp_cell_size
        
        # BackTracking Points
        self.BTP = []
        # TASP trajectory (a list of [x, y] waypoints)
        self.TASPtrajectory = []
        
    def tasp_path_planning(self, current_pose, unknown_map, start_pose):
        """
        Main TASP function that determines a new TASP goal.
        
        Args:
            current_pose (tuple): (x, y, theta) of the robot's current position
            unknown_map (OccupancyGrid or your map structure):
                - Must provide a way to query occupancy info (0=free, -1=unknown, 100=occupied).
            start_pose (tuple): (x, y, theta) the robot’s initial pose
        
        Returns:
            TASPgoal (list [x, y]) or None if no goal is available
        """
        # If TASPtrajectory has fewer than 2 points, initialize it
        if len(self.TASPtrajectory) < 2:
            self.TASPtrajectory = [
                [current_pose[0], current_pose[1]],   # second point
                [start_pose[0], start_pose[1]]     # start position
            ]

        TASPcurrentPos = self.TASPtrajectory[-1]  # last position in TASPtrajectory
        TASPprevPos    = self.TASPtrajectory[-2]  # second-last position

        # 1. Get the front cell
        front_cell = self.get_front_cell(TASPcurrentPos, TASPprevPos, self.tasp_cell_size)
        
        # 2. Find free neighbors
        free_cells = self.get_free_cells(TASPcurrentPos, unknown_map, self.tasp_cell_size, self.TASPtrajectory)
        
        # 3. Update BTP with these free cells
        self.update_BTP(free_cells)
        
        # 4. Choose the next TASPgoal
        if not free_cells:
            # If no free cells, go to the closest BTP
            if not self.BTP:
                print("[TASP] No BTP available. Stopping the robot.")
                return None
            # Move to the closest BTP from TASPcurrentPos
            next_goal = self.find_closest_BTP(TASPcurrentPos, self.BTP, start_pose, unknown_map)
            if next_goal is None:
                print("[TASP] No valid path to BTP. Stopping the robot.")
                return None
            self.TASPtrajectory.append(next_goal)

        else:
            # If the cell in front is free, move straight
            if self.list_has_row(free_cells, front_cell):
                self.TASPtrajectory.append(front_cell)
            
            # If only one free cell, go there
            elif len(free_cells) == 1:
                self.TASPtrajectory.append(free_cells[0])
            
            # If two free cells, pick using "away from start" or distance to obstacle
            elif len(free_cells) == 2:
                cell1 = free_cells[0]
                cell2 = free_cells[1]
                
                c1_away = self.is_away_from_start(cell1, start_pose, TASPcurrentPos)
                c2_away = self.is_away_from_start(cell2, start_pose, TASPcurrentPos)
                
                if c1_away and not c2_away:
                    self.TASPtrajectory.append(cell1)
                elif c2_away and not c1_away:
                    self.TASPtrajectory.append(cell2)
                else:
                    # Both away (or both not away), pick one with the longer free path
                    d1 = self.get_distance_to_obstacle(TASPcurrentPos, cell1, unknown_map, self.tasp_cell_size)
                    d2 = self.get_distance_to_obstacle(TASPcurrentPos, cell2, unknown_map, self.tasp_cell_size)
                    if d1 > d2:
                        self.TASPtrajectory.append(cell1)
                    else:
                        self.TASPtrajectory.append(cell2)

        # Remove the newly chosen position from BTP if it’s near any point in BTP
        newly_chosen = self.TASPtrajectory[-1]
        threshold_distance = 0.1  # Define a small threshold for "nearness"

        for btp_point in self.BTP:
            if self.euclidean_distance(newly_chosen, btp_point) < threshold_distance:
                self.BTP.remove(btp_point)
                break  # Remove only the first matching (or nearest) point

        # Return the newly chosen TASPgoal
        return self.TASPtrajectory[-1]
    
    # --------------------------------------------------------------------------
    # HELPER FUNCTIONS
    # --------------------------------------------------------------------------
    def get_front_cell(self, current_pos, prev_pos, tasp_cell_size):
        """
        Return the cell directly in front of the robot (based on vector from prev_pos -> current_pos).
        """
        dx = current_pos[0] - prev_pos[0]
        dy = current_pos[1] - prev_pos[1]
        length = math.hypot(dx, dy)
        if length < 1e-1:
            # If no movement, pick an arbitrary 'front cell'
            return [current_pos[0] + tasp_cell_size, current_pos[1]]
        else:
            # Unit direction
            ux = dx / length
            uy = dy / length
            return [
                current_pos[0] + ux * tasp_cell_size,
                current_pos[1] + uy * tasp_cell_size
            ]

    def get_free_cells(self, position, unknown_map, tasp_cell_size, TASPtrajectory):
        """
        Looks at four neighboring cells (left, right, up, down) at distance tasp_cell_size.
        Checks if they're free in the occupancy map. Returns a list of free [x, y].
        """
        px, py = position
        neighbors = [
            [px - tasp_cell_size, py], 
            [px + tasp_cell_size, py],
            [px, py - tasp_cell_size],
            [px, py + tasp_cell_size]
        ]
        
        free_cells = []
        for cell in neighbors:
            # Check if this cell area is free
            if self.is_area_free(cell, unknown_map, tasp_cell_size):
                # Exclude cells already in the TASPtrajectory
                if not self.list_has_row(TASPtrajectory, cell):
                    free_cells.append(cell)
        
        return free_cells

    def is_area_free(self, center_cell, unknown_map, tasp_cell_size):
        """
        Check a small square around center_cell for occupancy < 50 (treating 50 or above as occupied).
        We sample at a small resolution to see if any point in that square is occupied.
        """
        half_size = 4*tasp_cell_size / 2.0
        sampling_resolution = 0.1  # can adjust

        x_min = center_cell[0] - half_size
        x_max = center_cell[0] + half_size
        y_min = center_cell[1] - half_size
        y_max = center_cell[1] + half_size

        xs = self.frange(x_min, x_max, sampling_resolution)
        ys = self.frange(y_min, y_max, sampling_resolution)

        for x_s in xs:
            for y_s in ys:
                occ_val = self.get_occupancy_value(unknown_map, x_s, y_s)
                if occ_val >= 50:  
                    # 50 or 100 => unknown or occupied => treat as not free
                    return False
        return True

    def update_BTP(self, free_cells):
        """
        Append free cells to BTP if they aren’t already there.
        """
        for cell in free_cells:
            if not self.list_has_row(self.BTP, cell):
                self.BTP.append(cell)

    def find_closest_BTP(self, current_pos, BTP, start_pose, unknown_map):
        """
        For each point in BTP, pick the closest (Euclidean). 
        If tie => pick one that is farthest from start_pose.
        """
        min_dist = float('inf')
        chosen = None
        for point in BTP:
            dist = self.euclidean_distance(current_pos, point)
            if dist < min_dist:
                min_dist = dist
                chosen = point
            elif abs(dist - min_dist) < 1e-6:
                # Tie => pick one farther from start_pose
                if (self.euclidean_distance(point, [start_pose[0], start_pose[1]]) >
                    self.euclidean_distance(chosen, [start_pose[0], start_pose[1]])):
                    chosen = point
        return chosen

    def is_away_from_start(self, cell, start_pose, current_pos):
        """
        Return True if 'cell' is farther from start than 'current_pos'.
        """
        dist_cell_to_start = self.euclidean_distance(cell, [start_pose[0], start_pose[1]])
        dist_current_to_start = self.euclidean_distance(current_pos, [start_pose[0], start_pose[1]])
        return dist_cell_to_start > dist_current_to_start
    
    def get_distance_to_obstacle(self, current_pos, cell, unknown_map, tasp_cell_size):
        """
        From 'cell' in direction from 'current_pos'->'cell', step by tasp_cell_size
        until occupancy >= 50 (occupied or unknown). Return total distance.
        """
        dx = cell[0] - current_pos[0]
        dy = cell[1] - current_pos[1]
        length = math.hypot(dx, dy)
        if length < 1e-6:
            return 0.0
        ux = dx / length
        uy = dy / length

        distance = 0.0
        current_x, current_y = cell[0], cell[1]

        while True:
            next_x = current_x + ux * tasp_cell_size
            next_y = current_y + uy * tasp_cell_size
            occ_val = self.get_occupancy_value(unknown_map, next_x, next_y)
            if occ_val >= 50:  # treat 50 or 100 as occupied/unknown
                break
            distance += tasp_cell_size
            current_x, current_y = next_x, next_y

        return distance

    # --------------------------------------------------------------------------
    # UTILITY / STUB FUNCTIONS
    # --------------------------------------------------------------------------
    def get_occupancy_value(self, unknown_map, x, y):
        """
        Return an integer in {0, 50, 100}:
          0   => free
          50  => unknown (-1 in OccupancyGrid)
          100 => occupied
        """
        gx, gy = self.world_to_grid(unknown_map, x, y)
        if (gx < 0 or gx >= unknown_map.info.width or
            gy < 0 or gy >= unknown_map.info.height):
            # Out of bounds => treat as occupied
            return 100
        
        idx = gy * unknown_map.info.width + gx
        val = unknown_map.data[idx]  # -1 (unknown), 0..100
        if val < 0:
            # unknown => treat as 50
            return 50
        return val  # 0 => free, 100 => occupied

    def world_to_grid(self, occupancy_grid, x, y):
        """
        Convert world coords to grid coords given occupancy_grid.info.
        """
        origin_x = occupancy_grid.info.origin.position.x
        origin_y = occupancy_grid.info.origin.position.y
        res = occupancy_grid.info.resolution
        gx = int((x - origin_x) / res)
        gy = int((y - origin_y) / res)
        return gx, gy

    def euclidean_distance(self, p1, p2):
        return math.hypot(p1[0] - p2[0], p1[1] - p2[1])

    def list_has_row(self, arr, element):
        """
        Check if arr (list of [x, y]) has an entry matching element (list [x, y]).
        """
        for row in arr:
            if (abs(row[0] - element[0]) < 1e-6 and
                abs(row[1] - element[1]) < 1e-6):
                return True
        return False

    def frange(self, start, stop, step):
        """
        Generate a list of float values from start to stop (inclusive) with a given step.
        """
        vals = []
        x = start
        # Use a small epsilon to avoid floating rounding issues
        epsilon = 1e-9
        while x <= stop + epsilon:
            vals.append(x)
            x += step
        return vals
