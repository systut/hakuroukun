#!/usr/bin/env python3
import rospy
import numpy as np
from nav_msgs.msg import OccupancyGrid, Odometry
from collections import deque

# Global variables
map_data = None
resolution = 0.05  # Map resolution (meters per cell)
origin_x, origin_y = 0.0, 0.0  # Map origin
cleaning_width_m = 1.0  # Cleaning width (1m)
robot_trajectory = deque(maxlen=1)  # Store trajectory points efficiently
cleaned_map_pub = None
distance_threshold = 0.2  # Minimum movement to update cleaning
persistent_grid = None  # Stores all cleaned areas permanently
radius_cells = int((cleaning_width_m / 2) / resolution)  # Cleaning radius in grid cells

def world_to_grid(x, y):
    """ Convert world coordinates (x, y) to grid indices. """
    return int((x - origin_x) / resolution), int((y - origin_y) / resolution)

def initialize_persistent_grid(map_msg):
    """ Initialize persistent grid by copying data from the original map. """
    global persistent_grid
    width, height = map_msg.info.width, map_msg.info.height
    persistent_grid = np.array(map_msg.data, dtype=np.int8).reshape((height, width))

def generate_circular_mask(radius):
    """ Precompute a circular mask for fast cleaning area expansion. """
    size = 2 * radius + 1
    mask = np.zeros((size, size), dtype=bool)
    center = radius

    for i in range(size):
        for j in range(size):
            if (i - center) ** 2 + (j - center) ** 2 <= radius ** 2:
                mask[i, j] = True
    return mask

# Precompute the circular mask
circular_mask = generate_circular_mask(radius_cells)

def apply_cleaning():
    """ Mark cells as cleaned using a circular mask, keeping original obstacles. """
    global persistent_grid, robot_trajectory, circular_mask

    if persistent_grid is None:
        return  # No map available yet

    width, height = persistent_grid.shape[1], persistent_grid.shape[0]

    # Convert trajectory points to grid indices
    grid_points = np.array([world_to_grid(x, y) for x, y in robot_trajectory])

    # Remove points that are out of bounds
    valid_points = (0 <= grid_points[:, 0]) & (grid_points[:, 0] < width) & \
                   (0 <= grid_points[:, 1]) & (grid_points[:, 1] < height)
    grid_points = grid_points[valid_points]

    # Apply the circular mask around each trajectory point
    for gx, gy in grid_points:
        x_min, x_max = max(0, gx - radius_cells), min(width, gx + radius_cells + 1)
        y_min, y_max = max(0, gy - radius_cells), min(height, gy + radius_cells + 1)

        # Extract the region of interest (ROI)
        roi = persistent_grid[y_min:y_max, x_min:x_max]

        # Ensure the mask fits inside the ROI
        mask_height, mask_width = roi.shape
        mask = circular_mask[:mask_height, :mask_width]

        # Only mark free space (0) as cleaned (100)
        roi[mask & (roi == 0)] = 50

def map_callback(msg):
    """ Store the map and update global variables. """
    global map_data, resolution, origin_x, origin_y, persistent_grid
    rospy.loginfo("Map received!")
    map_data = msg
    resolution = msg.info.resolution
    origin_x = msg.info.origin.position.x
    origin_y = msg.info.origin.position.y

    # Initialize the persistent grid when the map is first received
    initialize_persistent_grid(msg)

def odom_callback(msg):
    """ Extract robot position from Odometry and update cleaning area efficiently. """
    global map_data, cleaned_map_pub, robot_trajectory  

    if map_data is None or persistent_grid is None:
        rospy.logwarn("Waiting for map data...")
        return

    # Extract (x, y) from Odometry message
    x, y = msg.pose.pose.position.x, msg.pose.pose.position.y

    # Ignore very close points (reduce processing)
    if robot_trajectory and np.linalg.norm(np.array(robot_trajectory[-1]) - np.array([x, y])) < distance_threshold:
        return  # Ignore if movement is too small

    # Append to trajectory queue (faster updates)
    robot_trajectory.append((x, y))  

    # Apply cleaning effect to persistent map
    apply_cleaning()

    # Create a new OccupancyGrid message from the persistent grid
    cleaned_map_msg = OccupancyGrid()
    cleaned_map_msg.header.stamp = rospy.Time.now()  # Update timestamp
    cleaned_map_msg.header.frame_id = map_data.header.frame_id
    cleaned_map_msg.info = map_data.info
    cleaned_map_msg.data = persistent_grid.flatten().tolist()  # Use the persistent map

    # Publish updated map
    cleaned_map_pub.publish(cleaned_map_msg)

def main():
    global cleaned_map_pub
    rospy.init_node('cleaning_simulator')

    rospy.Subscriber("/map", OccupancyGrid, map_callback)
    rospy.Subscriber("/ground_truth/odometry", Odometry, odom_callback)  #/ground_truth/odometry #/hakuroukun_pose/rear_wheel_odometry

    cleaned_map_pub = rospy.Publisher("/cleaned_map", OccupancyGrid, queue_size=1)  # Reduce queue size for real-time updates
    rospy.loginfo("Cleaning simulator started!")
    rospy.spin()

if __name__ == '__main__':
    main()
