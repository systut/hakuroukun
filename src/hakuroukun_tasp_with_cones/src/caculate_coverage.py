#!/usr/bin/env python3
import rospy
import numpy as np
from nav_msgs.msg import OccupancyGrid

def calculate_coverage(grid_map):
    """ Calculate the cleaned area, total valid area, and coverage percentage. """
    grid_data = np.array(grid_map.data).reshape((grid_map.info.height, grid_map.info.width))
    resolution = grid_map.info.resolution  # Get map resolution (meters per cell)

    # Count cleaned and free (uncleaned) cells
    occupied_cells = np.sum(grid_data == 50)  # Count cleaned cells
    free_cells = np.sum(grid_data == 0)  # Count free (uncleaned) cells

    # Total valid area (excluding unknown cells -1)
    total_valid_cells = occupied_cells + free_cells

    # Convert to square meters (each cell has an area of resolution²)
    occupied_area = occupied_cells * (resolution ** 2)
    total_valid_area = total_valid_cells * (resolution ** 2)

    if total_valid_area == 0:
        return 0.0, 0.0, 0.0  # Avoid division by zero

    # Calculate coverage percentage
    coverage_percentage = (occupied_area / total_valid_area) * 100

    return occupied_area, total_valid_area, coverage_percentage

def map_callback(msg):
    occupied_area, total_valid_area, coverage = calculate_coverage(msg)

    rospy.loginfo(f"Occupied Area: {occupied_area:.2f} m²")
    rospy.loginfo(f"Total Valid Area: {total_valid_area:.2f} m²")
    rospy.loginfo(f"Coverage: {coverage:.2f}%")

def listener():
    rospy.init_node('coverage_calculator', anonymous=True)
    rospy.Subscriber("/cleaned_map", OccupancyGrid, map_callback)
    rospy.spin()

if __name__ == '__main__':
    listener()
