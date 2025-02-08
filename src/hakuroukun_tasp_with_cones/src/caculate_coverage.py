#!/usr/bin/env python3
import rospy
import numpy as np
from nav_msgs.msg import OccupancyGrid

def calculate_coverage(grid_map):
    grid_data = np.array(grid_map.data)

    # Count occupied (cleaned) and free (uncleaned) cells
    occupied_cells = np.sum(grid_data == 100)  # Count cleaned cells
    free_cells = np.sum(grid_data == 0)  # Count free (uncleaned) cells

    # Total area considered (exclude unknown cells -1)
    total_valid_cells = occupied_cells + free_cells

    if total_valid_cells == 0:
        return 0.0  # Avoid division by zero

    coverage_percentage = (occupied_cells / total_valid_cells) * 100
    return coverage_percentage

def map_callback(msg):
    coverage = calculate_coverage(msg)
    rospy.loginfo(f"Current Coverage: {coverage:.2f}%")

def listener():
    rospy.init_node('coverage_calculator', anonymous=True)
    rospy.Subscriber("/cleaned_map", OccupancyGrid, map_callback)
    rospy.spin()

if __name__ == '__main__':
    listener()
