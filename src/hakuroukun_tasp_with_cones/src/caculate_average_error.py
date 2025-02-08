#!/usr/bin/env python3

import pandas as pd
import numpy as np
from scipy.spatial import KDTree

def load_data(file_path, x_col, y_col):
    """
    Load the text file and extract x, y coordinates.
    """
    df = pd.read_csv(file_path, comment="#", names=[x_col, y_col])
    df[x_col] = pd.to_numeric(df[x_col], errors='coerce')
    df[y_col] = pd.to_numeric(df[y_col], errors='coerce')
    df = df.dropna().reset_index(drop=True)
    return df[[x_col, y_col]].values

def compute_distance_errors(tasp_file, robot_file):
    """
    Compute the Average and Maximum Distance Error using the nearest neighbor approach.
    """
    # Load TASP and robot positions
    tasp_positions = load_data(tasp_file, "tasp_x", "tasp_y")
    robot_positions = load_data(robot_file, "robot_x", "robot_y")
    
    # Build a KDTree for fast nearest neighbor search
    robot_tree = KDTree(robot_positions)
    
    # Find the nearest robot position for each TASP goal position
    distances, _ = robot_tree.query(tasp_positions)
    
    # Compute the Average and Maximum Distance Errors
    average_distance_error = distances.mean()
    max_distance_error = distances.max()
    
    return average_distance_error, max_distance_error

if __name__ == "__main__":
    # Example file paths (replace with actual paths)
    tasp_file = "tasp.txt"
    robot_file = "robot.txt"
    
    avg_error, max_error = compute_distance_errors(tasp_file, robot_file)
    print(f"Average Distance Error: {avg_error:.4f}")
    print(f"Maximum Distance Error: {max_error:.4f}")
