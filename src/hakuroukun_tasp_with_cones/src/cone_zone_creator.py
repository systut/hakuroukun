#!/usr/bin/env python3

import rospy
from visualization_msgs.msg import MarkerArray, Marker
from geometry_msgs.msg import Point
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2
import tf2_ros
import tf
import numpy as np

# Parameters
CIRCULAR_REGION_RADIUS = 4  # Maximum distance to connect cones (meters)

# Global Variables
detected_cones = []  # Store unique cones in world coordinates
cone_lines = []  # List of cone lines (each line is a list of connected cone positions)

# Publisher for cone lines as markers
line_publisher = rospy.Publisher('/cone_lines_markers', MarkerArray, queue_size=10, latch=True)
# Publisher for cone lines as obstacles
obstacle_publisher = rospy.Publisher('/cone_lines_obstacles', PointCloud2, queue_size=10, latch=True)

def is_duplicate_cone(cone, cone_list):
    for existing_cone in cone_list:
        if np.linalg.norm(np.array(cone) - np.array(existing_cone)) <= 0.4:  # Small threshold
            return True
    return False

def transform_cone_to_world(cone_position, transform):
    translation = transform.transform.translation
    rotation = transform.transform.rotation
    translation_matrix = tf.transformations.translation_matrix([translation.x, translation.y, translation.z])
    rotation_matrix = tf.transformations.quaternion_matrix([rotation.x, rotation.y, rotation.z, rotation.w])
    camera_to_map_transform = np.dot(translation_matrix, rotation_matrix)
    cone_position_h = np.array([cone_position[0], cone_position[1], cone_position[2], 1.0])
    cone_in_world_h = np.dot(camera_to_map_transform, cone_position_h)
    return cone_in_world_h[:3]

def greedy_nearest_neighbor(detected_cones):
    """
    Implement the Greedy Nearest Neighbor algorithm to create cone lines
    and optionally close them into polygons.
    """
    visited = [False] * len(detected_cones)
    new_cone_lines = []

    while not all(visited):
        # Start a new line with the first unvisited cone
        idx = visited.index(False)
        current_cone_front = detected_cones[idx]
        current_cone_back = current_cone_front
        visited[idx] = True
        current_line = [current_cone_front]

        while True:
            expansion_occurred = False

            # Expand the line from the front
            unvisited_cones = [detected_cones[i] for i in range(len(detected_cones)) if not visited[i]]
            if unvisited_cones:
                distances_front = [np.linalg.norm(np.array(c) - np.array(current_cone_front)) for c in unvisited_cones]
                min_dist_front = min(distances_front)
                if min_dist_front <= CIRCULAR_REGION_RADIUS:
                    nearest_cone_front = unvisited_cones[distances_front.index(min_dist_front)]
                    current_line.insert(0, nearest_cone_front)
                    current_cone_front = nearest_cone_front

                    # Find the index of nearest_cone_front in detected_cones and mark it as visited
                    nearest_cone_index_front = next(
                        i for i, cone in enumerate(detected_cones) if np.allclose(cone, nearest_cone_front, atol=1e-6)
                    )
                    visited[nearest_cone_index_front] = True
                    expansion_occurred = True

            # Expand the line from the back
            unvisited_cones = [detected_cones[i] for i in range(len(detected_cones)) if not visited[i]]
            if unvisited_cones:
                distances_back = [np.linalg.norm(np.array(c) - np.array(current_cone_back)) for c in unvisited_cones]
                min_dist_back = min(distances_back)
                if min_dist_back <= CIRCULAR_REGION_RADIUS:
                    nearest_cone_back = unvisited_cones[distances_back.index(min_dist_back)]
                    current_line.append(nearest_cone_back)
                    current_cone_back = nearest_cone_back

                    # Find the index of nearest_cone_back in detected_cones and mark it as visited
                    nearest_cone_index_back = next(
                        i for i, cone in enumerate(detected_cones) if np.allclose(cone, nearest_cone_back, atol=1e-6)
                    )
                    visited[nearest_cone_index_back] = True
                    expansion_occurred = True

            # Stop expanding if no expansions occurred
            if not expansion_occurred:
                break

        new_cone_lines.append(current_line)

    # Post-process: Close lines into polygons if the ends are close enough
    for line in new_cone_lines:
        if len(line) > 2:  # Only attempt to close lines with at least 3 points
            first_cone = line[0]
            last_cone = line[-1]

            # Check if the ends are close enough to connect
            if np.linalg.norm(np.array(first_cone) - np.array(last_cone)) <= CIRCULAR_REGION_RADIUS:
                line.append(first_cone)  # Close the line by connecting the last cone to the first

    return new_cone_lines

def create_marker_array(cone_lines):
    marker_array = MarkerArray()
    marker_id = 0
    for line in cone_lines:
        if len(line) < 2:
            continue
        line_marker = Marker()
        line_marker.header.frame_id = "map"
        line_marker.header.stamp = rospy.Time.now()
        line_marker.ns = "cone_lines"
        line_marker.id = marker_id
        line_marker.type = Marker.LINE_STRIP
        line_marker.action = Marker.ADD
        line_marker.scale.x = 0.05
        line_marker.color.r = 1.0
        line_marker.color.g = 0.5
        line_marker.color.b = 0.0
        line_marker.color.a = 1.0
        line_marker.pose.orientation.x = 0.0
        line_marker.pose.orientation.y = 0.0
        line_marker.pose.orientation.z = 0.0
        line_marker.pose.orientation.w = 1.0
        for cone in line:
            point = Point()
            point.x, point.y, point.z = cone[0], cone[1], 0.0
            line_marker.points.append(point)
        marker_array.markers.append(line_marker)
        marker_id += 1
    return marker_array

def detected_cones_callback(msg):
    global detected_cones, cone_lines
    try:
        transform = tf_buffer.lookup_transform("map", "camera_link", rospy.Time(0))
        for marker in msg.markers:
            cone_camera = [marker.pose.position.x, marker.pose.position.y, marker.pose.position.z]
            cone_world = transform_cone_to_world(cone_camera, transform)
            if not is_duplicate_cone(cone_world, detected_cones):
                detected_cones.append(cone_world)
        cone_lines = greedy_nearest_neighbor(detected_cones)
        line_markers = create_marker_array(cone_lines)
        publish_obstacles(cone_lines, resolution=0.05)  # Assuming 0.05 meters per cell
        line_publisher.publish(line_markers)
    except tf2_ros.LookupException:
        rospy.logwarn("Transform from camera_frame to map not available yet")
    except tf2_ros.ExtrapolationException:
        rospy.logwarn("Extrapolation error while looking up transform")

def bresenham(start, end, resolution):
    """Generate grid cells along a line using Bresenham's algorithm."""
    x1, y1 = int(start[0] / resolution), int(start[1] / resolution)
    x2, y2 = int(end[0] / resolution), int(end[1] / resolution)
    cells = []
    dx = abs(x2 - x1)
    dy = abs(y2 - y1)
    sx = 1 if x1 < x2 else -1
    sy = 1 if y1 < y2 else -1
    err = dx - dy
    while True:
        cells.append((x1, y1))
        if x1 == x2 and y1 == y2:
            break
        e2 = 2 * err
        if e2 > -dy:
            err -= dy
            x1 += sx
        if e2 < dx:
            err += dx
            y1 += sy
    return cells

def publish_obstacles(cone_lines, resolution):
    points = []
    for line in cone_lines:
        for i in range(len(line) - 1):
            start = line[i]
            end = line[i + 1]
            # Get cells along the line
            cells = bresenham(start[:2], end[:2], resolution)
            for cell in cells:
                # Convert cell indices back to world coordinates
                x, y = cell[0] * resolution, cell[1] * resolution
                points.append([x, y, 0.0])  # (x, y, z)

    # Create PointCloud2 message
    header = rospy.Header()
    header.frame_id = "map"
    header.stamp = rospy.Time.now()
    point_cloud_msg = pc2.create_cloud_xyz32(header, points)

    # Publish as obstacles
    obstacle_publisher.publish(point_cloud_msg)

def main():
    global tf_buffer, tf_listener
    rospy.init_node('cone_zone_creator', anonymous=True)
    tf_buffer = tf2_ros.Buffer()
    tf_listener = tf2_ros.TransformListener(tf_buffer)
    rospy.Subscriber('/detected_cones_markers', MarkerArray, detected_cones_callback)
    rospy.spin()

if __name__ == '__main__':
    main()
