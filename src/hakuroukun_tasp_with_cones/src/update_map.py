#!/usr/bin/env python3
import rospy
import math
import tf2_ros
from nav_msgs.msg import OccupancyGrid
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
from tf.transformations import euler_from_quaternion

class UpdateMap:
    def __init__(self, resolution, width, height, origin, camera_fov, camera_range):
        # Initialize TF2 Buffer and Listener
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        # Initialize the occupancy grid
        self.grid = OccupancyGrid()
        self.grid.info.resolution = resolution
        self.grid.info.width = width
        self.grid.info.height = height

        # The map origin (in world coordinates)
        self.grid.info.origin.position.x = origin[0]
        self.grid.info.origin.position.y = origin[1]
        self.grid.info.origin.position.z = 0.0

        # Initialize map data as unknown (-1)
        self.grid.data = [-1] * (width * height)

        # Camera FoV parameters
        self.camera_fov = camera_fov
        self.camera_range = camera_range

        # Initialize the camera FoV marker
        self.fov_marker = Marker()
        self.fov_marker.type = Marker.LINE_STRIP
        self.fov_marker.action = Marker.ADD
        self.fov_marker.scale.x = 0.05  # Line thickness
        self.fov_marker.color.r = 0.0
        self.fov_marker.color.g = 1.0
        self.fov_marker.color.b = 0.0
        self.fov_marker.color.a = 1.0
        self.fov_marker.pose.orientation.x = 0.0
        self.fov_marker.pose.orientation.y = 0.0
        self.fov_marker.pose.orientation.z = 0.0
        self.fov_marker.pose.orientation.w = 1.0
        self.fov_marker.header.frame_id = "map"
        self.fov_marker.ns = "camera_fov"
        self.fov_marker.id = 0

    def update_map_with_scan(self, scan):
        """
        Update the occupancy grid map and FoV marker based on the latest LaserScan data.

        Parameters:
        - scan: LaserScan message.
        """
        try:

            # Transform from 'map' to 'lidar_center'
            transform_lidar = self.tf_buffer.lookup_transform(
                'map',            # target frame
                'lidar_center',   # source frame
                rospy.Time(0)
            )

            # Transform from 'map' to 'camera_link'
            transform_camera = self.tf_buffer.lookup_transform(
                'map',           # target frame
                'camera_link',   # source frame
                rospy.Time(0)
            )

            # Extract lidar_center pose
            lidar_pose = transform_lidar.transform.translation
            lidar_orientation = transform_lidar.transform.rotation
            lidar_x = lidar_pose.x
            lidar_y = lidar_pose.y
            _, _, lidar_theta = self.quaternion_to_euler(lidar_orientation)

            # Update the map with the scan data relative to lidar_center
            self.process_scan((lidar_x, lidar_y, lidar_theta), scan)

            # Extract camera_link pose for FoV marker
            camera_pose = transform_camera.transform.translation
            camera_orientation = transform_camera.transform.rotation
            camera_x = camera_pose.x
            camera_y = camera_pose.y
            _, _, camera_theta = self.quaternion_to_euler(camera_orientation)
            camera_theta = camera_theta

            # Update the FoV marker based on camera_link pose
            self.update_fov_marker((camera_x, camera_y, camera_theta))

        except tf2_ros.LookupException as e:
            rospy.logwarn("TF2 LookupException: {}".format(e))
        except tf2_ros.ConnectivityException as e:
            rospy.logwarn("TF2 ConnectivityException: {}".format(e))
        except tf2_ros.ExtrapolationException as e:
            rospy.logwarn("TF2 ExtrapolationException: {}".format(e))  

    def quaternion_to_euler(self, q):
        """
        Convert a quaternion into Euler angles (roll, pitch, yaw).

        Returns:
        - Tuple of (roll, pitch, yaw)
        """
        return euler_from_quaternion([q.x, q.y, q.z, q.w])
    
    def process_scan(self, lidar_pose, scan):
        """
        Process the LaserScan data to update the occupancy grid.

        Parameters:
        - lidar_pose: Tuple (x, y, theta) representing the lidar_center's pose.
        - scan: LaserScan message.
        """
        lidar_x, lidar_y, lidar_theta = lidar_pose

        for i, raw_r in enumerate(scan.ranges):
            # Handle infinite or out-of-range measurements
            if math.isinf(raw_r) or raw_r > scan.range_max:
                r = scan.range_max
                mark_obstacle = False  # Always skip marking endpoint as an obstacle
            else:
                r = raw_r
                mark_obstacle = True  # Mark endpoint as obstacle

            # Ensure r is within [range_min, range_max] and not NaN
            if scan.range_min <= r <= scan.range_max and not math.isnan(r):
                # Calculate the global angle for this scan point
                angle = scan.angle_min + i * scan.angle_increment + lidar_theta
                obstacle_x = lidar_x + r * math.cos(angle)
                obstacle_y = lidar_y + r * math.sin(angle)

                # Only mark endpoint as an obstacle if it's a valid measurement
                if mark_obstacle:
                    grid_x, grid_y = self.to_grid_coordinates(obstacle_x, obstacle_y)
                    if 0 <= grid_x < self.grid.info.width and 0 <= grid_y < self.grid.info.height:
                        index = grid_y * self.grid.info.width + grid_x
                        self.grid.data[index] = 100  # Occupied

                # Mark free space between lidar_center and the obstacle
                # self.mark_free_space(lidar_x, lidar_y, obstacle_x, obstacle_y)

    def update_fov_marker(self, camera_pose):
        """
        Update the Field of View (FoV) marker based on camera_link's pose.

        Parameters:
        - camera_pose: Tuple (x, y, theta) representing the camera's pose.
        """
        camera_x, camera_y, camera_theta = camera_pose

        # Reset the marker points for the new frame
        self.fov_marker.points = []

        # Add the camera's position as the first point of the FoV
        self.fov_marker.points.append(Point(camera_x, camera_y, 0.0))

        # Calculate FoV boundaries
        left_angle = camera_theta - self.camera_fov / 2
        right_angle = camera_theta + self.camera_fov / 2

        # Compute the boundary points based on camera_pose
        left_x = camera_x + self.camera_range * math.cos(left_angle)
        left_y = camera_y + self.camera_range * math.sin(left_angle)

        right_x = camera_x + self.camera_range * math.cos(right_angle)
        right_y = camera_y + self.camera_range * math.sin(right_angle)

        # Add FoV boundary points to the marker
        self.fov_marker.points.append(Point(left_x, left_y, 0.0))
        self.fov_marker.points.append(Point(right_x, right_y, 0.0))
        self.fov_marker.points.append(Point(camera_x, camera_y, 0.0))  # Close the triangle

    def publish_map(self, map_publisher, fov_publisher):
        """Publish the occupancy grid and FoV marker."""
        self.grid.header.stamp = rospy.Time.now()
        self.grid.header.frame_id = "map"
        map_publisher.publish(self.grid)

        self.fov_marker.header.stamp = rospy.Time.now()
        fov_publisher.publish(self.fov_marker)

    def to_grid_coordinates(self, x, y):
        """
        Convert a (x, y) point in world coordinates
        to (grid_x, grid_y) indices in the occupancy grid.
        """
        grid_x = int((x - self.grid.info.origin.position.x) / self.grid.info.resolution)
        grid_y = int((y - self.grid.info.origin.position.y) / self.grid.info.resolution)
        return grid_x, grid_y

    def mark_free_space(self, start_x, start_y, end_x, end_y):
        """
        Mark the cells along the line from (start_x, start_y) to (end_x, end_y)
        as free (0) using Bresenham's Line Algorithm.
        """
        # Convert start and end points to grid coordinates
        start_grid_x, start_grid_y = self.to_grid_coordinates(start_x, start_y)
        end_grid_x, end_grid_y = self.to_grid_coordinates(end_x, end_y)

        # Compute differences
        dx = abs(end_grid_x - start_grid_x)
        dy = abs(end_grid_y - start_grid_y)
        sx = 1 if start_grid_x < end_grid_x else -1
        sy = 1 if start_grid_y < end_grid_y else -1
        err = dx - dy

        x, y = start_grid_x, start_grid_y

        # Iterate through grid cells
        while True:
            # Check grid bounds
            if 0 <= x < self.grid.info.width and 0 <= y < self.grid.info.height:
                index = y * self.grid.info.width + x

                # Stop if we've reached the end point
                if x == end_grid_x and y == end_grid_y:
                    break

                # Mark as free space only when the cell is unknown (-1)
                if self.grid.data[index] == -1:
                    self.grid.data[index] = 0

            # Update error and step
            e2 = 2 * err
            if e2 > -dy:
                err -= dy
                x += sx
            if e2 < dx:
                err += dx
                y += sy
