#!/usr/bin/env python3

import rospy, math, tf2_ros
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import PoseStamped, Point
from nav_msgs.msg import Path
from tf.transformations import euler_from_quaternion

class TASPVisualizer:
    def __init__(self, frame_id="odom"):
        # Initialize TF2 Buffer and Listener
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        # Publishers for various visualization messages
        self.btp_pub = rospy.Publisher('/tasp/btp_markers', MarkerArray, queue_size=10)
        self.goal_pub = rospy.Publisher('/tasp/goal_pose', PoseStamped, queue_size=10)
        self.goal_marker_pub = rospy.Publisher('/tasp/goal_marker', Marker, queue_size=10)
        self.path_pub = rospy.Publisher('/tasp/astar_path', Path, queue_size=10)
        self.start_marker_pub = rospy.Publisher('/tasp/start_marker', Marker, queue_size=10)
        self.fov_pub = rospy.Publisher("/camera_fov", Marker, queue_size=10)

        self.frame_id = frame_id
        self.camera_fov = math.radians(rospy.get_param("camera_fov", 70))
        self.camera_range = rospy.get_param("camera_range", 4) / math.cos(self.camera_fov / 2)

    def publish_start(self, start_pose):
        """
        start_pose: [x, y, (optionally theta)] or just [x, y].
        We'll ignore theta for the marker.
        """
        marker = Marker()
        marker.header.frame_id = self.frame_id
        marker.header.stamp = rospy.Time.now()
        marker.ns = "robot_start"
        marker.id = 0
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose.position.x = start_pose[0]
        marker.pose.position.y = start_pose[1]
        marker.pose.position.z = 0.0
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.1
        marker.scale.y = 0.1
        marker.scale.z = 0.1

        # Color = green
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        marker.color.a = 1.0

        self.start_marker_pub.publish(marker)

    def publish_btp(self, btp_points):
        """
        btp_points: list of [x, y].
        Publish new markers for the provided points, explicitly deleting old markers.
        """
        # Keep track of currently active marker IDs
        active_ids = set(range(len(btp_points)))
        # Keep track of previously published IDs
        if not hasattr(self, 'previous_ids'):
            self.previous_ids = set()

        # Create MarkerArray for new markers
        marker_array = MarkerArray()

        # Add new markers
        for i, (bx, by) in enumerate(btp_points):
            marker = Marker()
            marker.header.frame_id = self.frame_id
            marker.header.stamp = rospy.Time.now()
            marker.ns = "tasp_btp"
            marker.id = i
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD
            marker.pose.position.x = bx
            marker.pose.position.y = by
            marker.pose.position.z = 0.0
            marker.pose.orientation.w = 1.0
            marker.scale.x = 0.1
            marker.scale.y = 0.1
            marker.scale.z = 0.1

            # Blue color
            marker.color.r = 0.0
            marker.color.g = 0.0
            marker.color.b = 1.0
            marker.color.a = 1.0

            marker_array.markers.append(marker)

        # Determine IDs to delete (previously published but not in the new list)
        ids_to_delete = self.previous_ids - active_ids
        for id_to_delete in ids_to_delete:
            delete_marker = Marker()
            delete_marker.header.frame_id = self.frame_id
            delete_marker.header.stamp = rospy.Time.now()
            delete_marker.ns = "tasp_btp"
            delete_marker.id = id_to_delete
            delete_marker.action = Marker.DELETE
            marker_array.markers.append(delete_marker)

        # Update the previous IDs
        self.previous_ids = active_ids

        # Publish the MarkerArray
        self.btp_pub.publish(marker_array)

    def publish_goal(self, goal_point):
        """
        goal_point: [x, y]
        Publish as both a PoseStamped and a Marker.
        """
        gx, gy = goal_point

        # PoseStamped
        goal_pose = PoseStamped()
        goal_pose.header.frame_id = self.frame_id
        goal_pose.header.stamp = rospy.Time.now()
        goal_pose.pose.position.x = gx
        goal_pose.pose.position.y = gy
        goal_pose.pose.orientation.w = 1.0
        self.goal_pub.publish(goal_pose)

        # Marker
        marker = Marker()
        marker.header.frame_id = self.frame_id
        marker.header.stamp = rospy.Time.now()
        marker.ns = "tasp_goal"
        marker.id = 0
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose.position.x = gx
        marker.pose.position.y = gy
        marker.pose.position.z = 0.0
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.1
        marker.scale.y = 0.1
        marker.scale.z = 0.1

        # Red color
        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0
        marker.color.a = 1.0

        self.goal_marker_pub.publish(marker)

    def publish_camera_fov(self):
        """
        Publishes a Field of View (FoV) marker based on the camera_link's pose in the map frame.

        This function calculates the camera's FoV boundaries using its pose and orientation
        relative to the map frame and visualizes the FoV as a triangle marker in RViz.
        """
        try:
            # Lookup the transform from 'map' to 'camera_link'
            transform_camera = self.tf_buffer.lookup_transform(
                'odom',           # Target frame
                'camera_link',   # Source frame
                rospy.Time(0)    # Most recent transform available
            )

            # Extract camera's position and orientation
            camera_pose = transform_camera.transform.translation
            camera_orientation = transform_camera.transform.rotation
            camera_x = camera_pose.x
            camera_y = camera_pose.y

            # Convert orientation quaternion to Euler angles (yaw = theta)
            quaternion = [camera_orientation.x, camera_orientation.y, camera_orientation.z, camera_orientation.w]
            _, _, camera_theta = euler_from_quaternion(quaternion)

            # Initialize the FoV marker
            fov_marker = Marker()
            fov_marker.header.frame_id = self.frame_id
            fov_marker.header.stamp = rospy.Time.now()
            fov_marker.ns = "camera_fov"
            fov_marker.id = 0
            fov_marker.type = Marker.LINE_STRIP  # Line strip to draw the FoV triangle
            fov_marker.action = Marker.ADD
            fov_marker.scale.x = 0.05  # Line thickness
            fov_marker.color.r = 0.0
            fov_marker.color.g = 1.0
            fov_marker.color.b = 0.0
            fov_marker.color.a = 1.0

            # Calculate the FoV boundaries (left and right rays)
            left_angle = camera_theta - self.camera_fov / 2
            right_angle = camera_theta + self.camera_fov / 2

            # Compute the boundary points based on camera_pose
            left_x = camera_x + self.camera_range * math.cos(left_angle)
            left_y = camera_y + self.camera_range * math.sin(left_angle)
            right_x = camera_x + self.camera_range * math.cos(right_angle)
            right_y = camera_y + self.camera_range * math.sin(right_angle)

            # Define the points of the FoV triangle
            fov_marker.points = [
                Point(camera_x, camera_y, 0.0),  # Camera position
                Point(left_x, left_y, 0.0),     # Left boundary point
                Point(right_x, right_y, 0.0),   # Right boundary point
                Point(camera_x, camera_y, 0.0)  # Close the triangle
            ]

            # Publish the FoV marker
            self.fov_pub.publish(fov_marker)

        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            rospy.logwarn(f"Failed to lookup transform from 'map' to 'camera_link': {e}")

    def publish_astar_path(self, path_points):
        """
        path_points: list of [x, y] from A*
        We'll publish as nav_msgs/Path
        """
        path_msg = Path()
        path_msg.header.frame_id = self.frame_id
        path_msg.header.stamp = rospy.Time.now()

        for (px, py) in path_points:
            pose_stamped = PoseStamped()
            pose_stamped.header.frame_id = self.frame_id
            pose_stamped.header.stamp = rospy.Time.now()
            pose_stamped.pose.position.x = px
            pose_stamped.pose.position.y = py
            pose_stamped.pose.orientation.w = 1.0
            path_msg.poses.append(pose_stamped)

        self.path_pub.publish(path_msg)
