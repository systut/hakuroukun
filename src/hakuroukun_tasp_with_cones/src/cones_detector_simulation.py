#!/usr/bin/env python3

import rospy
from visualization_msgs.msg import Marker, MarkerArray
from nav_msgs.msg import Odometry
import tf2_ros
import tf
import numpy as np

# Known cone positions (ground truth in map frame)
known_cones = [  # 30x30 area
    [15.0025, 5.01389, 0],
    [14.9751, 3.0297, 0],
    [15.0143, 1.02261, 0],
    [14.9873, -1.02179, 0],
    [15.0227, -3.01431, 0],
    [1.5989, 11.2212, 0],
    [-1.10123, 11.2544, 0],
    [-14.9668, 13.4438, 0],
    [-15.0192, 11.0303, 0],
    [-15.0191, 8.45919, 0],
    [-15.0265, 5.98603, 0],
    [-15.0256, 3.19414, 0],
    [5.96933, -9.01061, 0],
    [3.44179, -8.99816, 0],
    [2.16339, -7.05643, 0],
    [2.00909, -4.59106, 0],
    [3.73849, -4.19251, 0],
    [7.95047, -3.02739, 0],
    [5.25688, -3.01954, 0],
    [8.01531, -5.55253, 0],
    [5.97509, -6.82091, 0],
]

# Initialize ROS node
rospy.init_node('cone_detector', anonymous=True)

# Load parameters
CAMERA_FOV = np.deg2rad(rospy.get_param('~camera_fov', 90))  # Default: 90 degrees
CAMERA_RANGE = rospy.get_param('~camera_range', 5.0)  # Default: 5 meters

# Publisher
marker_array_publisher = rospy.Publisher('/detected_cones_markers', MarkerArray, queue_size=10)

# TF Buffer and Listener
tf_buffer = tf2_ros.Buffer()
tf_listener = tf2_ros.TransformListener(tf_buffer)

def is_in_camera_view(cone_camera_position):
    """
    Check if the cone (in camera frame) is within the camera's FOV and range.
    """
    distance = np.linalg.norm(cone_camera_position[:2])  # Ignore z-axis for distance
    if distance > CAMERA_RANGE:
        return False

    angle_to_cone = np.arctan2(cone_camera_position[1], cone_camera_position[0])  # Yaw angle
    angle_diff = np.abs(angle_to_cone)
    return angle_diff <= CAMERA_FOV / 2

def transform_cone_to_camera(cone_position):
    """
    Transform the cone position from map frame to camera_link frame using a direct lookup.

    Parameters:
    - cone_position: [x, y, z] position of the cone in the map frame.
    - tf_buffer: TF2 buffer object to lookup transformations.

    Returns:
    - [x, y, z] position of the cone in the camera_link frame.
    """
    try:
        # Lookup the transform from map frame to camera_link frame
        transform = tf_buffer.lookup_transform("camera_link", "odom", rospy.Time(0))

        # Extract translation and rotation
        translation = transform.transform.translation
        rotation = transform.transform.rotation

        # Create the transformation matrix
        translation_matrix = tf.transformations.translation_matrix([translation.x, translation.y, translation.z])
        rotation_matrix = tf.transformations.quaternion_matrix([rotation.x, rotation.y, rotation.z, rotation.w])
        map_to_camera_transform = np.dot(translation_matrix, rotation_matrix)

        # Transform cone position to camera frame
        cone_position_h = np.array([cone_position[0], cone_position[1], cone_position[2], 1.0])  # Homogeneous coordinates
        cone_in_camera_h = np.dot(map_to_camera_transform, cone_position_h)

        return cone_in_camera_h[:3]  # Return x, y, z coordinates
    except tf2_ros.LookupException:
        rospy.logwarn("Transform not available yet")
        return None
    except tf2_ros.ExtrapolationException:
        rospy.logwarn("Extrapolation error while looking up transform")
        return None

def simulate_cone_detection(odom_msg):
    """
    Simulate cone detection based on odom data and publish in the camera frame.
    """
    try:
        detected_markers = MarkerArray()
        for i, cone in enumerate(known_cones):
            # Transform cone to camera frame
            cone_camera = transform_cone_to_camera(cone)

            # Skip if the transform failed
            if cone_camera is None:
                continue

            # Check if cone is in the camera's FOV and range
            if is_in_camera_view(cone_camera):
                marker = Marker()
                marker.header.stamp = rospy.Time.now()
                marker.header.frame_id = "camera_link"
                marker.ns = "detected_cones"
                marker.id = i
                marker.type = Marker.SPHERE
                marker.action = Marker.ADD
                marker.pose.position.x = cone_camera[0]
                marker.pose.position.y = cone_camera[1]
                marker.pose.position.z = cone_camera[2]
                marker.pose.orientation.x = 0.0
                marker.pose.orientation.y = 0.0
                marker.pose.orientation.z = 0.0
                marker.pose.orientation.w = 1.0  # Identity quaternion
                marker.scale.x = 0.3
                marker.scale.y = 0.3
                marker.scale.z = 0.2
                marker.color.r = 1.0  # Full red
                marker.color.g = 0.2  # Reduced green for reddish-orange
                marker.color.b = 0.0  # No blue
                marker.color.a = 1.0  # Full opacity
                detected_markers.markers.append(marker)

        # Publish the detected cones in the camera frame
        marker_array_publisher.publish(detected_markers)

    except tf2_ros.LookupException:
        rospy.logwarn("Transform not available yet")
    except tf2_ros.ExtrapolationException:
        rospy.logwarn("Extrapolation error while looking up transform")

if __name__ == '__main__':
    # Subscribe to odometry to get the robot's pose in the map frame
    rospy.Subscriber('/hakuroukun_pose/rear_wheel_odometry', Odometry, simulate_cone_detection)
    rospy.spin()
