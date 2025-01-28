#!usr/bin/env python3
##
# @file hakuroukun_pose.py
#
# @brief Provide implementation of Hakuroukun pose node.
#
# @section author_doxygen_example Author(s)
# - Created by Dinh Ngoc Duc on 24/10/2024.
#
# Copyright (c) 2024 System Engineering Laboratory.  All rights reserved.

# Standard Libraries
import os
import math
import time
from datetime import datetime

# External Libraries
import tf
import rospy
import pytz
from std_msgs.msg import Float64
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu, NavSatFix

# Internal Libraries
import geonav_transform.geonav_conversions as gc


class HakuroukunPose:
    """! HakuroukunPose class
    The class provides implementation of Hakuroukun pose node.
    """
    # ==========================================================================
    # PUBLIC METHODS
    # ==========================================================================

    def __init__(self):
        """! Constructor
        """
        super(HakuroukunPose, self).__init__()

        rospy.init_node("robot_localization", anonymous=True)

        self._yaw = 0.0  # Initialize yaw to a default value (e.g., 0.0 radians)
        
        self._register_parameters()

        self._get_initial_orientation()

        self._get_initial_pose()

        rospy.sleep(3)

        self._register_publishers()

        self._register_subscribers()

        self._register_log_data()

        self._register_timers()

        self.previous_yaw = self._imu_offset

    def run(self):
        """! Start ros node
        """
        rospy.spin()

    # ==========================================================================
    # PRIVATE METHODS
    # ==========================================================================
    def _register_parameters(self):
        """! Register ROS parameters method
        """
        self._log = rospy.get_param(
            "~log", True)

        self._publish_rate = rospy.get_param(
            "~publish_rate", 0.1)

        self._gps_to_rear_axis = rospy.get_param(
            "~gps_to_rear_axis", 0.6)

        self._imu_mode = rospy.get_param(
            "~imu_mode", "quaternion")

        self._imu_epsilon = rospy.get_param(
            "~imu_epsilon", 0.0001)

        self._imu_calibration_threshold = rospy.get_param(
            "~imu_calibration_threshold", 200)

    def _register_subscribers(self):
        """! Register ROS subscribers method
        """
        self._gps_sub = rospy.Subscriber(
            "/fix", NavSatFix, self._gps_callback)

        self._imu_sub = rospy.Subscriber(
            "/imu/data_raw", Imu, self._imu_callback)

    def _register_publishers(self):
        """! Register publishers method
        """
        self._rear_odom_pub = rospy.Publisher(
            "/hakuroukun_pose/rear_wheel_odometry", Odometry, queue_size=10)

        self._tf_broadcaster = tf.TransformBroadcaster()

    def _register_timers(self):
        """! Register timers method
        This method register the timer for publishing localization data
        with publish rate
        """
        rospy.Timer(rospy.Duration(self._publish_rate),
                    self._publish_rear_wheel_odometry)

        if self._log:

            rospy.Timer(rospy.Duration(self._publish_rate),
                        self._log_pose)

    def _register_log_data(self):
        """! Register log localization data method
        """
        self._log_start_time = None

        log_folder = rospy.get_param("~log_folder", None)

        current_time = datetime.now(pytz.timezone('Asia/Tokyo')).strftime(
            "position_log_%Y%m%d_%H-%M")

        self._file_name = os.path.join(
            log_folder, current_time + ".csv")

        with open(self._file_name, mode="a") as f:

            title = "Time (s), x_rear(m), y_rear(m), yaw(deg)\n"

            f.write(title)

    def _get_initial_pose(self):
        """! Get initial pose method
        This method will guarantee that data from GPS is received before
        the robot start moving
        """
        first_gps_mess = rospy.wait_for_message(
            '/fix', NavSatFix, timeout=10)

        rospy.loginfo("GPS Data Received")

        self._initial_lat = first_gps_mess.latitude

        self._initial_lon = first_gps_mess.longitude

    def _get_initial_orientation(self):
        """! Get initial orientation
        THis method will guarantee that data from IMU is received before
        the robot start moving
        """

        if self._imu_mode == "quaternion":

            start_time = time.time()

            imu_data = []

            subtracted_values = []

            while not rospy.is_shutdown() and (time.time() - start_time < 30):
                try:
                    data = rospy.wait_for_message(
                        "/imu/data_raw", Imu, timeout=3.0)

                    euler = tf.transformations.euler_from_quaternion(
                        [data.orientation.x,
                        data.orientation.y,
                        data.orientation.z,
                        data.orientation.w])

                    imu_data.append(euler[2])

                    if len(imu_data) > 1:
                        difference = imu_data[-1] - imu_data[-2]

                        subtracted_values.append(difference)

                        if len(subtracted_values) > self._imu_calibration_threshold:
                            subtracted_values.pop(0)

                        if len(subtracted_values) == self._imu_calibration_threshold and all(val < self._imu_epsilon for val in subtracted_values):
                            rospy.loginfo(f"Breaking out: last {self._imu_calibration_threshold} differences are zero.")

                            self._imu_offset = euler[2]
                            print('_imu_offset',self._imu_offset)

                            break

                    rospy.loginfo("Calibrating IMU ...")

                except rospy.ROSException:
                    rospy.logwarn("No IMU message received within timeout.")

        else:
            rospy.wait_for_message("/imu/data_raw", Imu, timeout=10)

            self._yaw = 0.0

        rospy.loginfo("IMU data received.")

    def _gps_callback(self, data: NavSatFix):
        """! GPS callback method
        @param data: NavSatFix message
        @return: x_gps, y_gps, x_rear, y_rear
        @ x_gps: x position of the gps in the global frame
        @ y_gps: y position of the gps in the global frame
        @ x_rear: x position of the rear wheel in the global frame
        @ y_rear: y position of the rear wheel in the global frame
        """
        self._x_gps, self._y_gps = self._get_xy_from_latlon(
            data.latitude, data.longitude,
            self._initial_lat, self._initial_lon)

        self._x_rear = self._x_gps - self._gps_to_rear_axis * \
            math.cos(self._yaw)

        self._y_rear = self._y_gps - self._gps_to_rear_axis * \
            math.sin(self._yaw)

    def _imu_callback(self, data: Imu):
        """! IMU callback method
        @param data: Imu message
        @return: yaw
        @ yaw: The yaw angle of the robot
        """
        self.quaternion_x = data.orientation.x
        self.quaternion_y = data.orientation.y
        self.quaternion_z = data.orientation.z
        self.quaternion_w = data.orientation.w

        self.angular_velocity_x = data.angular_velocity.x
        self.angular_velocity_y = data.angular_velocity.y
        self.angular_velocity_z = data.angular_velocity.z

        self.linear_acceleration_x = data.linear_acceleration.x
        self.linear_acceleration_y = data.linear_acceleration.y
        self.linear_acceleration_z = data.linear_acceleration.z

        if self._imu_mode == "quaternion":

            self.euler = tf.transformations.euler_from_quaternion(
                [self.quaternion_x,
                self.quaternion_y,
                self.quaternion_z,
                self.quaternion_w])

            new_yaw = self.euler[2] - self._imu_offset
            new_yaw = math.atan2(math.sin(new_yaw), math.cos(new_yaw))  # Normalize yaw

        else:
            new_yaw = self._integrate_yaw(self._yaw, self.angular_velocity_z, 0.01)

        # Filter noise using a threshold
        threshold = 0.1  # Adjust this value based on your noise tolerance

        if hasattr(self, 'previous_yaw'):  # Check if previous_yaw exists
            yaw_change = abs(new_yaw - self.previous_yaw)
            if yaw_change > math.pi:  # Handle wrap-around
                yaw_change = abs(yaw_change - 2 * math.pi)

            if yaw_change < threshold:  # Only update yaw if change is below threshold
                self._yaw = new_yaw
            else:
                # Ignore the update if the change is too large (noise detected)
                pass
        else:
            # Initialize previous_yaw for the first callback
            self._yaw = new_yaw

        # Update previous_yaw
        self.previous_yaw = self._yaw

        (self.quaternion_x, self.quaternion_y,
        self.quaternion_z, self.quaternion_w) = tf.transformations.quaternion_from_euler(0, 0, self._yaw)

    def _publish_rear_wheel_odometry(self, timer):
        """! Publish rear wheel pose method
        @param timer: Timer (unused)
        """
        if not hasattr(self, '_x_rear') or not hasattr(self, '_y_rear'):
            rospy.logwarn("GPS data not yet received; skipping odometry publish.")
            return
        rear_odom_msg = Odometry()
        rear_odom_msg.header.stamp = rospy.get_rostime()
        rear_odom_msg.header.frame_id = "odom"
        rear_odom_msg.child_frame_id = "base_link"

        rear_odom_msg.pose.pose.position.x = self._x_rear
        rear_odom_msg.pose.pose.position.y = self._y_rear
        rear_odom_msg.pose.pose.position.z = 0.0
        rear_odom_msg.pose.pose.orientation.x = self.quaternion_x
        rear_odom_msg.pose.pose.orientation.y = self.quaternion_y
        rear_odom_msg.pose.pose.orientation.z = self.quaternion_z
        rear_odom_msg.pose.pose.orientation.w = self.quaternion_w

        rear_odom_msg.twist.twist.angular.x = self.angular_velocity_x
        rear_odom_msg.twist.twist.angular.y = self.angular_velocity_y
        rear_odom_msg.twist.twist.angular.z = self.angular_velocity_z

        self._rear_odom_pub.publish(rear_odom_msg)

        self._tf_broadcaster.sendTransform(
            (self._x_rear, self._y_rear, 0.0),  # Translation
            (self.quaternion_x, self.quaternion_y, self.quaternion_z, self.quaternion_w),  # Rotation (quaternion)
            rospy.Time.now(),  # Timestamp
            "base_link",  # Child frame
            "odom"         # Parent frame
        )

    def _log_pose(self, timer):
        """! Log pose method
        @param timer: Timer (unused)
        """
        if self._log_start_time is None:
            self._log_start_time = time.time()

        elapsed_time = (time.time() - self._log_start_time)

        pose = f"{elapsed_time}, {self._x_rear}, {self._y_rear}, {math.degrees(self._yaw)}"

        # pose = (f"{elapsed_time}, {self._x_rear}, {self._y_rear}, "
        rospy.loginfo(f"Pose: {pose}")
        rospy.loginfo(f"Pose: {pose}")

        with open(self._file_name, mode="a") as f:

            f.write(pose + "\n")

    def _get_xy_from_latlon(self, lat, long, _initial_lat, _initial_lon):
        """! Get x, y from latitude and longitude method
        @param latitude: Latitude of the robot
        @param longitude: Longitude of the robot
        @param _initial_lat: Initial latitude
        @param _initial_lon: Initial longitude

        @return: x_gps_local, y_gps_local
        @ x_gps_local: x position of the gps in the local frame
        @ y_gps_local: y position of the gps in the local frame
        """
        rotation_angle = math.radians(rospy.get_param("~rotation_angle", 0.0))

        x_gps, y_gps = gc.ll2xy(lat, long, _initial_lat, _initial_lon)

        x_gps_local = x_gps * math.cos(rotation_angle) - y_gps * math.sin(
            rotation_angle) + self._gps_to_rear_axis * math.cos(self._yaw)

        y_gps_local = x_gps * math.sin(rotation_angle) + y_gps * math.cos(
            rotation_angle) + self._gps_to_rear_axis * math.sin(self._yaw)

        return x_gps_local, y_gps_local

    @staticmethod
    def _integrate_yaw(current_orientation, angular_rate, dt):
        """! This function calculate yaw angle with angular velocity
        """
        current_orientation += (angular_rate) * dt

        return current_orientation
