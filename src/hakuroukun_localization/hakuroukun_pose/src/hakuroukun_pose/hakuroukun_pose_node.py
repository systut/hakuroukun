import math
from datetime import datetime
import pytz
import os

import rospy
from sensor_msgs.msg import NavSatFix
from sensor_msgs.msg import Imu
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float64
# from tf.transformations import euler_from_quaternion
import geonav_transform.geonav_conversions as gc
import tf.transformations as tf


class HakuroukunPose:

    # ==========================================================================
    # PUBLIC FUNCTION
    # ==========================================================================

    def __init__(self):

        self.log = True

        rospy.init_node("robot_localization", anonymous=True)

        self._register_parameters()

        self._register_log_file()

        self._get_initial_pose()

        self._get_initial_orientation()

        rospy.sleep(1)

        self._register_publishers()

        self._register_subscribers()

        rospy.sleep(1)

        self._register_timers()

    def run(self):
        """! Start ros node
        """
        rospy.spin()

    # ==========================================================================
    # PRIVATE FUNCTION
    # ==========================================================================
    def _register_parameters(self):
        """! Register parameters
        """
        self.publish_rate = 0.1

        self.orientation = 0

    def _register_subscribers(self):
        """! Register subscribers
        """
        self.gps_sub = rospy.Subscriber(
            "/fix", NavSatFix, self._gps_callback)

        self.imu_sub = rospy.Subscriber(
            "/imu/data_raw", Imu, self._imu_callback)

    def _register_publishers(self):
        """! Register publishers
        """
        self.rear_pose_pub = rospy.Publisher(
            "/hakuroukun_pose/rear_wheel_position", PoseStamped, queue_size=10)

        self.orientation_pub = rospy.Publisher(
            "/hakuroukun_pose/orientation", Float64, queue_size=1)

    def _register_timers(self):
        """! Register timers
        """
        rospy.Timer(rospy.Duration(self.publish_rate),
                    self._publish_rear_wheel_pose)

        if self.log:

            rospy.Timer(rospy.Duration(self.publish_rate),
                        self._log_pose)

    def _register_log_file(self):
        """! Register log file
        """
        current_folder = os.path.dirname(os.path.abspath(__file__))

        new_folder = os.path.join(current_folder, '..', '..', '..', 'log_data')

        new_folder = os.path.normpath(new_folder)

        japan_timezone = pytz.timezone('Asia/Tokyo')

        current_time = datetime.now(japan_timezone).strftime(
            "position_log_%Y%m%d_%H-%M")

        self.file_name = os.path.join(
            new_folder, current_time + ".csv")

    def _get_initial_pose(self):
        """! Get initial pose
        """
        first_gps_mess = rospy.wait_for_message(
            '/fix', NavSatFix, timeout=10)

        rospy.loginfo("GPS Data Received")

        self.initial_lat = first_gps_mess.latitude

        self.initial_lon = first_gps_mess.longitude

    def _get_initial_orientation(self):
        """! Get initial orientation
        """
        rospy.loginfo("IMU Data Received")

    def _gps_callback(self, data: NavSatFix):

        self.x_gps, self.y_gps = self._get_xy_from_latlon(
            data.latitude, data.longitude, self.initial_lat, self.initial_lon)

        gps_to_rear_axis = 0.6

        self.x_rear = self.x_gps - \
            gps_to_rear_axis * math.cos(self.orientation)

        self.y_rear = self.y_gps - \
            gps_to_rear_axis * math.sin(self.orientation)

    def _imu_callback(self, data: Imu):

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

        self.quad = [self.quaternion_x, self.quaternion_y,
                     self.quaternion_z, self.quaternion_w]

        self.euler = self._get_euler_from_quaternion(self.quad)

        self.yaw = self.euler[2] + 0.2526352784505572

    def _publish_rear_wheel_pose(self, timer):

        rear_wheel_msg = PoseStamped()
        rear_wheel_msg.header.stamp = rospy.get_rostime()
        rear_wheel_msg.pose.position.x = self.x_rear
        rear_wheel_msg.pose.position.y = self.y_rear
        rear_wheel_msg.pose.position.z = 0.0
        rear_wheel_msg.pose.orientation.x = self.quaternion_x
        rear_wheel_msg.pose.orientation.y = self.quaternion_y
        rear_wheel_msg.pose.orientation.z = self.quaternion_z
        rear_wheel_msg.pose.orientation.w = self.quaternion_w

        rospy.loginfo(
            f"x: {rear_wheel_msg.pose.position.x}, y: {rear_wheel_msg.pose.position.y}")

        self.rear_pose_pub.publish(rear_wheel_msg)

    def _log_pose(self, timer):

        pose = f"{self.x_rear}, {self.y_rear}, {(self.yaw)}" + "\n"

        with open(self.file_name, mode="a") as f:

            f.write(pose)

    def _get_xy_from_latlon(self, latitude, longitude, initial_lat, initial_lon):

        # What is this lol
        # This is a magic number - the angle between the GPS and the robot's
        rotation_angle = math.radians(-13.5255 - 6.3102 + 11.9329)

        x_gps, y_gps = gc.ll2xy(latitude, longitude, initial_lat, initial_lon)

        x_gps_local = x_gps*math.cos(rotation_angle) - \
            y_gps*math.sin(rotation_angle)

        y_gps_local = x_gps*math.sin(rotation_angle) + \
            y_gps*math.cos(rotation_angle)

        return x_gps_local, y_gps_local

    def _get_euler_from_quaternion(self, quad):

        quaternion = (quad[0], quad[1], quad[2], quad[3])

        euler = tf.euler_from_quaternion(quaternion)

        return euler

    def _integrate_yaw(self, current_orientation, angular_rate, dt):
        """! This function calculate the orientation of the robot using the
            angular rate from the IMU
        """

        current_orientation += (angular_rate+0.36) * dt

        return current_orientation
