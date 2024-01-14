import geonav_transform.geonav_conversions as gc
import alvinxy.alvinxy as axy
import math
import time
import os

import rospy
from sensor_msgs.msg import NavSatFix
from sensor_msgs.msg import Imu
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float64
from tf.transformations import euler_from_quaternion

class HakuroukunPose:

    # ==========================================================================
    # PUBLIC FUNCTION
    # ==========================================================================

    def __init__(self):

        self.file_name = os.getcwd() + "/../catkin_ws/src/data/tut_run/" + str(time.time()) + ".csv"

        rospy.init_node("robot_localization", anonymous=True)

        # Wait until got first (lon, lat, alt) from gps as origin
        first_gps_mess = rospy.wait_for_message('/fix', NavSatFix, timeout=10)
        rospy.loginfo("GPS Data Received")
        # Wait until got first quad from imu
        first_imu_mess = rospy.wait_for_message('/imu/data_raw', Imu, timeout=20)
        
        rospy.loginfo("IMU Data Received")

        self.lat0 = first_gps_mess.latitude        # First GPS Data
        
        self.lon0 = first_gps_mess.longitude        # First GPS Data

        self.dt = 0.01
        
        self.orientation = 90.0

        rospy.sleep(1)

        ## Pub - Sub

        self.gps_sub = rospy.Subscriber("/fix", NavSatFix, self._gps_callback) 

        self.imu_sub = rospy.Subscriber("/imu/data_raw", Imu, self._imu_callback)

        self.pose_pub = rospy.Publisher("/hakuroukun_pose/pose", PoseStamped, queue_size=10)

        self.orientation_pub = rospy.Publisher("/hakuroukun_pose/orientation", Float64, queue_size=1)
        
        rospy.sleep(1)

        rospy.Timer(rospy.Duration(0.03), self._publish_pose)

        rospy.Timer(rospy.Duration(0.03), self._publish_orientation)

        # rospy.Timer(rospy.Duration(0.03), self._log_pose)
        
    def run(self):
        """! Start ros node
        """

        rospy.spin()

    # ==========================================================================
    # PRIVATE FUNCTION
    # ==========================================================================

    def _get_xy_from_lat_lon(self, lat,lon):

        x,y = gc.ll2xy(lat, lon, self.lat0, self.lon0)

        rotation_angle = math.radians(-8.6732)

        current_pose_x = x*math.cos(rotation_angle) - y*math.sin(rotation_angle)

        current_pose_y = x*math.sin(rotation_angle) + y*math.cos(rotation_angle)

        return current_pose_x, current_pose_y

    def _get_euler_from_quaternion(self, quad):
        
        quaternion = (quad[0], quad[1], quad[2], quad[3])

        euler = euler_from_quaternion(quaternion)

        return euler
    
    def _get_initial_offset(self, yaw):

        if yaw <= math.pi:

            offset = -yaw

        else:

            offset = (2 * math.pi - yaw)

        return offset
    
    def _integrate_yaw(self, current_orientation, angular_rate, dt):

        # a = str(round(time.time() * 1000))

        # rospy.loginfo(a)

        current_orientation += (angular_rate+0.36) * dt  # There was an offset with 0.36 deegree when stood still

        return current_orientation

    # ====================================
    # Callback Functions
    # ====================================

    def _gps_callback(self, data):

        self.current_lat = data.latitude

        self.current_lon = data.longitude

        self.current_pose_x, self.current_pose_y = self._get_xy_from_lat_lon(self.current_lat, self.current_lon)

        # rospy.loginfo(f"%f, %f", self.current_pose_x, self.current_pose_y)

    def _imu_callback(self, data):

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

        self.quad = [self.quaternion_x, self.quaternion_y, self.quaternion_z, self.quaternion_w]

        self.euler = self._get_euler_from_quaternion(self.quad)

        self.yaw = self.euler[2] + 0.139626

        if self.yaw < 0:

            self.yaw = self.yaw + math.pi * 2

        self.orientation = self._integrate_yaw(self.orientation, self.angular_velocity_z, self.dt)

    def _log_pose(self, timer):

        # rospy.loginfo(f"{self.current_pose_x}, {self.current_pose_y}, {(self.yaw)}")

        buf = f"{self.current_pose_x}, {self.current_pose_y}, {(self.yaw)}" + "\n"

        with open(self.file_name, mode = "a") as f:

            f.write(buf)

    # ====================================
    # Publish Function
    # ====================================
            
    def _publish_pose(self, timer):

        now = rospy.get_rostime()

        pose_msg = PoseStamped()
        pose_msg.header.stamp = now
        pose_msg.pose.position.x = self.current_pose_x
        pose_msg.pose.position.y = self.current_pose_y
        pose_msg.pose.position.z = 0.0
        pose_msg.pose.orientation.x = self.quaternion_x
        pose_msg.pose.orientation.y = self.quaternion_y
        pose_msg.pose.orientation.z = self.quaternion_z
        pose_msg.pose.orientation.w = self.quaternion_w

        self.pose_pub.publish(pose_msg)

    def _publish_orientation(self, timer):

        orientation_msg = Float64()

        # orientation_msg.data = float(math.degrees(self.yaw))

        orientation_msg.data = float(self.orientation)

        self.orientation_pub.publish(orientation_msg)