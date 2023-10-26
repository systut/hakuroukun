from tf.transformations import euler_from_quaternion
import geonav_transform.geonav_conversions as gc
import alvinxy.alvinxy as axy
import math
import time

import rospy
from sensor_msgs.msg import NavSatFix
from sensor_msgs.msg import Imu
from geometry_msgs.msg import PoseStamped

class HakuroukunPose:

    # ==========================================================================
    # PUBLIC FUNCTION
    # ==========================================================================

    def __init__(self):

        rospy.init_node("robot_localization", anonymous=True)

        ## Wait until got first (lat,lat,alt) from gps as origin
        first_gps_mess = rospy.wait_for_message('/fix', NavSatFix, timeout=10)

        self.lat0 = first_gps_mess.latitude
        self.lon0 = first_gps_mess.longitude

        rospy.sleep(5)

        ## Pub - Sub
        self.gps_sub = rospy.Subscriber("/fix", NavSatFix, self._gps_callback) 

        self.imu_sub = rospy.Subscriber("/imu/data_raw", Imu, self._imu_callback) 

        self.pose_pub = rospy.Publisher("/hakuroukun_pose/pose", PoseStamped, queue_size=10)
        
        rospy.Timer(rospy.Duration(0.1), self._publish_pose)
        
    def run(self):
        """! Start ros node
        """
        rospy.spin()

    # ==========================================================================
    # PRIVATE FUNCTION
    # ==========================================================================

    def _gps_callback(self, data):

        self.current_lat = data.latitude

        self.current_lon = data.longitude

        self._get_xy_from_lat_lon(self.current_lat, self.current_lon)

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

        self._get_euler_from_quaternion()

    ## PRIVATE FUNCTION

    def _get_xy_from_lat_lon(self, lat,lon):

        x,y = gc.ll2xy(lat,lon,self.lat0,self.lon0)

        # print(x,y)

        self.current_pose_x = x*math.cos(90) + y*math.sin(90)

        self.current_pose_y = -x*math.sin(90) + y*math.cos(90)


    def _get_euler_from_quaternion(self):
        
        quaternion_ = [self.quaternion_x, self.quaternion_y, self.quaternion_z, self.quaternion_w]
        
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
