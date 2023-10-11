from tf.transformations import euler_from_quaternion
import geonav_transform.geonav_conversions as gc
import alvinxy.alvinxy as axy
import math

import rospy
from sensor_msgs.msg import NavSatFix
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Pose, Vector3

class HakuroukunPose:

    # ==========================================================================
    # PUBLIC FUNCTION
    # ==========================================================================

    def __init__(self):

        rospy.init_node("robot_localization", anonymous=True)

        ## Wait until got first (lat,lat,alt) from gps as origin
        first_gps_mess = rospy.wait_for_message('/fix', NavSatFix, timeout=5)
        self.lat0 = first_gps_mess.latitude
        self.lon0 = first_gps_mess.longitude

        rospy.sleep(5)

        ## Pub - Sub
        self.gps_sub = rospy.Subscriber("/fix", NavSatFix, self._gps_callback) 

        self.imu_sub = rospy.Subscriber("/imu/data_raw", Imu, self._imu_callback) 

        self.pose_pub = rospy.Publisher("/robot_pose", Vector3, queue_size=10)
        
        rospy.Timer(rospy.Duration(0.05), self._publish_pose)
        
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

        self.current_pose_x, self.current_pose_y = gc.ll2xy(lat,lon,self.lat0,self.lon0)

    def _get_euler_from_quaternion(self):
        
        quaternion_ = [self.quaternion_x, self.quaternion_y, self.quaternion_z, self.quaternion_w]
        
        (self.roll, self.pitch, self.yaw) = euler_from_quaternion(quaternion_)

    def _publish_pose(self, timer):

        pose_msg = Vector3()
        pose_msg.x = self.current_pose_x
        pose_msg.y = self.current_pose_y
        pose_msg.z = math.degrees(self.yaw)

        self.pose_pub.publish(pose_msg)




    