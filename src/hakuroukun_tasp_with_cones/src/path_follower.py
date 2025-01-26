#!/usr/bin/env python3

import rospy
import math

from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float64MultiArray, Bool
from scipy.spatial.transform import Rotation

class PurePursuitNode:
    def __init__(self):
        """Initialize the pure pursuit controller node."""
        rospy.init_node('pure_pursuit_hakuroukun', anonymous=True)

        # ---------------------------
        #  Load Parameters from YAML
        # ---------------------------
        self.MAX_SPEED = rospy.get_param("pure_pursuit_hakuroukun/max_speed", 0.6)
        self.MIN_SPEED = rospy.get_param("pure_pursuit_hakuroukun/min_speed", 0.4)
        self.MAX_ACCEL = rospy.get_param("pure_pursuit_hakuroukun/max_accel", 2.5)
        self.MAX_STEERING = rospy.get_param("pure_pursuit_hakuroukun/max_steering", 0.78)
        self.MIN_STEERING = rospy.get_param("pure_pursuit_hakuroukun/min_steering", -0.78)
        self.lookahead_distance = rospy.get_param("pure_pursuit_hakuroukun/lookahead_distance", 0.8)
        self.wheelbase = rospy.get_param("pure_pursuit_hakuroukun/wheelbase", 1.1)
        self.control_rate = rospy.get_param("pure_pursuit_hakuroukun/control_rate", 1)

        # ---------------------------
        #  Internal States
        # ---------------------------
        self.current_pose = None    # (x, y, yaw)
        self.previous_speed = 0.0   # For acceleration limiting
        self.path_points = []       # List of (x, y) from Path
        self.path_available = False
        [self.velocity_cmd, self.steering_cmd] = [0,0]

        # ---------------------------
        #  Publishers & Subscribers
        # ---------------------------
        rospy.Subscriber('/hakuroukun_pose/rear_wheel_odometry', Odometry, self.odom_callback)
        rospy.Subscriber('/desired_path', Path, self.path_callback)
        rospy.Subscriber('/stop_signal', Bool, self.stop_signal_callback)

        self.cmd_pub = rospy.Publisher('/cmd_controller', Float64MultiArray, queue_size=10)

    def stop_signal_callback(self, msg):
        """Callback to stop the robot upon receiving a stop signal."""
        if msg.data:  # If the stop signal is True
            self.stop_robot()

    def odom_callback(self, msg):
        """Callback to update robot’s current pose and speed from Odometry."""
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y

        quat = (
            msg.pose.pose.orientation.x,
            msg.pose.pose.orientation.y,
            msg.pose.pose.orientation.z,
            msg.pose.pose.orientation.w
        )
        yaw = Rotation.from_quat(quat).as_euler("zyx", degrees=False)[0]
        self.current_pose = (x, y, yaw)

    def path_callback(self, msg):
        """Callback to store path points (x, y) from a nav_msgs/Path message."""
        self.path_points = []
        for pose_stamped in msg.poses:
            px = pose_stamped.pose.position.x
            py = pose_stamped.pose.position.y
            self.path_points.append((px, py))
        self.path_available = (len(self.path_points) > 0)

    def run(self):
        """Main loop: compute and publish control commands at a fixed rate."""
        rate = rospy.Rate(self.control_rate)
        while not rospy.is_shutdown():
            rospy.on_shutdown(self.stop_robot)
            if self.current_pose is not None and self.path_available:
                self.velocity_cmd, self.steering_cmd = self.compute_pure_pursuit()
                cmd_msg = Float64MultiArray()
                cmd_msg.data = [self.velocity_cmd, self.steering_cmd]
                self.cmd_pub.publish(cmd_msg)
            rate.sleep()

    def stop_robot(self):
        """Stop the robot by publishing zero velocity while keeping the last steering command."""
        rospy.loginfo("Shutting down: Stopping the robot.")
        cmd_msg = Float64MultiArray()
        cmd_msg.data = [0.0, self.steering_cmd]  # Zero velocity, maintain steering
        self.cmd_pub.publish(cmd_msg)

    def compute_pure_pursuit(self):
        x, y, yaw = self.current_pose
        lookahead_point = self.get_lookahead_point(x, y)

        if lookahead_point is None:
            print('stop because lookahead_point is None')
            return (0.0, 0.0)

        Ld_x, Ld_y = lookahead_point
        dx = Ld_x - x
        dy = Ld_y - y
        alpha = math.atan2(dy, dx) - yaw
        alpha = math.atan2(math.sin(alpha), math.cos(alpha))

        steering = math.atan2(2.0 * self.wheelbase * math.sin(alpha),
                              self.lookahead_distance)
        steering = max(min(steering, self.MAX_STEERING), self.MIN_STEERING)

        desired_speed = (self.MAX_SPEED - self.MIN_SPEED)* (1.0 - abs(steering) / self.MAX_STEERING) + self.MIN_SPEED

        dt = 1.0 / self.control_rate
        max_delta_v = self.MAX_ACCEL * dt
        speed_diff = desired_speed - self.previous_speed
        if abs(speed_diff) > max_delta_v:
            speed_diff = math.copysign(max_delta_v, speed_diff)
        new_speed = self.previous_speed + speed_diff
        new_speed = max(min(new_speed, self.MAX_SPEED), 0.0)

        self.previous_speed = new_speed
        return (new_speed, steering)

    def get_lookahead_point(self, robot_x, robot_y):
        for px, py in self.path_points:
            dist = math.hypot(px - robot_x, py - robot_y)
            if dist >= self.lookahead_distance:
                return (px, py)
        return None


if __name__ == '__main__':
    node = PurePursuitNode()
    node.run()
