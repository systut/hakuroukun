#!/usr/bin/env python3

import rospy
import math
import numpy as np

from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float64MultiArray
from scipy.spatial.transform import Rotation

class PurePursuitNode:
    def __init__(self):
        """Initialize the pure pursuit controller node."""
        rospy.init_node('pure_pursuit_hakuroukun', anonymous=True)

        # ---------------------------
        #  User-defined Parameters
        # ---------------------------
        # Limits
        self.MAX_SPEED = 0.45         # [m/s]
        self.MAX_ACCEL = 2.0          # [m/s^2]
        self.MAX_STEERING = 0.87      # [rad]
        self.MIN_STEERING = -0.87     # [rad]

        # Pure Pursuit Params
        self.lookahead_distance = 0.8 # [m] - Adjust as needed
        self.wheelbase = 1.1         # [m] - Robot’s wheelbase

        # Node Loop Rate
        self.control_rate = 1  # [Hz]

        # ---------------------------
        #  Internal States
        # ---------------------------
        self.current_pose = None    # (x, y, yaw)
        self.current_speed = 0.0    # Current linear speed estimate
        self.previous_speed = 0.0   # For acceleration limiting
        self.path_points = []       # List of (x, y) from Path
        self.path_available = False

        # ---------------------------
        #  Publishers & Subscribers
        # ---------------------------
        # Subscriber for odometry
        rospy.Subscriber('/hakuroukun_pose/rear_wheel_odometry', Odometry,
                         self.odom_callback)

        # Subscriber for desired path (e.g., from an A* planner)
        rospy.Subscriber('/desired_path', Path, self.path_callback)

        # Publisher for control commands
        self.cmd_pub = rospy.Publisher(
            '/cmd_controller',
            Float64MultiArray,
            queue_size=10
        )

    def odom_callback(self, msg):
        """Callback to update robot’s current pose and speed from Odometry."""
        # Extract position
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y

        # Extract yaw from quaternion
        quat = (
            msg.pose.pose.orientation.x,
            msg.pose.pose.orientation.y,
            msg.pose.pose.orientation.z,
            msg.pose.pose.orientation.w
        )
        yaw = Rotation.from_quat(quat).as_euler("zyx", degrees=False)[0]

        # Estimate speed from the linear velocity in Odometry
        vx = msg.twist.twist.linear.x
        vy = msg.twist.twist.linear.y
        speed = math.sqrt(vx**2 + vy**2)

        self.current_pose = (x, y, yaw)
        self.current_speed = speed

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
            # Only compute control if we have a pose and a path
            if self.current_pose is not None and self.path_available:
                velocity_cmd, steering_cmd = self.compute_pure_pursuit()
                # Publish commands
                cmd_msg = Float64MultiArray()
                cmd_msg.data = [velocity_cmd, steering_cmd]
                self.cmd_pub.publish(cmd_msg)
            rate.sleep()

    def compute_pure_pursuit(self):
        """
        Calculate velocity and steering using pure pursuit.
        
        Returns:
            (v, delta): (float, float)
                v     = linear velocity command [m/s]
                delta = steering angle command [rad]
        """
        x, y, yaw = self.current_pose

        # 1. Find the lookahead target on path
        lookahead_point = self.get_lookahead_point(x, y)

        if lookahead_point is None:
            # If no valid lookahead point found, stop
            return (0.0, 0.0)

        # 2. Transform lookahead point to local coordinates
        Ld_x, Ld_y = lookahead_point
        dx = Ld_x - x
        dy = Ld_y - y

        # Robot’s heading = yaw, transform to robot frame
        # but we only need the angle "alpha"
        # alpha = difference between robot’s heading and line to the lookahead
        # E.g., using atan2(dy, dx) - yaw
        alpha = math.atan2(dy, dx) - yaw

        # Normalize alpha to [-pi, pi]
        alpha = math.atan2(math.sin(alpha), math.cos(alpha))

        # 3. Compute steering using pure pursuit formula:
        #   delta = atan(2 * L * sin(alpha) / Ld)
        #   where L is wheelbase, Ld is lookahead_distance
        steering = math.atan2(2.0 * self.wheelbase * math.sin(alpha),
                              self.lookahead_distance)

        # 4. Limit steering to the robot’s physical steering angle range
        steering = max(min(steering, self.MAX_STEERING), self.MIN_STEERING)

        # 5. Compute or set a desired speed
        #    For simplicity, choose a constant desired speed = self.MAX_SPEED
        #    Then limit acceleration.
        desired_speed = self.MAX_SPEED

        # Acceleration limiting
        dt = 1.0 / self.control_rate
        max_delta_v = self.MAX_ACCEL * dt
        speed_diff = desired_speed - self.previous_speed
        # Clamp speed difference
        if abs(speed_diff) > max_delta_v:
            speed_diff = math.copysign(max_delta_v, speed_diff)
        new_speed = self.previous_speed + speed_diff

        # Final clamp for speed
        new_speed = max(min(new_speed, self.MAX_SPEED), 0.0)

        self.previous_speed = new_speed
        return (new_speed, steering)

    def get_lookahead_point(self, robot_x, robot_y):
        """
        Find the first point on the path at least 'lookahead_distance' away
        from the robot. Return (x, y) or None if no point is found.
        """
        for px, py in self.path_points:
            dist = math.hypot(px - robot_x, py - robot_y)
            if dist >= self.lookahead_distance:
                return (px, py)
        # If we reach here, all points are closer than Ld
        # e.g., the robot might be near the path end
        return None


if __name__ == '__main__':
    node = PurePursuitNode()
    node.run()
