#!/usr/bin/env python3

import rospy
import math
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry, OccupancyGrid, Path
from geometry_msgs.msg import Twist, TransformStamped, PoseStamped
from visualization_msgs.msg import Marker
from tf2_ros import StaticTransformBroadcaster
from tf.transformations import euler_from_quaternion
from tasp_path_planner import TASPPathPlanner
from update_map import UpdateMap
from simple_astar import SimpleOccupancyGrid, astar_plan
from visualizer import TASPVisualizer

class TASPNode:
    def __init__(self):
        rospy.init_node("tasp_with_cones")

        # Initialize global variables
        self.start_pose = [0, 0, 0.0]
        self.current_pose = self.start_pose
        self.inflated_grid_msg = None
        self.current_tasp_goal = None
        self.current_path = []

        # Camera parameters
        self.CAMERA_RANGE = 4 / math.cos(math.radians(35))
        self.CAMERA_FOV = math.radians(70)

        # Map and path planner
        self.map_handler = UpdateMap(
            resolution=0.05,
            width=1000,
            height=1000,
            origin=(-25, -25),
            camera_fov=self.CAMERA_FOV,
            camera_range=self.CAMERA_RANGE
        )
        self.tasp_planner = TASPPathPlanner(tasp_cell_size=1)
        self.vis = TASPVisualizer(frame_id="map")

        # Subscribers
        rospy.Subscriber('/hakuroukun_pose/rear_wheel_odometry', Odometry, self.odom_callback)
        rospy.Subscriber('/my_costmap_node/costmap/costmap', OccupancyGrid, self.costmap_callback)
        rospy.Subscriber('/scan_multi', LaserScan, self.lidar_callback)

        # Publishers
        self.map_pub = rospy.Publisher('/map', OccupancyGrid, queue_size=10)
        self.fov_pub = rospy.Publisher('/camera_fov', Marker, queue_size=10)
        self.path_pub = rospy.Publisher('/desired_path', Path, queue_size=10)

        # Static Transform
        self.setup_static_transform()

        # Main loop rate
        self.rate = rospy.Rate(2)
        self.start_time = rospy.Time.now()
        self.wait_duration = rospy.Duration(5.0)

    def setup_static_transform(self):
        broadcaster = StaticTransformBroadcaster()
        static_transform = TransformStamped()
        static_transform.header.stamp = rospy.Time.now()
        static_transform.header.frame_id = "map"
        static_transform.child_frame_id = "odom"
        static_transform.transform.translation.x = 0.0
        static_transform.transform.translation.y = 0.0
        static_transform.transform.rotation.w = 1.0
        broadcaster.sendTransform(static_transform)

    def odom_callback(self, data):
        position = data.pose.pose.position
        orientation = data.pose.pose.orientation
        _, _, yaw = euler_from_quaternion([orientation.x, orientation.y, orientation.z, orientation.w])
        self.current_pose = [position.x, position.y, yaw]

    def costmap_callback(self, msg):
        self.inflated_grid_msg = msg

    def lidar_callback(self, scan):
        self.map_handler.update_map_with_scan(scan)
        self.map_handler.publish_map(self.map_pub, self.fov_pub)

    def robot_has_reached_tasp_goal(self, tolerance=0.8):
        if self.current_tasp_goal is None:
            return False
        dx = self.current_tasp_goal[0] - self.current_pose[0]
        dy = self.current_tasp_goal[1] - self.current_pose[1]
        return math.hypot(dx, dy) <= tolerance

    def plan_path(self, occupancy_grid_msg, start, goal):
        width, height, resolution = occupancy_grid_msg.info.width, occupancy_grid_msg.info.height, occupancy_grid_msg.info.resolution
        origin_x, origin_y = occupancy_grid_msg.info.origin.position.x, occupancy_grid_msg.info.origin.position.y
        data = occupancy_grid_msg.data

        ogrid = SimpleOccupancyGrid(width, height, resolution, origin_x, origin_y, data)
        return astar_plan(ogrid, start[0], start[1], goal[0], goal[1], connectivity=8)

    def publish_astar_path(self, path_points):
        path_msg = Path()
        path_msg.header.frame_id = "map"
        path_msg.header.stamp = rospy.Time.now()
        for (px, py) in path_points:
            pose_stamped = PoseStamped()
            pose_stamped.header.frame_id = "map"
            pose_stamped.header.stamp = rospy.Time.now()
            pose_stamped.pose.position.x = px
            pose_stamped.pose.position.y = py
            pose_stamped.pose.orientation.w = 1.0
            path_msg.poses.append(pose_stamped)
        self.path_pub.publish(path_msg)

    def run(self):
        while not rospy.is_shutdown():
            self.vis.publish_start(self.start_pose)
            if rospy.Time.now() - self.start_time < self.wait_duration:
                self.rate.sleep()
                continue

            if self.inflated_grid_msg is None:
                rospy.logwarn_throttle(5, "Waiting for inflated costmap data...")
                self.rate.sleep()
                continue

            if self.robot_has_reached_tasp_goal():
                new_goal = self.tasp_planner.tasp_path_planning(
                    current_pose=self.current_pose,
                    unknown_map=self.inflated_grid_msg,
                    start_pose=self.start_pose
                )
                self.vis.publish_btp(self.tasp_planner.BTP)
                if new_goal:
                    rospy.loginfo(f"[TASP] New goal: {new_goal}")
                    self.current_tasp_goal = new_goal
                    self.vis.publish_goal(new_goal)

            if self.current_tasp_goal:
                self.current_path = self.plan_path(self.inflated_grid_msg, self.current_pose, self.current_tasp_goal)
                if not self.current_path:
                    rospy.logwarn("[TASP] No path found to the TASP goal.")
                else:
                    self.publish_astar_path(self.current_path)
                    self.vis.publish_astar_path(self.current_path)

            self.rate.sleep()


if __name__ == "__main__":
    node = TASPNode()
    node.run()
