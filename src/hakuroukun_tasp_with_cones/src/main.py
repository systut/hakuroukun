#!/usr/bin/env python3
import sys
import rospy
import math
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry, OccupancyGrid, Path
from geometry_msgs.msg import TransformStamped, PoseStamped
from visualization_msgs.msg import Marker
from tf2_ros import StaticTransformBroadcaster
from tf.transformations import euler_from_quaternion
from std_msgs.msg import Bool

# Import your TASPPathPlanner
from tasp_path_planner import TASPPathPlanner
# Import your map handler
from update_map import UpdateMap
# Path-planning and motion-control functions
from simple_astar import SimpleOccupancyGrid, astar_plan
# Visualization
from visualizer import TASPVisualizer

# Global variables
current_pose = None       # Current pose of the robot
inflated_grid_msg = None  # Latest inflated costmap

def send_stop_signal():
    """Send a True stop signal to the PurePursuitNode."""
    rospy.loginfo("Sending stop signal to stop the robot...")
    stop_msg = Bool()
    stop_msg.data = True  # Send True to stop the robot
    stop_signal_pub.publish(stop_msg)

def odom_callback(data):
    global current_pose
    position = data.pose.pose.position
    orientation = data.pose.pose.orientation
    _, _, yaw = euler_from_quaternion([orientation.x, orientation.y, orientation.z, orientation.w])
    current_pose = [position.x, position.y, yaw]


def costmap_callback(msg):
    global inflated_grid_msg
    inflated_grid_msg = msg


def lidar_callback(scan):
    global current_pose
    map_handler.update_map_with_scan(scan)
    map_handler.publish_map(map_pub, fov_pub)


def setup_static_transform():
    broadcaster = StaticTransformBroadcaster()
    static_transform = TransformStamped()

    static_transform.header.stamp = rospy.Time.now()
    static_transform.header.frame_id = "map"
    static_transform.child_frame_id = "odom"
    static_transform.transform.translation.x = 0.0
    static_transform.transform.translation.y = 0.0
    static_transform.transform.translation.z = 0.0
    static_transform.transform.rotation.x = 0.0
    static_transform.transform.rotation.y = 0.0
    static_transform.transform.rotation.z = 0.0
    static_transform.transform.rotation.w = 1.0

    broadcaster.sendTransform(static_transform)


def robot_has_reached_goal(current_pose, goal, tolerance):
    dx = goal[0] - current_pose[0]
    dy = goal[1] - current_pose[1]
    dist = math.hypot(dx, dy)
    return dist <= tolerance


def plan_path(occupancy_grid_msg, start, goal):
    width = occupancy_grid_msg.info.width
    height = occupancy_grid_msg.info.height
    resolution = occupancy_grid_msg.info.resolution
    origin_x = occupancy_grid_msg.info.origin.position.x
    origin_y = occupancy_grid_msg.info.origin.position.y
    data = occupancy_grid_msg.data

    ogrid = SimpleOccupancyGrid(width, height, resolution, origin_x, origin_y, data)
    return astar_plan(ogrid, start[0], start[1], goal[0], goal[1], connectivity=8)


def publish_path(path_points):
    path_msg = Path()
    path_msg.header.frame_id = "map"
    path_msg.header.stamp = rospy.Time.now()

    for px, py in path_points:
        pose_stamped = PoseStamped()
        pose_stamped.header.frame_id = "map"
        pose_stamped.header.stamp = rospy.Time.now()
        pose_stamped.pose.position.x = px
        pose_stamped.pose.position.y = py
        pose_stamped.pose.orientation.w = 1.0
        path_msg.poses.append(pose_stamped)

    path_pub.publish(path_msg)


if __name__ == "__main__":
    rospy.init_node("tasp_with_cones")

    # Load parameters from the parameter server
    start_pose = rospy.get_param("start_pose", [0, 0, 0.0])
    camera_fov = math.radians(rospy.get_param("camera_fov", 70))
    camera_range = rospy.get_param("camera_range", 4) / math.cos(camera_fov / 2)
    map_resolution = rospy.get_param("map_resolution", 0.05)
    map_width = rospy.get_param("map_width", 1000)
    map_height = rospy.get_param("map_height", 1000)
    map_origin = tuple(rospy.get_param("map_origin", [-25, -25]))
    tasp_cell_size = rospy.get_param("tasp_cell_size", 1)
    tasp_goal_tolerance = rospy.get_param("tasp_goal_tolerance", 1)
    wait_duration = rospy.Duration(rospy.get_param("wait_duration", 5.0))
    rate_hz = rospy.get_param("rate_hz", 2)

    # Initialize map handler
    map_handler = UpdateMap(
        resolution=map_resolution,
        width=map_width,
        height=map_height,
        origin=map_origin,
        camera_fov=camera_fov,
        camera_range=camera_range,
    )
    tasp_planner = TASPPathPlanner()
    vis = TASPVisualizer(frame_id="map")

    setup_static_transform()

    # Define topics in the script
    rospy.Subscriber("/hakuroukun_pose/rear_wheel_odometry", Odometry, odom_callback)
    rospy.Subscriber("/my_costmap_node/costmap/costmap", OccupancyGrid, costmap_callback)
    rospy.Subscriber("/scan_multi", LaserScan, lidar_callback)

    map_pub = rospy.Publisher("/map", OccupancyGrid, queue_size=10)
    fov_pub = rospy.Publisher("/camera_fov", Marker, queue_size=10)
    path_pub = rospy.Publisher("/desired_path", Path, queue_size=10)
    stop_signal_pub = rospy.Publisher('/stop_signal', Bool, queue_size=10)

    rate = rospy.Rate(rate_hz)
    start_time = rospy.Time.now()

    current_goal = None
    current_path = []

    while not rospy.is_shutdown():
        vis.publish_start(start_pose)

        if rospy.Time.now() - start_time < wait_duration:
            rate.sleep()
            continue

        if inflated_grid_msg is None:
            rospy.logwarn_throttle(5, "Waiting for inflated costmap data...")
            rate.sleep()
            continue

        if current_goal is None or robot_has_reached_goal(current_pose, current_goal, tasp_goal_tolerance):
            new_goal = tasp_planner.tasp_path_planning(current_pose, inflated_grid_msg, start_pose)
            vis.publish_btp(tasp_planner.BTP)
            if new_goal:
                rospy.loginfo(f"[TASP] New goal: {new_goal}")
                current_goal = new_goal
                vis.publish_goal(current_goal)
            else:
                rospy.loginfo("[TASP] No more goals.")
                send_stop_signal()

        if current_goal:
            current_path = plan_path(inflated_grid_msg, current_pose, current_goal)
            if current_path:
                publish_path(current_path)
                vis.publish_astar_path(current_path)
            else:
                rospy.logwarn("[TASP] Path planning failed.")
                send_stop_signal()
        rate.sleep()
