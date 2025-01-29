#!/usr/bin/env python3
import rospy
import math
from nav_msgs.msg import Odometry, OccupancyGrid, Path
from geometry_msgs.msg import PoseStamped
from tf.transformations import euler_from_quaternion
from std_msgs.msg import Bool

# Import your TASPPathPlanner
from tasp_path_planner import TASPPathPlanner
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
    path_msg.header.frame_id = "odom"
    path_msg.header.stamp = rospy.Time.now()

    for px, py in path_points:
        pose_stamped = PoseStamped()
        pose_stamped.header.frame_id = "odom"
        pose_stamped.header.stamp = rospy.Time.now()
        pose_stamped.pose.position.x = px
        pose_stamped.pose.position.y = py
        pose_stamped.pose.orientation.w = 1.0
        path_msg.poses.append(pose_stamped)

    path_pub.publish(path_msg)

if __name__ == "__main__":
    rospy.init_node("tasp_with_cones")

    # Load parameters from the parameter server
    start_pose = rospy.get_param("start_pose", [0, 0, 0])
    map_resolution = rospy.get_param("map_resolution", 0.05)
    map_width = rospy.get_param("map_width", 4000)
    map_height = rospy.get_param("map_height", 4000)
    map_origin = tuple(rospy.get_param("map_origin", [-100, -100]))
    tasp_cell_size = rospy.get_param("tasp_cell_size", 1)
    tasp_goal_tolerance = rospy.get_param("tasp_goal_tolerance", 1)
    wait_duration = rospy.Duration(rospy.get_param("wait_duration", 5.0))
    rate_hz = rospy.get_param("rate_hz", 2)

    tasp_planner = TASPPathPlanner()
    vis = TASPVisualizer(frame_id="odom")

    # Define topics in the script
    rospy.Subscriber("/hakuroukun_pose/rear_wheel_odometry", Odometry, odom_callback)
    rospy.Subscriber("/costmap_node/costmap/costmap", OccupancyGrid, costmap_callback)

    path_pub = rospy.Publisher("/desired_path", Path, queue_size=10)
    stop_signal_pub = rospy.Publisher('/stop_signal', Bool, queue_size=10)

    rate = rospy.Rate(rate_hz)
    start_time = rospy.Time.now()

    current_goal = None
    current_path = []

    while not rospy.is_shutdown():
        vis.publish_start(start_pose)
        vis.publish_camera_fov()

        # Wait for map update
        if rospy.Time.now() - start_time < wait_duration:
            rate.sleep()
            continue

        if inflated_grid_msg is None:
            rospy.logwarn_throttle(5, "Waiting for inflated costmap data...")
            rate.sleep()
            continue

        # ------------------------------------------------
        # 1. If no current goal, get a new TASP goal
        # ------------------------------------------------
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

        # ------------------------------------------------
        # 2. Attempt A* path to the current_goal
        # ------------------------------------------------
        current_path = plan_path(inflated_grid_msg, current_pose, current_goal)

        if current_path == "GOAL_OCCUPIED":
            # -----------------------------------------------
            # The goal cell itself is occupied -> skip it
            # -----------------------------------------------
            rospy.logwarn("[TASP] The current goal cell is occupied. Skipping that goal.")
            # Optionally remove current_goal from the BTP if it exists there
            if current_goal in tasp_planner.BTP:
                tasp_planner.BTP.remove(current_goal)

            # Force the planner to pick another TASP goal
            current_goal = None
            rate.sleep()
            continue

        if not current_path:
            # -------------------------------------------------------------
            # Path is empty for some other reason (blocked, no route, etc.)
            # You can decide to skip the goal or stop the robot.
            # -------------------------------------------------------------
            rospy.logwarn("[TASP] Path planning failed (no route). Skipping goal.")
            send_stop_signal()
            # Optionally remove current_goal from the BTP if it exists there
            # if current_goal in tasp_planner.BTP:
            #     tasp_planner.BTP.remove(current_goal)

            # current_goal = None
            # rate.sleep()
            continue

        # If we got here, we have a valid path
        publish_path(current_path)
        vis.publish_astar_path(current_path)

        rate.sleep()