#!/usr/bin/env python3

import rospy
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path

class TASPVisualizer:
    def __init__(self, frame_id="map"):
        # Publishers for various visualization messages
        self.btp_pub = rospy.Publisher('/tasp/btp_markers', MarkerArray, queue_size=10)
        self.goal_pub = rospy.Publisher('/tasp/goal_pose', PoseStamped, queue_size=10)
        self.goal_marker_pub = rospy.Publisher('/tasp/goal_marker', Marker, queue_size=10)
        self.path_pub = rospy.Publisher('/tasp/astar_path', Path, queue_size=10)
        self.start_marker_pub = rospy.Publisher('/tasp/start_marker', Marker, queue_size=10)

        self.frame_id = frame_id

    def publish_start(self, start_pose):
        """
        start_pose: [x, y, (optionally theta)] or just [x, y].
        We'll ignore theta for the marker.
        """
        marker = Marker()
        marker.header.frame_id = self.frame_id
        marker.header.stamp = rospy.Time.now()
        marker.ns = "robot_start"
        marker.id = 0
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose.position.x = start_pose[0]
        marker.pose.position.y = start_pose[1]
        marker.pose.position.z = 0.0
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.1
        marker.scale.y = 0.1
        marker.scale.z = 0.1

        # Color = green
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        marker.color.a = 1.0

        self.start_marker_pub.publish(marker)

    def publish_btp(self, btp_points):
        """
        btp_points: list of [x, y].
        Publish new markers for the provided points, explicitly deleting old markers.
        """
        # Keep track of currently active marker IDs
        active_ids = set(range(len(btp_points)))
        # Keep track of previously published IDs
        if not hasattr(self, 'previous_ids'):
            self.previous_ids = set()

        # Create MarkerArray for new markers
        marker_array = MarkerArray()

        # Add new markers
        for i, (bx, by) in enumerate(btp_points):
            marker = Marker()
            marker.header.frame_id = self.frame_id
            marker.header.stamp = rospy.Time.now()
            marker.ns = "tasp_btp"
            marker.id = i
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD
            marker.pose.position.x = bx
            marker.pose.position.y = by
            marker.pose.position.z = 0.0
            marker.pose.orientation.w = 1.0
            marker.scale.x = 0.1
            marker.scale.y = 0.1
            marker.scale.z = 0.1

            # Blue color
            marker.color.r = 0.0
            marker.color.g = 0.0
            marker.color.b = 1.0
            marker.color.a = 1.0

            marker_array.markers.append(marker)

        # Determine IDs to delete (previously published but not in the new list)
        ids_to_delete = self.previous_ids - active_ids
        for id_to_delete in ids_to_delete:
            delete_marker = Marker()
            delete_marker.header.frame_id = self.frame_id
            delete_marker.header.stamp = rospy.Time.now()
            delete_marker.ns = "tasp_btp"
            delete_marker.id = id_to_delete
            delete_marker.action = Marker.DELETE
            marker_array.markers.append(delete_marker)

        # Update the previous IDs
        self.previous_ids = active_ids

        # Publish the MarkerArray
        self.btp_pub.publish(marker_array)

    def publish_goal(self, goal_point):
        """
        goal_point: [x, y]
        Publish as both a PoseStamped and a Marker.
        """
        gx, gy = goal_point

        # PoseStamped
        goal_pose = PoseStamped()
        goal_pose.header.frame_id = self.frame_id
        goal_pose.header.stamp = rospy.Time.now()
        goal_pose.pose.position.x = gx
        goal_pose.pose.position.y = gy
        goal_pose.pose.orientation.w = 1.0
        self.goal_pub.publish(goal_pose)

        # Marker
        marker = Marker()
        marker.header.frame_id = self.frame_id
        marker.header.stamp = rospy.Time.now()
        marker.ns = "tasp_goal"
        marker.id = 0
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose.position.x = gx
        marker.pose.position.y = gy
        marker.pose.position.z = 0.0
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.1
        marker.scale.y = 0.1
        marker.scale.z = 0.1

        # Red color
        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0
        marker.color.a = 1.0

        self.goal_marker_pub.publish(marker)

    def publish_astar_path(self, path_points):
        """
        path_points: list of [x, y] from A*
        We'll publish as nav_msgs/Path
        """
        path_msg = Path()
        path_msg.header.frame_id = self.frame_id
        path_msg.header.stamp = rospy.Time.now()

        for (px, py) in path_points:
            pose_stamped = PoseStamped()
            pose_stamped.header.frame_id = self.frame_id
            pose_stamped.header.stamp = rospy.Time.now()
            pose_stamped.pose.position.x = px
            pose_stamped.pose.position.y = py
            pose_stamped.pose.orientation.w = 1.0
            path_msg.poses.append(pose_stamped)

        self.path_pub.publish(path_msg)
