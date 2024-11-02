#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# vim:fenc=utf-8
#
# Distributed under terms of the MIT license.

# Standard library

# External library
import rospy
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool

class ObstacleDetectionNode(object):

    def __init__(self) -> None:
        
        rospy.init_node("obstacle_detection_node", anonymous=True)

        # Subscribe to lidar data
        self.lidar_sub = rospy.Subscriber(
            "/scan_multi", LaserScan, self._lidar_callback)

        # Publish Emergency Signal
        self.emergency_stop_pub = rospy.Publisher(
            "emergency_stop", Bool, queue_size=10)

        # Set the obstacle detection threshold
        self.obstacle_threshold = 2.5  #diameter (m)

        # Initialize the emergency stop flag
        self.emergency_stop_signal = Bool()
        self.emergency_stop_signal.data = False

    def _lidar_callback(self, lidar_data) -> None:
        # Process the data to detect obstacle
        rospy.loginfo("Lidar data received")

        self.emergency_stop_signal.data = False

        for i, distance in enumerate(lidar_data.ranges):

            if distance <= self.obstacle_threshold:
                
                rospy.loginfo(f'Obstacle detecte at angle {i}: {distance} m away')

                self.emergency_stop_signal.data = True
                
                return
        
    def run(self):
        # Main loop
        while not rospy.is_shutdown():
            self.emergency_stop_pub.publish(self.emergency_stop_signal)
            if self.emergency_stop_signal.data:
                # Trigger emergency stop
                rospy.logwarn('Emergency stop trigged !')
            else:
                rospy.loginfo("No obstacle in range")

            rospy.sleep(0.1)
