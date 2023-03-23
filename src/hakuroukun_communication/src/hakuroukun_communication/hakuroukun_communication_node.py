#! /usr/bin/env python3
# -*- coding: utf-8 -*-
# vim:fenc=utf-8
#
# Distributed under terms of the MIT license.

# Standard library


# External library
import serial
import rospy
from geometry_msgs.msg import Twist
import math
import numpy as np

class HakuroukunCommunicationNode(object):
    """!
    @brief This class privide a ROS node to control hakuroukun robot using serial
    connection to motor control circuit
    """
    # ==========================================================================
    # PUBLICH FUNCTION
    # ==========================================================================
    def __init__(self) -> None:
        """! Class constructor
        """
        rospy.init_node("hakuroukun_communication_node", anonymous=True)
        
        port = rosparam.get_param("/hakuroukun_communication_node/port")

        baud_rate = rosparam.get_param(
            "/hakuroukun_communication_node/baud_rate")

        controller_rate = rosparam.get_param(
            "/hakuroukun_communication_node/controller_rate")

        self.connection = serial.Serial(port, baud_rate, timeout=None)

        self.velocity_subscriber = rospy.Subscriber(
            "/cmd_vel", Twist, self._velocity_callback)

        self.timer = rospy.Timer(
            rospy.Duration(controller_rate), 
            self._timer_callback)

    def run(self) -> None:
        """! Start ros node
        """
        rospy.spin()

    # ==========================================================================
    # PRIVATE FUNCTION
    # ==========================================================================
    def _timer_callback(self, event: rospy.TimerEvent) -> None:
        """! Callback function for velocity timer
        @param[in] event: timer event
        """
        acceleration_command, steering_command = self._apply_indentification()

        self.connection.write(f"{acceleration_command}{steering_command}")
    
    def _velocity_callback(self, msg: Twist) -> None:
        """! Callback function for velocity subscriber
        @param[in] msg: velocity message in Twist form
        """
        self.velocity_msg = msg
    
    def _apply_indentification(self) -> Tuple[str, str]:
        """! Apply system indentification so as to send the right voltage
        @param[in] msg: velocity message in Twist form
        """
        linear_velocity = self.velocity_msg.linear.x

        angular_velocity = self.velocity_msg.angular.z

        # TODO: Add system indentification equation here

        ## NOTE: why sampling time is needed here?
        sampling_time = 0.1

        ## NOTE: we should avoid magical number
        acceleration_command = (linear_velocity + 1.93)/0.003486

        ## NOTE: we should avoid magical number
        steering command = (math.degrees(np.arcsin(0.95*angular_velocity/0.27))+127.26)/0.2362

        ## NOTE: we should avoid magical number
        if acceleration_command > 680:
            acceleration_command = 680
        else if acceleration_command < 580:
            acceleration_command = 580

        ## NOTE: we should avoid magical number
        if steering_command > 760:
            steering_command = 760
        else if steering_command < 370:
            steering_command = 370

        return acceleration_command, steering_command