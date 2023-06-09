#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# vim:fenc=utf-8
#
# Distributed under terms of the MIT license.

# Standard library

# External library
import serial
import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool
import math
import numpy as np
import rosparam
import time


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
        
        # Get parametes
        port = rospy.get_param("/hakuroukun_communication_node/port")

        baud_rate = rospy.get_param(
            "/hakuroukun_communication_node/baud_rate")

        controller_rate = rospy.get_param(
            "/hakuroukun_communication_node/controller_rate")

        self.connection = serial.Serial(port, int(baud_rate), timeout=None)

        # Velocity subscriber
        self.velocity_subscriber = rospy.Subscriber(
            "/cmd_vel", Twist, self._velocity_callback)

        # Ros Timer
        self.timer = rospy.Timer(
            rospy.Duration(1/float(controller_rate)), 
            self._timer_callback)

        self.sequence_id = 0

        self.velocity_msg = Twist()

    def run(self) -> None:
        """! Start ros node
        """
        rospy.spin()

    # ==========================================================================
    # PRIVATE FUNCTION
    # ==========================================================================
    def _timer_callback(self, event) -> None:
        """! Callback function for velocity timer
        @param[in] event: timer event
        """
        acceleration_command, steering_command = self._apply_indentification()

        command = f"0{steering_command}{acceleration_command}"

        self.connection.write(bytes(f"{command}\r\n", encoding='ascii'))

        self.connection.flush()

        data = b""

        data = self.connection.readline()

        rospy.loginfo(data)


    def _generate_command(self, acceleration_command, steering_command):
        """! Generate comment for serial communication
        @param[in] msg: velocity message in Twist form
        """
        pass

    def _velocity_callback(self, msg: Twist) -> None:
        """! Callback function for velocity subscriber
        @param[in] msg: velocity message in Twist form
        """
        self.velocity_msg = msg
    
    def _apply_indentification(self):
        """! Apply system indentification so as to send the right voltage
        @param[in] msg: velocity message in Twist form
        """

        # Emergency Stop Flag Check
        linear_velocity = self.velocity_msg.linear.x

        angular_velocity = self.velocity_msg.angular.z

        # ==========================================================================
        # TODO: Add system indentification equation here
        # ==========================================================================

        ## NOTE: we should avoid magical number
        acceleration_command = (linear_velocity + 1)*290

        ## NOTE: we should avoid magical number
        steering_command = (math.degrees(np.arcsin(0.95*angular_velocity/0.27))+127.26)/0.2362

        ## NOTE: we should avoid magical number
        if acceleration_command > 680:
            acceleration_command = 680
        elif acceleration_command < 290:
            acceleration_command = 290

        ## NOTE: we should avoid magical number
        if steering_command > 760:
            steering_command = 760
        elif steering_command < 370:
            steering_command = 370

        return int(acceleration_command), int(steering_command)