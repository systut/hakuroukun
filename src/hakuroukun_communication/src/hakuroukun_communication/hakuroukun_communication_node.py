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
from std_msgs.msg import Float64MultiArray
import math
import numpy as np
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
        
        self.controller_subscriber = rospy.Subscriber(
            "/cmd_controller_input", Float64MultiArray, self._controller_input_callback)

        # Ros Timer
        self.timer = rospy.Timer(
            rospy.Duration(1/float(controller_rate)), 
            self._timer_callback)

        self.sequence_id = 0

        self.velocity_msg = Twist()

        self.controller_msg = Float64MultiArray()
        self.controller_msg.data = [0.0, 0.0]

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

        rospy.loginfo(f"0{self.direction}{steering_command}{acceleration_command}")

        command = f"0{self.direction}{steering_command}{acceleration_command}"

        self.connection.write(bytes(f"{command}\r\n", encoding='ascii'))

        time.sleep(3)

        self.connection.flush()

        data = b""

        data = self.connection.readline()

        rospy.loginfo(data)


    def _generate_command(self, acceleration_command, steering_command):
        """! Generate comment for serial communication
        @param[in] msg: velocity message in Twist form
        """
        pass

    def _controller_input_callback(self, msg: Float64MultiArray) -> None:
        """! Callback function for Controller input subscriber
        @param[in] msg: Controller input message in Float64MultiArray form
        """
        self.controller_msg = msg
    
    def _apply_indentification(self):
        """! Apply system indentification so as to send the right voltage
        @param[in] msg: velocity message in Twist form
        """

        self.direction = 0

        if self.controller_msg.data[0] < 0:
            self.direction = 1

        linear_velocity = abs(self.controller_msg.data[0])

        steering_angle = math.degrees(self.controller_msg.data[1])

        # # ==========================================================================
        # # TODO: Add system indentification equation here
        # # ==========================================================================

        ## NOTE: we should avoid magical number
        if linear_velocity == 0:
            acceleration_command = 290
        else:
            acceleration_command = (linear_velocity + 1)*315
            # acceleration_command = (linear_velocity + 1.41) / 0.002817

        if acceleration_command > 680:
            acceleration_command = 680
        elif acceleration_command < 290:
            acceleration_command = 290

        ## NOTE: we should avoid magical number
        steering_command = round(538.78+steering_angle/0.2362) # 538
        

        if steering_command > 760:
            steering_command = 760
        elif steering_command < 370:
            steering_command = 370

        return int(acceleration_command), int(steering_command)