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

        time.sleep(2)

        rospy.loginfo(f"Connected to {port} at {baud_rate} baud rate")
        self.cmd_controller_subscriber = rospy.Subscriber(
            "/cmd_controller", Float64MultiArray, self._cmd_controller_callback)

        # self.cmd_vel_subscriber = rospy.Subscriber(
        #     "/cmd_vel", Twist, self._cmd_vel_callback)

        # Ros Timer
        self.timer = rospy.Timer(
            rospy.Duration(1/float(controller_rate)),
            self._timer_callback)

        self.sequence_id = 0

        self.cmd_vel_msg = Twist()

        self.cmd_controller_msg = Float64MultiArray()

        self.cmd_controller_msg.data = [0.0, 0.0]

        self.cumulative_steering_angle = 0.0  # Initialize cumulative steering angle

        self.cmd_vel_flag = False

        self.cmd_controller_flag = False

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

        command = f"0{self.direction}{steering_command}{acceleration_command}"

        rospy.loginfo(command)

        self.connection.write(bytes(f"{command}\r\n", encoding='ascii'))

        self.connection.flush()

        data = b""

        data = self.connection.readline()

    def _generate_command(self, acceleration_command, steering_command):
        """! Generate comment for serial communication
        @param[in] msg: velocity message in Twist form
        """
        pass

    def _cmd_controller_callback(self, msg: Float64MultiArray) -> None:
        """! Callback function for Controller input subscriber
        @param[in] msg: Controller input message in Float64MultiArray form
        """
        self.cmd_controller_msg = msg

        self.cmd_controller_flag = True

    def _cmd_vel_callback(self, msg: Twist) -> None:
        """! Callback function for velocity subscriber
        @param[in] msg: velocity message in Twist form
        """
        self.cmd_vel_msg = msg

        rospy.loginfo(f"Velocity: {self.cmd_vel_msg}")

        self.cmd_vel_flag = True

        # rospy.loginfo(f"Velocity: {self.cmd_vel_msg}")

    def _apply_indentification(self):
        """! Apply system indentification so as to send the right voltage
        @param[in] msg: velocity message in Twist form
        """

        linear_velocity = 0.0

        steering_angle = 0.0

        self.direction = 0

        if self.cmd_vel_flag:

            self.cmd_vel_flag = False

            if self.cmd_vel_msg.linear.x < 0:

                self.direction = 1

            linear_velocity = abs(self.cmd_vel_msg.linear.x)

            steering_angle_ = math.degrees(self.cmd_vel_msg.angular.z)

            self.cumulative_steering_angle += steering_angle_ * 0.5

            steering_angle = self.cumulative_steering_angle

        elif self.cmd_controller_flag:

            if self.cmd_controller_msg.data[0] < 0:
                self.direction = 1

            linear_velocity = abs(self.cmd_controller_msg.data[0])

            steering_angle = math.degrees(self.cmd_controller_msg.data[1])

        # NOTE: we should avoid magical number
        if linear_velocity == 0:
            acceleration_command = 290
        else:
            acceleration_command = (linear_velocity + 1)*500
            # acceleration_command = (linear_velocity + 1.41) / 0.002817

        if acceleration_command > 680:
            acceleration_command = 680
        elif acceleration_command < 290:
            acceleration_command = 290

        # NOTE: we should avoid magical number
        steering_command = round(520+steering_angle/0.2362)  # 538
        # steering_command = round(555+steering_angle/0.2362) # 538

        if steering_command > 760:
            steering_command = 760
        elif steering_command < 370:
            steering_command = 370

        return int(acceleration_command), int(steering_command)
