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
        
        port = rosparam.get_param("/port")

        baud_rate = rosparam.get_param("/baud_rate")

        controller_rate = rosparam.get_param("/controller_rate")

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

        return acceleration_command, steering_command