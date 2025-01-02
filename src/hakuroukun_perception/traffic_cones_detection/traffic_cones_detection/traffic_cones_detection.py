#!/usr/bin/env python3
##
# @file traffic_cones_detection.py
#
# @brief Provide implementation of traffic cones detection for autonomous
# driving.
#
# @section author_doxygen_example Author(s)
# - Created by Dinh Ngoc Duc on 16/10/2024.
#
# Copyright (c) 2024 System Engineering Laboratory.  All rights reserved.

# Standard library


# External libraries
import rospy
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image, CameraInfo
from message_filters import Subscriber, ApproximateTimeSynchronizer


class TrafficConesDetection:
    """! Traffic cones detection

    The class provides implementation of traffic cones detection for autonomous
    driving.
    """
    # ==================================================================================================
    # PUBLIC METHODS
    # ==================================================================================================

    def __init__(self):
        """! Constructor
        """
        self._read_parameters()

        self._register_subscribers()

        self._register_timers()

    # ==================================================================================================
    # PRIVATE METHODS
    # ==================================================================================================
    def _read_parameters(self):
        """! Read parameters
        """
        self._sampling_time = rospy.get_param("~sampling_time", 0.1)

    def _register_timers(self):
        """! Register timers
        """
        self.timer = rospy.Timer(
            rospy.Duration(self._sampling_time), self._timer_callback)

    def _register_subscribers(self):
        """! Register subscribers
        """
        rgb_image_subscriber = Subscriber(
            "/camera/color/image_raw", Image)

        depth_image_subscriber = Subscriber(
            "/camera/depth/image_raw", Image)

        camera_info_subscriber = Subscriber(
            "/camera/depth/camera_info", CameraInfo)

        self._image = None

        ApproximateTimeSynchronizer([
            rgb_image_subscriber,
            depth_image_subscriber,
            camera_info_subscriber], 10, 0.1
        ).registerCallback(self._msgs_callback)

    def _msgs_callback(self, rgb_image, depth_image, camera_info):
        """! Callback for syncronized subscribers
        @param rgb_image<Image>: RGB image callback messages
        @param depth_image<Image>: depth image callback messages
        @param camera_info<CameraInfo>: camera info callback message
        """
        with self.mutex_lock:
            try:
                self._rgb_image = CvBridge().imgmsg_to_cv2(rgb_image, "bgr8")

                # self.depth_image -> np.float32
                self._depth_image = CvBridge().imgmsg_to_cv2(
                    depth_image, desired_encoding='passthrough')

                self._camera_info = camera_info

            except CvBridgeError as e:
                rospy.logerr(e)

    def _timer_callback(self, event):
        """! Timer callback
        """
        if not self._rgb_image:
            return

        self._detect_traffic_cones()

    def _detect_traffic_cones(self):
        """! Detect traffic cones
        """
        pass
