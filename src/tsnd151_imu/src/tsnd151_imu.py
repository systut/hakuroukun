#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# vim:fenc=utf-8

from tsnd.tsnd151 import TSND151    
from queue import Queue
import numpy as np
import math
import time
from sensor_msgs.msg import Imu

import rospy

class TSND151_IMU:

    # ==========================================================================
    # PUBLIC FUNCTION
    # ==========================================================================

    def __init__(self):
            
        # ==========================================================================
        # Setting
        # ==========================================================================

        serial_port = rospy.get_param("/tsnd151_imu_node/serial_port")

        publish_rate = rospy.get_param("/tsnd151_imu_node/publish_rate")

        self.tsnd151 = TSND151.open(serial_port, wait_sec_on_open_for_stability=0.2, wait_sec_on_auto_close_for_stability=0.2)
        self.tsnd151.clear_all_queue()
        # self.tsnd151.close()
        hz = 100
        # self.tsnd151.stop_recording()
        self.tsnd151.set_time()
        self.tsnd151.set_acc_range(16)  # +-16g
        self.tsnd151.set_gyro_range(2000)  # +-2000dps
        self.tsnd151.set_acc_and_gyro_interval(  # perhaps ignored, because quat is enabled.
            interval_in_ms=2
            , avg_num_for_send=int(1000 / hz / 2)  # 10 if 50Hz: 10 avg with 2 ms data = 20 ms
            , avg_num_for_save=0)
        self.tsnd151.set_quaternion_interval(
            interval_in_5ms_unit=1
            , avg_num_for_send=int(1000 / hz / 5)  # 4 if 50Hz: 4 avg with 5 ms data = 20 ms
            , avg_num_for_save=0)

        self.tsnd151.set_magnetism_interval(0, 0, 0)  # disable
        self.tsnd151.set_atmosphere_interval(0, 0, 0)  # disable
        self.tsnd151.set_battery_voltage_measurement(False, False)  # disable
        self.tsnd151.set_overwrite_protection(False)  # enable overwrite
        self.tsnd151.set_auto_power_off(0)  # disable
        self.q = Queue()
        self.tsnd151.set_response_queue('quaternion_acc_gyro_data', self.q)
        self.start_time = self.tsnd151.start_recording(force_restart=True).timestamp()

        # ==========================================================================

        rospy.init_node("tsnd151_imu_node", anonymous=True)

        self.imu_pub = rospy.Publisher("/imu/data_raw", Imu, queue_size=10)

        self.timer = rospy.Timer(rospy.Duration(1/float(publish_rate)), self._publish_imu_data)

    def _run(self):

        rospy.spin()

    def _convert_imu_to_data(self,q_data):
        ms, quat, acc, gyro = self.tsnd151.parse_quaternion_acc_gyro(q_data)
        res=[]
        res.extend(gyro)
        res.extend(acc)
        res.append(quat)
        return res
    
    def _publish_imu_data(self, timer):

        imu_msg = Imu()

        if self.q.empty():
            print("No data")

        else:
            imu_data = [self._convert_imu_to_data(self.q.get()) for i in range(self.q.qsize())]
            # print(imu_data[0][6])

            imu_msg.header.frame_id = "imu"
            imu_msg.orientation.w =imu_data[0][6][0]/10000
            imu_msg.orientation.x =imu_data[0][6][1]/10000
            imu_msg.orientation.y =imu_data[0][6][2]/10000
            imu_msg.orientation.z =imu_data[0][6][3]/10000
            imu_msg.angular_velocity.x = float(imu_data[0][0]) * 0.01745329252   # 0.01 deg/s to rad/s
            imu_msg.angular_velocity.y = float(imu_data[0][1]) * 0.01745329252   # 0.01 deg/s to rad/s 
            imu_msg.angular_velocity.z = float(imu_data[0][2]) * 0.01745329252   # 0.01 deg/s to rad/s
            imu_msg.linear_acceleration.x = float(imu_data[0][3]) / 10**6        # mili-g to m/s^2
            imu_msg.linear_acceleration.y = float(imu_data[0][4]) / 10**6        # mili-g to m/s^2
            imu_msg.linear_acceleration.z = float(imu_data[0][5]) / 10**6        # mili-g to m/s^2

        self.imu_pub.publish(imu_msg)
