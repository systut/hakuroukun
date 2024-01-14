#!/usr/bin/env python3

import rospy
from tsnd151_imu import TSND151_IMU

if __name__ == '__main__':

    try:
        tsnd151 = TSND151_IMU()
        print("Turning on IMU")
        tsnd151._run()
    finally:
        print("Shuting down ")
        tsnd151.tsnd151.stop_recording()
        tsnd151.tsnd151.close()
        pass
