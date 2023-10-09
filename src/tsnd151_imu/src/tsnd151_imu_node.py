#!/usr/bin/env python3

import rospy
from tsnd151_imu import TSND151_IMU

if __name__ == '__main__':

    try:
        tsnd151 = TSND151_IMU()
        print("entering Try")
        tsnd151._run()
    finally:
        print("shuting down ##################")
        tsnd151.tsnd151.stop_recording()
        tsnd151.tsnd151.close()
        pass
