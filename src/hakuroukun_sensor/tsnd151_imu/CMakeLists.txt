cmake_minimum_required(VERSION 2.8.3)
project(tsnd151_imu)


find_package(catkin REQUIRED COMPONENTS
  rospy
  roscpp
  geometry_msgs
  sensor_msgs
  serial
  std_msgs
  std_srvs
  tf
)

catkin_package(
  # INCLUDE_DIRS include
  CATKIN_DEPENDS rospy roscpp geometry_msgs sensor_msgs serial std_msgs std_srvs tf
  # DEPENDS system_lib
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)