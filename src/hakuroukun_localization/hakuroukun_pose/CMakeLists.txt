cmake_minimum_required(VERSION 3.0.2)
project(hakuroukun_pose)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  sensor_msgs
  geonav_transform
  geometry_msgs
  nav_msgs
  roscpp
  rospy
  tf
)

catkin_python_setup()

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES robot_position_localzation
 CATKIN_DEPENDS geonav_transform sensor_msgs geometry_msgs nav_msgs roscpp rospy tf
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

catkin_install_python(PROGRAMS scripts/hakuroukun_pose_node
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

#############
## Install ##
#############



#############
## Testing ##
#############