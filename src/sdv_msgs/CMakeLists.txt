cmake_minimum_required(VERSION 3.0.2)
project(sdv_msgs)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  message_generation
  roscpp
  rospy
  genmsg
  std_msgs
  tf
)

# find_package(Eigen3 REQUIRED)

add_message_files(
  DIRECTORY msg
  FILES
  Trajectory.msg
  TrajectoryPoint.msg
)

# add_service_files(
#   FILES
#   TrajectoryFlags.srv
# )

generate_messages(
  DEPENDENCIES
  std_msgs
  # sdv_msgs
)

catkin_package(
INCLUDE_DIRS include
DEPENDS EIGEN3
CATKIN_DEPENDS message_runtime std_msgs
)

include_directories(
  include
  ${EIGEN_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)