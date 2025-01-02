cmake_minimum_required(VERSION 3.0.2)
project(traffic_cones_detection)

find_package(catkin REQUIRED COMPONENTS
  rospy
)

catkin_python_setup()

catkin_package(
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

###########
# Install #
###########

# Mark executable scripts (Python etc.) for installation
catkin_install_python(PROGRAMS
  scripts/traffic_cones_detection_node
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)