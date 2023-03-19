cmake_minimum_required(VERSION 3.0.2)
project(hakuroukun_control)


find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  edge_msgs
  geometry_msgs
)

catkin_python_setup()

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES
CATKIN_DEPENDS rospy std_msgs edge_msgs
#  DEPENDS system_lib
)


include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

catkin_install_python(PROGRAMS scripts/hakuroukun_control_node
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

## Comment out but not delete for future use
# Catkin configuration to build tests
# if(CATKIN_ENABLE_TESTING)
#  catkin_add_nosetests(test/hakuroukun_control_test.py)
# endif()
