cmake_minimum_required(VERSION 3.0.2)
project(hakuroukun_steering_controller)

# Load catkin and all dependencies required for this package
find_package(catkin REQUIRED COMPONENTS
  controller_interface
  diff_drive_controller
  hardware_interface
  nav_msgs
  pluginlib
  realtime_tools
  roscpp
  tf
)

find_package(Boost REQUIRED)

find_package(urdfdom REQUIRED)

# Declare a catkin package
catkin_package(
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS
    controller_interface
    diff_drive_controller
    hardware_interface
    nav_msgs
    realtime_tools
    roscpp
    tf
)

###########
## Build ##
###########

# Specify header include paths
include_directories(
  include
  ${Boost_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
  ${urdfdom_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME} src/hakuroukun_steering_controller.cpp src/odometry.cpp)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} ${urdfdom_LIBRARIES})

#############
## Install ##
#############

# Install headers
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

# Install targets
install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# Install plugins
install(FILES ${PROJECT_NAME}_plugins.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
