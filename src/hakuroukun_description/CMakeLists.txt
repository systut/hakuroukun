cmake_minimum_required(VERSION 3.0.2)
project(hakuroukun_description)

find_package(catkin REQUIRED COMPONENTS
    roslaunch
    urdf
    xacro
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES hakuroukun_description
#  CATKIN_DEPENDS other_catkin_pkg
#  DEPENDS system_lib
)

install(DIRECTORY launch urdf
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

include_directories(
# include
# ${catkin_INCLUDE_DIRS}
)