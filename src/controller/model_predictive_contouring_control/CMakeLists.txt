cmake_minimum_required(VERSION 3.0.2)
project(model_predictive_contouring_control)

find_package(catkin REQUIRED COMPONENTS
  genmsg
  message_generation
  roscpp
  sdv_msgs
  std_msgs
)

Set(SOURCES
    ${CMAKE_CURRENT_SOURCE_DIR}/src/arc_length_spline.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/bounds.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/constraints_test.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/constraints.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/cost_test.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/cost.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/cubic_spline.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/hpipm_interface.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/integrator.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/model_integrator_test.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/model.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/mpc.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/params.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/plotting.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/soft_constraints.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/solver_interface.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/spline_test.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/track.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/types.cpp
    )

set(HEADERS
    ${CMAKE_CURRENT_SOURCE_DIR}/include/model_predictive_contouring_control/arc_length_spline.h
    ${CMAKE_CURRENT_SOURCE_DIR}/include/model_predictive_contouring_control/bounds.h
    ${CMAKE_CURRENT_SOURCE_DIR}/include/model_predictive_contouring_control/config.h
    ${CMAKE_CURRENT_SOURCE_DIR}/include/model_predictive_contouring_control/constraints_test.h
    ${CMAKE_CURRENT_SOURCE_DIR}/include/model_predictive_contouring_control/constraints.h
    ${CMAKE_CURRENT_SOURCE_DIR}/include/model_predictive_contouring_control/cost_test.h
    ${CMAKE_CURRENT_SOURCE_DIR}/include/model_predictive_contouring_control/cost.h
    ${CMAKE_CURRENT_SOURCE_DIR}/include/model_predictive_contouring_control/cubic_spline.h
    ${CMAKE_CURRENT_SOURCE_DIR}/include/model_predictive_contouring_control/hpipm_interface.h
    ${CMAKE_CURRENT_SOURCE_DIR}/include/model_predictive_contouring_control/integrator.h
    ${CMAKE_CURRENT_SOURCE_DIR}/include/model_predictive_contouring_control/model_integrator_test.h
    ${CMAKE_CURRENT_SOURCE_DIR}/include/model_predictive_contouring_control/model.h
    ${CMAKE_CURRENT_SOURCE_DIR}/include/model_predictive_contouring_control/mpc.h
    ${CMAKE_CURRENT_SOURCE_DIR}/include/model_predictive_contouring_control/params.h
    ${CMAKE_CURRENT_SOURCE_DIR}/include/model_predictive_contouring_control/plotting.h
    ${CMAKE_CURRENT_SOURCE_DIR}/include/model_predictive_contouring_control/soft_constraints.h
    ${CMAKE_CURRENT_SOURCE_DIR}/include/model_predictive_contouring_control/solver_interface.h
    ${CMAKE_CURRENT_SOURCE_DIR}/include/model_predictive_contouring_control/spline_test.h
    ${CMAKE_CURRENT_SOURCE_DIR}/include/model_predictive_contouring_control/track.h
    ${CMAKE_CURRENT_SOURCE_DIR}/include/model_predictive_contouring_control/types.h

    )

add_library(mpcc_lib ${SOURCES} ${HEADERS})
find_package(Python REQUIRED Development)
find_package(PythonLibs 2.7 REQUIRED)
include_directories(${PYTHON_INCLUDE_DIRS})

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_FLAGS "-O2")

## System dependencies are found with CMake's conventions
include_directories(
  include ${catkin_INCLUDE_DIRS}
  include/External/blasfeo/lib/include
  include/External/hpipm/lib/include
  include/External/matplotlib
  include/External/Eigen
  include/External/Json/include
)

###################################
## catkin specific configuration ##
###################################

catkin_package(
 INCLUDE_DIRS include
 LIBRARIES ${PROJECT_NAME}
 CATKIN_DEPENDS roscpp
)

###########
## Build ##
###########

add_executable(mpcc src/main.cpp)
target_link_libraries(mpcc ${catkin_LIBRARIES} mpcc_lib
/root/catkin_ws/src/controller/model_predictive_contouring_control/include/External/hpipm/lib/lib/libhpipm.a
/root/catkin_ws/src/controller/model_predictive_contouring_control/include/External/blasfeo/lib/lib/libblasfeo.a)
target_link_libraries(mpcc ${Python_LIBRARIES})
target_link_libraries(mpcc Python::Python)

add_executable(mpcc_node src/mpcc_controller_node.cpp)
target_link_libraries(mpcc_node ${catkin_LIBRARIES} mpcc_lib
/root/catkin_ws/src/controller/model_predictive_contouring_control/include/External/hpipm/lib/lib/libhpipm.a
/root/catkin_ws/src/controller/model_predictive_contouring_control/include/External/blasfeo/lib/lib/libblasfeo.a)
target_link_libraries(mpcc_node ${Python_LIBRARIES})
target_link_libraries(mpcc_node Python::Python)

# add_executable(mpcc src/main.cpp)
# target_link_libraries(mpcc ${catkin_LIBRARIES} mpcc_lib
# /home/musashi/work/duc/robot_hakuroukun/hakuroukun_ws/src/controller/model_predictive_contouring_control/include/External/hpipm/lib/lib/libhpipm.a
# /home/musashi/work/duc/robot_hakuroukun/hakuroukun_ws/src/controller/model_predictive_contouring_control/include/External/blasfeo/lib/lib/libblasfeo.a)
# target_link_libraries(mpcc ${Python_LIBRARIES})
# target_link_libraries(mpcc Python::Python)

# add_executable(mpcc_node src/mpcc_controller_node.cpp)
# target_link_libraries(mpcc_node ${catkin_LIBRARIES} mpcc_lib
# /home/musashi/work/duc/robot_hakuroukun/hakuroukun_ws/src/controller/model_predictive_contouring_control/include/External/hpipm/lib/lib/libhpipm.a
# /home/musashi/work/duc/robot_hakuroukun/hakuroukun_ws/src/controller/model_predictive_contouring_control/include/External/blasfeo/lib/lib/libblasfeo.a)
# target_link_libraries(mpcc_node ${Python_LIBRARIES})
# target_link_libraries(mpcc_node Python::Python)