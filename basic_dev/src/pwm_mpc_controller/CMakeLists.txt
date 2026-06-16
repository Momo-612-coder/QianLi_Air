cmake_minimum_required(VERSION 3.0.2)
project(pwm_mpc_controller)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
set(pcl_conversions_DIR /usr/local/share/pcl_conversions/cmake)
set(pcl_ros_DIR /usr/local/share/pcl_ros/cmake)
find_package(catkin REQUIRED COMPONENTS
airsim_ros
message_generation
nav_msgs
# nodelet
roscpp
rospy
sensor_msgs
std_msgs
geographic_msgs
geometry_msgs
std_srvs
tf2
tf2_ros
tf2_sensor_msgs
tf2_geometry_msgs
image_transport
cv_bridge
pcl_ros
pcl_conversions
)

find_package(OpenCV REQUIRED)
find_package(Threads REQUIRED)

find_path(CASADI_INCLUDE_DIR casadi/casadi.hpp
  HINTS
    /opt/casadi/include
    /usr/local/include
)
find_library(CASADI_LIBRARY casadi
  HINTS
    /opt/casadi/lib
    /usr/local/lib
)

if(NOT CASADI_INCLUDE_DIR OR NOT CASADI_LIBRARY)
  message(FATAL_ERROR "CasADi not found. Please install CasADi C++ headers and library (e.g. under /opt/casadi).")
endif()

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS message_runtime roscpp std_msgs # nodelet 
)

include_directories(include ${catkin_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS} ${CASADI_INCLUDE_DIR})


add_executable(pwm_mpc_controller src/pwm_mpc_controller.cpp src/terminal_control.cpp src/mpc.cpp)
target_link_libraries(pwm_mpc_controller ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} ${CASADI_LIBRARY} ${CMAKE_THREAD_LIBS_INIT})

install(DIRECTORY
  launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  # PATTERN ".git" EXCLUDE
)