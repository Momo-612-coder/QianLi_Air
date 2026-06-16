cmake_minimum_required(VERSION 3.0.2)
project(pwm_se3_controller)

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
quadrotor_msgs
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

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS airsim_ros message_runtime quadrotor_msgs roscpp std_msgs # nodelet
)

include_directories(include ${catkin_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS})


add_executable(pwm_se3_controller src/pwm_se3_controller.cpp src/se3.cpp src/terminal_control.cpp)
add_dependencies(pwm_se3_controller ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(pwm_se3_controller ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} ${CMAKE_THREAD_LIBS_INIT})

install(DIRECTORY
  launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  # PATTERN ".git" EXCLUDE
)
