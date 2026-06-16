cmake_minimum_required(VERSION 3.0.2)
project(lidar_solver)

## Compile as C++14 for std::make_unique and modern ROS Noetic toolchains.
add_compile_options(-std=c++14)

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
visualization_msgs
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

add_message_files(
  FILES
  Obstacle.msg
  ObstacleArray.msg
)

generate_messages(
  DEPENDENCIES
  geometry_msgs
  std_msgs
)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS airsim_ros geometry_msgs message_runtime roscpp std_msgs visualization_msgs # nodelet
)

include_directories(include ${catkin_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS})


add_executable(lidar_solver src/lidar_solver.cpp )
add_dependencies(lidar_solver ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(lidar_solver ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

install(DIRECTORY
  launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  # PATTERN ".git" EXCLUDE
)
