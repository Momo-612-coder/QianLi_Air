cmake_minimum_required(VERSION 3.0.2)
project(my_orb_slam)

## Compile as C++11, supported in ROS Kinetic and newer
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  airsim_ros
  message_generation
  nav_msgs
  roscpp
  rospy
  sensor_msgs
  std_msgs
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

set(ORB_SLAM3_ROOT "" CACHE PATH "Path to ORB_SLAM3 source/build root")
find_path(ORB_SLAM3_INCLUDE_DIR
  NAMES System.h
  PATHS
    ${ORB_SLAM3_ROOT}/include
    /usr/local/include/ORB_SLAM3
    /usr/include/ORB_SLAM3
)
find_library(ORB_SLAM3_LIBRARY
  NAMES ORB_SLAM3 libORB_SLAM3
  PATHS
    ${ORB_SLAM3_ROOT}/lib
    /usr/local/lib
    /usr/lib
)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS message_runtime roscpp std_msgs
)

include_directories(include ${catkin_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS})

if(ORB_SLAM3_INCLUDE_DIR AND ORB_SLAM3_LIBRARY)
  message(STATUS "Found ORB-SLAM3: ${ORB_SLAM3_LIBRARY}")
  include_directories(${ORB_SLAM3_INCLUDE_DIR})
  add_definitions(-DORB_SLAM3_AVAILABLE=1)
else()
  message(WARNING "ORB-SLAM3 not found. my_orb_slam will use fallback VO.")
  add_definitions(-DORB_SLAM3_AVAILABLE=0)
endif()


add_executable(my_orb_slam src/my_orb_slam.cpp )
target_link_libraries(my_orb_slam ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

if(ORB_SLAM3_INCLUDE_DIR AND ORB_SLAM3_LIBRARY)
  target_link_libraries(my_orb_slam ${ORB_SLAM3_LIBRARY})
endif()

install(DIRECTORY
  launch
  rviz
  config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)