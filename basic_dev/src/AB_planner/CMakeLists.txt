cmake_minimum_required(VERSION 3.0.2)
project(AB_planner)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
airsim_ros
message_generation
nav_msgs
# nodelet
quadrotor_msgs
roscpp
rospy
std_msgs
geographic_msgs
geometry_msgs
lidar_solver
std_srvs
)

find_package(OsqpEigen REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS airsim_ros geometry_msgs lidar_solver message_runtime nav_msgs quadrotor_msgs roscpp std_msgs # nodelet
)

include_directories(include ${catkin_INCLUDE_DIRS})


add_executable(AB_planner_node
  src/AB_planner.cpp
  src/A_star.cpp
  src/B_spline.cpp
  src/mpc_smoother.cpp
)
add_dependencies(AB_planner_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(AB_planner_node ${catkin_LIBRARIES})

if(TARGET OsqpEigen::OsqpEigen)
  target_link_libraries(AB_planner_node OsqpEigen::OsqpEigen)
else()
  target_include_directories(AB_planner_node PRIVATE ${OsqpEigen_INCLUDE_DIRS})
  target_link_libraries(AB_planner_node ${OsqpEigen_LIBRARIES})
endif()

install(TARGETS AB_planner_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY
  launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  # PATTERN ".git" EXCLUDE
)
