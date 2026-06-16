cmake_minimum_required(VERSION 3.0.2)
project(scp_mpc_trajectory)

add_compile_options(-std=c++14)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
airsim_ros
bspline_opt
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
traj_utils
)

find_package(Eigen3 REQUIRED)
find_package(OsqpEigen REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS bspline_opt message_runtime quadrotor_msgs roscpp std_msgs traj_utils # nodelet 
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(scp_mpc_trajectory src/scp_mpc_trajectory.cpp src/mpc_smoother.cpp)
target_link_libraries(scp_mpc_trajectory
  ${catkin_LIBRARIES}
  Eigen3::Eigen
)

if(TARGET OsqpEigen::OsqpEigen)
  target_link_libraries(scp_mpc_trajectory OsqpEigen::OsqpEigen)
else()
  target_include_directories(scp_mpc_trajectory PRIVATE ${OsqpEigen_INCLUDE_DIRS})
  target_link_libraries(scp_mpc_trajectory ${OsqpEigen_LIBRARIES})
endif()

install(TARGETS scp_mpc_trajectory
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY
  launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  # PATTERN ".git" EXCLUDE
)
