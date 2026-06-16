cmake_minimum_required(VERSION 3.0.2)
project(pwm_adrc_controller)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
airsim_ros
nav_msgs
quadrotor_msgs
roscpp
std_msgs
)

find_package(Eigen3 REQUIRED)
find_package(Threads REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS airsim_ros nav_msgs quadrotor_msgs roscpp std_msgs
)

include_directories(include ${catkin_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIR})


add_executable(pwm_adrc_controller src/pwm_adrc_controller.cpp src/teminal_control.cpp)
target_link_libraries(pwm_adrc_controller ${catkin_LIBRARIES} ${CMAKE_THREAD_LIBS_INIT})

catkin_install_python(PROGRAMS
  scripts/plot_step_response.py
  scripts/plot_gyro.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY
  launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  # PATTERN ".git" EXCLUDE
)
