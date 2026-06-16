cmake_minimum_required(VERSION 3.0.2)
project(pointcloud2_to_livox)

add_compile_options(-std=c++14 -Wall -Wextra -Wpedantic)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  geometry_msgs
  livox_ros_driver
  tf2_ros
  tf2_sensor_msgs
  pcl_conversions
)

find_package(PCL REQUIRED COMPONENTS filters)

catkin_package(
  CATKIN_DEPENDS roscpp sensor_msgs geometry_msgs livox_ros_driver tf2_ros tf2_sensor_msgs pcl_conversions
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

add_executable(pointcloud2_to_livox_node src/pointcloud2_to_livox_node.cpp)
add_dependencies(pointcloud2_to_livox_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(pointcloud2_to_livox_node ${catkin_LIBRARIES} ${PCL_LIBRARIES})

install(TARGETS pointcloud2_to_livox_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
