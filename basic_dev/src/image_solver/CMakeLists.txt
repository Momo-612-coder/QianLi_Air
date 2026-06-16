cmake_minimum_required(VERSION 3.0.2)
project(image_solver)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

add_definitions(-DUSE_CUDA_DETCTOR)
# 设置默认的 ONNX Runtime 根目录，如果用户没有手动提供
if(NOT ONNXRUNTIME_ROOTDIR)
  if(WIN32)
    # Windows 系统下设置默认的 ONNX Runtime 路径
    set(ONNXRUNTIME_ROOTDIR "C:/Program Files (x86)/onnxruntime")
  else()
    # 非 Windows 系统下设置默认路径，使用 Docker 镜像中安装的路径
    set(ONNXRUNTIME_ROOTDIR "/opt/onnxruntime")
  endif()
endif()

# 添加包含路径
include_directories(
  ${ONNXRUNTIME_ROOTDIR}/include
  ${OPENVINO_ROOTDIR}/runtime/include
)

## Find catkin macros and libraries
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

find_package(CUDA REQUIRED)
find_package(OpenCV REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS message_runtime roscpp std_msgs # nodelet 
)

include_directories(
  include 
  ${catkin_INCLUDE_DIRS} 
  ${OpenCV_INCLUDE_DIRS}
  ${CUDA_INCLUDE_DIRS}
  /usr/local/cuda/include
)

# 官方发布的 tgz 库在 lib 文件夹下
link_directories(${ONNXRUNTIME_ROOTDIR}/lib)

add_executable(image_solver src/image_solver.cpp src/cuda_detector.cpp src/path_planner.cpp)
target_link_libraries(image_solver 
  ${catkin_LIBRARIES} 
  ${OpenCV_LIBRARIES}
  onnxruntime
)

install(DIRECTORY
  launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  # PATTERN ".git" EXCLUDE
)