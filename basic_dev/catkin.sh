#!/bin/bash

echo "Starting build process for basic_dev ROS workspace..."

echo "Starting build process for basic_dev ROS workspace..."

source /opt/ros/noetic/setup.bash

build_package() {
  local package_name="$1"
  shift

  catkin_make -DCATKIN_WHITELIST_PACKAGES="${package_name}" "$@"
  source ./devel/setup.bash
}

# build_package "livox_ros_driver"
# build_package "pointcloud2_to_livox"
# build_package "pose_utils"
# build_package "quadrotor_msgs"
# build_package "airsim_ros"
# build_package "point_lio" -j3
# build_package "multi_map_server"
# build_package "pwm_se3_controller"
# build_package "plan_env"
# build_package "path_searching"
# build_package "traj_utils"
# build_package "bspline_opt"
# build_package "ego_planner" -j4
# build_package "scp_mpc_trajectory"
# build_package "imu_gps_fusion"
# build_package "lidar_solver"
# build_package "AB_planner"

build_package "pwm_se3_controller"
build_package "pwm_adrc_controller"
build_package "imu_gps_fusion"
build_package "lidar_solver"
build_package "AB_planner"

#catkin_make -DCATKIN_WHITELIST_PACKAGES="pwm_se3_controller"

echo "Build complete"
