#!/bin/bash
set -e

source /opt/ros/noetic/setup.bash

RUNTIME_PACKAGES=(pwm_se3_controller pwm_adrc_controller imu_gps_fusion AB_planner lidar_solver)

build_runtime_packages() {
  catkin_make --only-pkg-with-deps "${RUNTIME_PACKAGES[@]}"
}

ensure_node() {
  local package_name="$1"
  local node_name="$2"
  local node_path="/basic_dev/devel/lib/${package_name}/${node_name}"

  if [ -x "$node_path" ]; then
    return 0
  fi

  echo "Missing executable node: ${node_path}" >&2
  echo "Building required ROS packages in /basic_dev before launch..." >&2
  cd /basic_dev
  build_runtime_packages
  source /basic_dev/devel/setup.bash

  if [ ! -x "$node_path" ]; then
    echo "Build finished, but node is still missing: ${node_path}" >&2
    exit 1
  fi
}

if [ ! -f /basic_dev/devel/setup.bash ]; then
  echo "Missing /basic_dev/devel/setup.bash. Building runtime ROS packages..." >&2
  cd /basic_dev
  build_runtime_packages
fi

source /basic_dev/devel/setup.bash

ensure_node imu_gps_fusion imu_clock_publisher
ensure_node imu_gps_fusion imu_gps_fusion
ensure_node pwm_se3_controller pwm_se3_controller
ensure_node pwm_adrc_controller pwm_adrc_controller
ensure_node AB_planner AB_planner_node
ensure_node lidar_solver lidar_solver

if [ ! -s /basic_dev/waypoint.txt ]; then
  echo "Missing or empty waypoint file: /basic_dev/waypoint.txt" >&2
  exit 1
fi

echo "Waiting for AirSim IMU topic before enabling simulated time..."
until rostopic echo -n 1 /airsim_node/drone_1/imu/imu >/dev/null 2>&1; do
  sleep 0.2
done

rosparam set /use_sim_time true
echo "AirSim IMU is available. Starting pwm_adrc_controller, imu_gps_fusion, AB_planner and lidar_solver with IMU-driven /clock."

roslaunch imu_gps_fusion drone_fly.launch \
  enable_sim_time:=true \
  start_pwm_adrc:=true \
  start_pwm_se3:=false \
  start_imu_gps_fusion:=true \
  start_ab_planner:=true \
  start_lidar_solver:=true \
  use_rviz:=false \
  start_qianli:=false &
launch_pid=$!

cleanup() {
  kill "$launch_pid" 2>/dev/null || true
  wait "$launch_pid" 2>/dev/null || true
}
trap cleanup EXIT INT TERM

wait "$launch_pid"
