#!/bin/bash

set -e

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"

mkdir -p "$SCRIPT_DIR/build" "$SCRIPT_DIR/devel" "$SCRIPT_DIR/plots"

if [ -z "$DISPLAY" ]; then
  echo "DISPLAY is empty. Please run this from a desktop session with X11."
  exit 1
fi

echo "Granting local Docker X11 access..."
xhost +local:docker >/dev/null 2>&1 || true

# 1. 检查是否存在同名容器，如果存在则先停止它
if [ "$(docker ps -aq -f name=basic_dev_dev_gui)" ]; then
    echo "Stopping and removing existing container..."
    docker rm -f basic_dev_dev_gui >/dev/null 2>&1 || true
    while [ "$(docker ps -aq -f name=basic_dev_dev_gui)" ]; do
        echo "Waiting for old basic_dev_dev_gui removal to finish..."
        sleep 0.2
    done
fi

# 启动后台容器（不再在子终端里 stop/rm）
docker run -d --net host --gpus all --ipc=host \
  --name basic_dev_dev_gui \
  --entrypoint /bin/bash \
  -e DISPLAY="$DISPLAY" \
  -e QT_X11_NO_MITSHM=1 \
  -e NVIDIA_DRIVER_CAPABILITIES=all \
  -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
  -v "$SCRIPT_DIR/src:/basic_dev/src" \
  -v "$SCRIPT_DIR/build:/basic_dev/build" \
  -v "$SCRIPT_DIR/devel:/basic_dev/devel" \
  -v "$SCRIPT_DIR/plots:/basic_dev/plots" \
  -v "$SCRIPT_DIR/note/score_pictures:/basic_dev/score_pictures" \
  -v "$SCRIPT_DIR/src/image_solver/model:/basic_dev/model" \
  -v "$SCRIPT_DIR/waypoint.txt:/basic_dev/waypoint.txt" \
  -w /basic_dev \
  basic_dev \
  -c 'source /opt/ros/noetic/setup.bash && tail -f /dev/null'

export use_rviz=true
export controller="${UAV_CONTROLLER:-adrc}"
export start_stack="${START_STACK:-full}"
if [ -n "${ADRC_INPUT:-}" ]; then
  export adrc_input="${ADRC_INPUT}"
elif [ "$start_stack" = "manual" ]; then
  export adrc_input="keyboard"
else
  export adrc_input="planner"
fi
export adrc_z_offset="${ADRC_Z_OFFSET:-0.5}"

case "$start_stack" in
  full|manual)
    ;;
  *)
    echo "Unknown START_STACK=$start_stack. Use 'full' or 'manual'." >&2
    exit 1
    ;;
esac

case "$adrc_input" in
  planner|keyboard)
    ;;
  *)
    echo "Unknown ADRC_INPUT=$adrc_input. Use 'planner' or 'keyboard'." >&2
    exit 1
    ;;
esac

if [ "$start_stack" = "manual" ] && [ "$adrc_input" = "planner" ]; then
  echo "START_STACK=manual skips AB_planner, so ADRC planner input would receive no /pose_cmd. Use ADRC_INPUT=keyboard or START_STACK=full." >&2
  exit 1
fi

case "$controller" in
  adrc|pwm_adrc_controller)
    export controller_pkg=pwm_adrc_controller
    export controller_node=pwm_adrc_controller
    ;;
  se3|pwm_se3_controller)
    export controller_pkg=pwm_se3_controller
    export controller_node=pwm_se3_controller
    ;;
  *)
    echo "Unknown UAV_CONTROLLER=$controller. Use 'adrc' or 'se3'." >&2
    exit 1
    ;;
esac

echo "Selected controller: ${controller_pkg}/${controller_node}"
echo "Selected stack: ${start_stack}"
echo "Selected ADRC input: ${adrc_input}"
echo "Selected ADRC z offset: ${adrc_z_offset}"

echo "Building selected controller so the running node matches current source..."
docker exec basic_dev_dev_gui bash -lc "
    set -e
    source /opt/ros/noetic/setup.bash
    if [ -f /basic_dev/devel/setup.bash ]; then
        source /basic_dev/devel/setup.bash
    fi
    node_path=/basic_dev/devel/lib/${controller_pkg}/${controller_node}
    cd /basic_dev
    catkin_make --only-pkg-with-deps ${controller_pkg}
    source /basic_dev/devel/setup.bash
    if [ ! -x \"\$node_path\" ]; then
        echo \"Build finished, but executable is still missing: \$node_path\" >&2
        exit 1
    fi
"

gnome-terminal -- bash -c '
    docker exec -it basic_dev_dev_gui bash -c "
        set -e
        source /opt/ros/noetic/setup.bash
        source /basic_dev/devel/setup.bash
        echo \"Waiting for AirSim IMU topic before enabling simulated time...\"
        until rostopic echo -n 1 /airsim_node/drone_1/imu/imu >/dev/null 2>&1; do
          sleep 0.2
        done
        rosparam set /use_sim_time true
        if [ \"${controller_pkg}\" = \"pwm_adrc_controller\" ]; then
          if [ \"${adrc_input}\" = \"keyboard\" ]; then
            rosparam set /use_planner_input false
          else
            rosparam set /use_planner_input true
          fi
          rosparam set /planner_cmd_topic /pose_cmd
          rosparam set /planner_z_offset ${adrc_z_offset}
        fi
        echo \"AirSim IMU is available. Starting ${controller_pkg} with IMU-driven /clock.\"
        rosrun ${controller_pkg} ${controller_node}
    ";
	status=$?
	echo "[controller:${controller_pkg}] exited with status ${status}. Container basic_dev_dev_gui is kept running for debugging.";
    exec bash
' &
PID1=$!

gnome-terminal -- bash -c '
    docker exec -it basic_dev_dev_gui bash -c "
        set -e
        source /opt/ros/noetic/setup.bash
        source /basic_dev/devel/setup.bash
        if [ ! -s /basic_dev/waypoint.txt ]; then
          echo \"Missing or empty waypoint file: /basic_dev/waypoint.txt\" >&2
          exit 1
        fi
        echo \"Waiting for AirSim IMU topic before enabling simulated time...\"
        until rostopic echo -n 1 /airsim_node/drone_1/imu/imu >/dev/null 2>&1; do
          sleep 0.2
        done
        rosparam set /use_sim_time true
        echo \"AirSim IMU is available. Starting imu_gps_fusion with IMU-driven /clock.\"
        roslaunch imu_gps_fusion imu_gps_fusion.launch enable_sim_time:=true use_rviz:=${use_rviz}
    ";
	status=$?
	echo "[imu_gps_fusion] exited with status ${status}. Container basic_dev_dev_gui is kept running for debugging.";
    exec bash
' &
PID2=$!

if [ "$start_stack" = "full" ]; then
gnome-terminal -- bash -c '
    docker exec -it basic_dev_dev_gui bash -c "
        set -e
        source /opt/ros/noetic/setup.bash
        source /basic_dev/devel/setup.bash
        if [ ! -s /basic_dev/waypoint.txt ]; then
          echo \"Missing or empty waypoint file: /basic_dev/waypoint.txt\" >&2
          exit 1
        fi
        echo \"Waiting for AirSim IMU topic before enabling simulated time...\"
        until rostopic echo -n 1 /airsim_node/drone_1/imu/imu >/dev/null 2>&1; do
          sleep 0.2
        done
        rosparam set /use_sim_time true
        echo \"AirSim IMU is available. Starting AB_planner with IMU-driven /clock.\"
        roslaunch AB_planner AB_planner.launch use_rviz:=${use_rviz}
    ";
	status=$?
	echo "[AB_planner] exited with status ${status}. Container basic_dev_dev_gui is kept running for debugging.";
    exec bash
' &
PID3=$!

gnome-terminal -- bash -c '
    docker exec -it basic_dev_dev_gui bash -c "
        set -e
        source /opt/ros/noetic/setup.bash
        source /basic_dev/devel/setup.bash
        if [ ! -s /basic_dev/waypoint.txt ]; then
          echo \"Missing or empty waypoint file: /basic_dev/waypoint.txt\" >&2
          exit 1
        fi
        echo \"Waiting for AirSim IMU topic before enabling simulated time...\"
        until rostopic echo -n 1 /airsim_node/drone_1/imu/imu >/dev/null 2>&1; do
          sleep 0.2
        done
        rosparam set /use_sim_time true
        echo \"AirSim IMU is available. Starting lidar_solver with IMU-driven /clock.\"
        roslaunch lidar_solver lidar_solver.launch use_rviz:=${use_rviz}
    ";
	status=$?
	echo "[lidar_solver] exited with status ${status}. Container basic_dev_dev_gui is kept running for debugging.";
    exec bash
' &
PID4=$!
else
  echo "START_STACK=manual: skipped AB_planner and lidar_solver. Only controller + imu_gps_fusion are started."
fi

# 这里的 exec bash 是为了让终端在命令执行完后保持打开状态，你可以根据需要调整
# 如果你也希望主终端可以进入容器交互，就执行这句：
# docker exec -it basic_dev_dev_gui bash
