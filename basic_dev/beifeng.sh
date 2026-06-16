#!/bin/bash

set -e

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"

mkdir -p "$SCRIPT_DIR/build" "$SCRIPT_DIR/devel"

if [ -z "$DISPLAY" ]; then
  echo "DISPLAY is empty. Please run this from a desktop session with X11."
  exit 1
fi

echo "Granting local Docker X11 access..."
xhost +local:docker >/dev/null 2>&1 || true

docker run -it --net host --rm \
  --name basic_dev_dev_gui \
  --entrypoint /bin/bash \
  -e DISPLAY="$DISPLAY" \
  -e QT_X11_NO_MITSHM=1 \
  -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
  -v "$SCRIPT_DIR/src:/basic_dev/src" \
  -v "$SCRIPT_DIR/build:/basic_dev/build" \
  -v "$SCRIPT_DIR/devel:/basic_dev/devel" \
  -v "$SCRIPT_DIR/build_basic_dev_dev.sh:/usr/local/bin/build_basic_dev_dev.sh" \
  -w /basic_dev \
  basic_dev \
  -c '
    source /opt/ros/noetic/setup.bash
    
    echo "Starting catkin_make"
    catkin_make
    
    echo "Sourcing workspace"
    source ./devel/setup.bash
    
    echo "Starting launch"
    roslaunch imu_gps_fusion imu_gps_fusion.launch
    
    echo "Launch finished or aborted. Entering interactive bash shell..."
    exec bash
  '
