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

docker run -it --net host --rm --gpus all --ipc=host \
  --name basic_dev_dev_gui \
  --entrypoint /bin/bash \
  -e DISPLAY="$DISPLAY" \
  -e QT_X11_NO_MITSHM=1 \
  -e NVIDIA_DRIVER_CAPABILITIES=all \
  -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
  -v "$SCRIPT_DIR/src:/basic_dev/src" \
  -v "$SCRIPT_DIR/build:/basic_dev/build" \
  -v "$SCRIPT_DIR/devel:/basic_dev/devel" \
  -v "$SCRIPT_DIR/build_basic_dev_dev.sh:/basic_dev/build_basic_dev_dev.sh" \
  -v "$SCRIPT_DIR/catkin.sh:/basic_dev/catkin.sh" \
  -v "$SCRIPT_DIR/entrypoint.sh:/basic_dev/entrypoint.sh" \
  -v /home/ake/RMUA/IntelligentUAVChampionshipBase/basic_dev/src/image_solver/model:/basic_dev/model \
  -v "$SCRIPT_DIR/waypoint.txt:/basic_dev/waypoint.txt" \
  -w /basic_dev \
  basic_dev \
  -lc 'source /opt/ros/noetic/setup.bash && bash'
