#!/bin/bash

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
IMAGE_NAME="basic_dev"
CONTAINER_NAME="basic_dev_build_once"

mkdir -p "$SCRIPT_DIR/build" "$SCRIPT_DIR/devel"

if ! command -v docker >/dev/null 2>&1; then
	echo "docker command not found. Please install Docker first."
	exit 1
fi

echo "Starting one-shot build container: $CONTAINER_NAME"

docker run --rm --net host \
	--name "$CONTAINER_NAME" \
	--entrypoint /bin/bash \
	-v "$SCRIPT_DIR/src:/basic_dev/src" \
	-v "$SCRIPT_DIR/build:/basic_dev/build" \
	-v "$SCRIPT_DIR/devel:/basic_dev/devel" \
	-w /basic_dev \
	"$IMAGE_NAME" \
	-lc 'source /opt/ros/noetic/setup.bash && catkin_make'

echo "Docker build finished, container exited."
