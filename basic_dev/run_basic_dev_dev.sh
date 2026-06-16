#!/bin/bash

set -e

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"

mkdir -p "$SCRIPT_DIR/build" "$SCRIPT_DIR/devel"

docker run -it --net host --rm \
  --name basic_dev_dev \
  --entrypoint /bin/bash \
  -v "$SCRIPT_DIR/src:/basic_dev/src" \
  -v "$SCRIPT_DIR/build:/basic_dev/build" \
  -v "$SCRIPT_DIR/devel:/basic_dev/devel" \
  -v "$SCRIPT_DIR/build_basic_dev_dev.sh:/usr/local/bin/build_basic_dev_dev.sh" \
  -w /basic_dev \
  basic_dev \
  -lc 'source /opt/ros/noetic/setup.bash && bash'