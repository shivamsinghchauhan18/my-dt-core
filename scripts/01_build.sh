#!/usr/bin/env bash

# Build the catkin workspace in-place (no overlay).

set -euo pipefail

SCRIPT_DIR="$( cd -- "$( dirname -- "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )"
# shellcheck source=common.sh
source "$SCRIPT_DIR/common.sh"

cd_repo_root
source_ros

if [ "${1:-}" = "clean" ]; then
  echo "[build] Cleaning build/ devel/ logs/"
  rm -rf build devel logs
fi

echo "[build] Running catkin_make at $REPO_ROOT"
# Ensure message packages are not ignored
if [ -f src/packages/duckietown_msgs/CATKIN_IGNORE ]; then
  echo "[build] Removing CATKIN_IGNORE from duckietown_msgs to allow message generation"
  rm -f src/packages/duckietown_msgs/CATKIN_IGNORE
fi
catkin_make

echo "[build] Done. To use this workspace: source devel/setup.bash"
