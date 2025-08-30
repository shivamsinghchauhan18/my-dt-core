#!/usr/bin/env bash

# Source ROS and this workspace, print a quick status.

set -euo pipefail

SCRIPT_DIR="$( cd -- "$( dirname -- "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )"
# shellcheck source=common.sh
source "$SCRIPT_DIR/common.sh"

cd_repo_root
source_ros
source_ws

echo "[source] ROS_DISTRO=${ROS_DISTRO:-unknown}"
echo "[source] ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH"

# Quick sanity: ensure a few packages are discoverable
for pkg in duckietown_demos lane_control line_detector; do
  if ! rospack find "$pkg" >/dev/null 2>&1; then
    echo "[source] Warning: package '$pkg' not found by rospack" >&2
  fi
done

echo "[source] Environment ready. You can now roslaunch."
