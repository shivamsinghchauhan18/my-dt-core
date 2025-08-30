#!/usr/bin/env bash

# Common helpers for building and launching this catkin workspace without overlays.
# Usage: source this file or let the per-feature scripts source it automatically.

set -euo pipefail

# Resolve repository root (one level up from this scripts directory)
SCRIPT_DIR="$( cd -- "$( dirname -- "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )"
REPO_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"

# Ensure we are in the repo root when needed
cd_repo_root() {
  cd "$REPO_ROOT"
}

# Source ROS and this workspace (idempotent)
source_ros() {
  if command -v rosversion >/dev/null 2>&1; then
    return 0
  fi
  local distro=${ROS_DISTRO:-noetic}
  local ros_setup="/opt/ros/${distro}/setup.bash"
  if [ -f "$ros_setup" ]; then
    # shellcheck disable=SC1090
    source "$ros_setup"
  else
    echo "[common] ROS not found at $ros_setup. Ensure ROS is installed and ROS_DISTRO is set." >&2
    exit 1
  fi
}

source_ws() {
  local ws_setup="$REPO_ROOT/devel/setup.bash"
  if [ ! -f "$ws_setup" ]; then
    echo "[common] Workspace not built yet (missing devel/setup.bash). Run scripts/01_build.sh first." >&2
    exit 1
  fi
  # shellcheck disable=SC1090
  source "$ws_setup"
}

# Determine vehicle name from $VEHICLE_NAME or first CLI arg
require_vehicle() {
  if [ "${VEHICLE_NAME:-}" != "" ]; then
    echo "[common] Using VEHICLE_NAME=$VEHICLE_NAME"
    return 0
  fi
  if [ "${1:-}" != "" ]; then
    export VEHICLE_NAME="$1"
    echo "[common] Using VEHICLE_NAME=$VEHICLE_NAME"
    return 0
  fi
  echo "Usage: $0 <VEHICLE_NAME> (or export VEHICLE_NAME first)" >&2
  exit 2
}

# Small helper to run roslaunch with echo'd command
roslaunch_echo() {
  echo "> roslaunch $*"
  roslaunch "$@"
}
