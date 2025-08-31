#!/usr/bin/env bash

set -euo pipefail
SCRIPT_DIR="$( cd -- "$( dirname -- "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )"
source "$SCRIPT_DIR/common.sh"

require_vehicle "${1:-}"
cd_repo_root
source_ros
source_ws

# Preflight: make sure image_geometry is available (apt: ros-noetic-image-geometry)
if ! python3 -c 'import image_geometry' >/dev/null 2>&1; then
	echo "[preflight] Missing Python module 'image_geometry'." >&2
	echo "[preflight] Fix: install apt package 'ros-noetic-image-geometry' inside the running container, then rebuild/overlay the workspace." >&2
	echo "[preflight] Example (inside container): apt-get update && apt-get install -y ros-noetic-image-geometry && . /opt/ros/${ROS_DISTRO}/setup.bash && catkin_make" >&2
	# Continue anyway so roslaunch shows the exact traceback; comment 'exit 1' if you prefer soft warning.
	# exit 1
fi

roslaunch_echo duckietown_demos image_rectification.launch veh:="${VEHICLE_NAME}"
