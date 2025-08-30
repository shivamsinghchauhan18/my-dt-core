#!/usr/bin/env bash

set -euo pipefail
SCRIPT_DIR="$( cd -- "$( dirname -- "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )"
source "$SCRIPT_DIR/common.sh"

require_vehicle "${1:-}"
cd_repo_root
source_ros
source_ws

# Use the predefined demo which includes master.launch wiring for LF
roslaunch_echo duckietown_demos lane_following.launch veh:="${VEHICLE_NAME}" visualization:=true
