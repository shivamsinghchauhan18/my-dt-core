#!/usr/bin/env bash
set -euo pipefail

# Helper script to start roscore, launch the minimal pipeline (file mode), and run health-check.
# Designed to run inside the Duckiebot container or any ROS-enabled environment.

VEH=${1:-blueduckie}
USE_FILE=${2:-true}
VIDEO_PATH=${3:-}

ROOT_DIR="$(cd "$(dirname "$0")/.." && pwd)"
LAUNCH_FILE="$ROOT_DIR/src/launchers/minimal_pipeline.launch"
HEALTH_NODE="$ROOT_DIR/src/launchers/health_check_node.py"

# Start roscore in background
if ! pgrep -f roscore >/dev/null 2>&1; then
  echo "Starting roscore..."
  roscore >/tmp/roscore.log 2>&1 &
  ROSCORE_PID=$!
  # wait for roscore
  sleep 2
else
  echo "roscore already running"
fi

# decide whether to run file camera or real device
if [ "$USE_FILE" = "true" ] || [ "$USE_FILE" = "True" ]; then
  echo "Launching minimal pipeline with file camera"
  roslaunch $LAUNCH_FILE veh:=$VEH use_file:=true &
else
  echo "Launching minimal pipeline with device camera"
  roslaunch $LAUNCH_FILE veh:=$VEH use_file:=false &
fi
LAUNCH_PID=$!

# wait a bit for nodes to start
sleep 5

# If VIDEO_PATH provided, set param for file node
if [ -n "$VIDEO_PATH" ]; then
  rosparam set /temp_camera_file/__name__/file "$VIDEO_PATH" || true
fi

# Run health check (will wait up to timeout seconds)
# start health check in background (run directly since it's not a ROS package)
PYTHON=$(which python3 || which python)
HEALTH_CHECK_SCRIPT="$(dirname "$0")/../src/launchers/health_check_node.py"
"$PYTHON" "$HEALTH_CHECK_SCRIPT" &

# Cleanup if this script started roscore or roslaunch (optional)
# kill $LAUNCH_PID || true
# if [ -n "${ROSCORE_PID:-}" ]; then kill $ROSCORE_PID || true; fi

echo "Completed run_minimal_pipeline.sh"
