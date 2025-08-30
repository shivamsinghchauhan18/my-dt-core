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
catkin_make

echo "[build] Done. To use this workspace: source devel/setup.bash"
