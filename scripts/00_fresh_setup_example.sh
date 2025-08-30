#!/usr/bin/env bash

# Example of a fresh setup flow on a machine with ROS already installed.
# This is informational and may be adapted; it does not clone this repo automatically to avoid destructive actions.

set -euo pipefail

echo "1) git clone <this-repo-url> ~/my-dt-core"
echo "2) cd ~/my-dt-core && ./scripts/01_build.sh"
echo "3) source ./scripts/02_source.sh (or 'source devel/setup.bash')"
echo "4) export VEHICLE_NAME=<your_bot>"
echo "5) ./scripts/20_lane_following.sh  # or others in scripts/"
