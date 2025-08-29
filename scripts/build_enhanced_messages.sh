#!/bin/bash

# Build Enhanced Messages Script
# Ensures that duckietown_enhanced_msgs are properly built and available

set -e

echo "🔧 Building duckietown_enhanced_msgs..."
echo "======================================="

# Detect a catkin workspace (prefer overlay if present)
for CANDIDATE in \
    "/code/enhanced_ws" \
    "/code/catkin_ws"
do
    if [ -d "$CANDIDATE/src" ]; then
        CATKIN_WS="$CANDIDATE"
        break
    fi
done

if [ -z "$CATKIN_WS" ]; then
    echo "❌ No catkin workspace found at /code/enhanced_ws or /code/catkin_ws"
    echo "   This script should be run inside the Duckietown container"
    exit 1
fi

echo "✓ Using workspace: $CATKIN_WS"

PKG_PATHS=(
    "$CATKIN_WS/src/my-dt-core/packages/duckietown_enhanced_msgs"
    "$CATKIN_WS/src/dt-duckiebot-interface/my-dt-core/packages/duckietown_enhanced_msgs"
    "$CATKIN_WS/src/duckietown_enhanced_msgs"
)

PKG_DIR=""
for p in "${PKG_PATHS[@]}"; do
    if [ -d "$p" ]; then
        PKG_DIR="$p"
        break
    fi
done

if [ -z "$PKG_DIR" ]; then
    echo "❌ duckietown_enhanced_msgs package not found in workspace src"
    exit 1
fi

echo "✓ Found duckietown_enhanced_msgs at: $PKG_DIR"

cd "$CATKIN_WS"

# Source ROS environment
if [ -f /opt/ros/noetic/setup.bash ]; then
    # shellcheck disable=SC1091
    source /opt/ros/noetic/setup.bash
elif [ -f /opt/ros/melodic/setup.bash ]; then
    # shellcheck disable=SC1091
    source /opt/ros/melodic/setup.bash
fi

echo "🏗️  Building duckietown_enhanced_msgs..."
catkin build duckietown_enhanced_msgs

echo "   Sourcing workspace..."
# shellcheck disable=SC1091
source "$CATKIN_WS/devel/setup.bash"

echo "🔍 Verifying enhanced messages..."
python3 - <<'PY'
import sys
try:
        from duckietown_enhanced_msgs.msg import (
                SafetyStatus, AdvancedLanePose, ObjectDetectionArray,
                LaneCurve, LaneCurves, ObjectDetection
        )
        print('✓ SafetyStatus available')
        print('✓ AdvancedLanePose available')
        print('✓ ObjectDetectionArray available')
        print('✓ LaneCurve available')
        print('✓ LaneCurves available')
        print('✓ ObjectDetection available')
        print('✅ duckietown_enhanced_msgs ready!')
except Exception as e:
        print('❌ Verification failed:', e)
        sys.exit(1)
PY

echo "✅ Build completed successfully."
