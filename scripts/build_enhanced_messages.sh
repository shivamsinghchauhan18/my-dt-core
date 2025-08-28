#!/bin/bash

# Build Enhanced Messages Script
# Ensures that enhanced duckietown_msgs are properly built and available

set -e

echo "🔧 Building Enhanced Duckietown Messages..."
echo "=========================================="

# Check if we're in a catkin workspace
if [ -d "/code/catkin_ws" ]; then
    CATKIN_WS="/code/catkin_ws"
    echo "✓ Found catkin workspace at: $CATKIN_WS"
else
    echo "❌ No catkin workspace found at /code/catkin_ws"
    echo "   This script should be run inside a Duckietown Docker container"
    exit 1
fi

# Check if duckietown_msgs package exists
if [ ! -d "$CATKIN_WS/src/dt-core/packages/duckietown_msgs" ]; then
    echo "❌ duckietown_msgs package not found"
    echo "   Expected at: $CATKIN_WS/src/dt-core/packages/duckietown_msgs"
    exit 1
fi

echo "✓ Found duckietown_msgs package"

# Check for enhanced messages
ENHANCED_MSG_DIR="$CATKIN_WS/src/dt-core/packages/duckietown_msgs/msg/enhanced"
if [ -d "$ENHANCED_MSG_DIR" ]; then
    echo "✓ Found enhanced messages directory"
    echo "   Enhanced messages available:"
    ls -la "$ENHANCED_MSG_DIR"/*.msg | sed 's/.*\//   - /'
else
    echo "❌ Enhanced messages directory not found"
    exit 1
fi

# Build the duckietown_msgs package
echo ""
echo "🏗️  Building duckietown_msgs package..."
cd "$CATKIN_WS"

# Source ROS environment
source /opt/ros/noetic/setup.bash

# Clean and build duckietown_msgs
echo "   Cleaning previous build..."
catkin clean duckietown_msgs -y || true

echo "   Building duckietown_msgs..."
catkin build duckietown_msgs

# Source the workspace
echo "   Sourcing workspace..."
source devel/setup.bash

# Verify the messages are available
echo ""
echo "🔍 Verifying enhanced messages..."
python3 -c "
import sys
sys.path.insert(0, '$CATKIN_WS/devel/lib/python3/dist-packages')
try:
    from duckietown_msgs.msg import SafetyStatus, AdvancedLanePose, ObjectDetectionArray
    print('✓ SafetyStatus message available')
    print('✓ AdvancedLanePose message available')
    print('✓ ObjectDetectionArray message available')
    print('✅ All enhanced messages are working!')
except ImportError as e:
    print(f'❌ Import error: {e}')
    print('Enhanced messages may not be properly built')
    sys.exit(1)
"

echo ""
echo "✅ Enhanced duckietown_msgs build completed successfully!"
echo "   You can now run the enhanced autonomous system."
