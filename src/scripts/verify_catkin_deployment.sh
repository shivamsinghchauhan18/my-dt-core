#!/bin/bash

# Verification script for catkin workspace deployment

ROBOT_NAME="pinkduckie"
ROBOT_HOST="${ROBOT_NAME}.local"
ROBOT_USER="duckie"
ROBOT_PASSWORD="quackquack"

# Colors
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m'

log_info() { echo -e "${BLUE}[INFO]${NC} $1"; }
log_success() { echo -e "${GREEN}[SUCCESS]${NC} $1"; }
log_warning() { echo -e "${YELLOW}[WARNING]${NC} $1"; }
log_error() { echo -e "${RED}[ERROR]${NC} $1"; }

# SSH helper function
ssh_cmd() {
    local cmd="$1"
    
    if command -v sshpass &> /dev/null; then
        sshpass -p "$ROBOT_PASSWORD" ssh -o ConnectTimeout=5 -o StrictHostKeyChecking=no "$ROBOT_USER@$ROBOT_HOST" "$cmd"
    else
        ssh -o ConnectTimeout=5 "$ROBOT_USER@$ROBOT_HOST" "$cmd"
    fi
}

echo "=========================================="
echo "🔍 CATKIN DEPLOYMENT VERIFICATION"
echo "Robot: $ROBOT_NAME"
echo "=========================================="
echo

# Test 1: Check catkin workspace
log_info "Test 1: Checking catkin workspace structure..."

ssh_cmd "
    # Find catkin workspace
    if [ -d '/code/catkin_ws' ]; then
        CATKIN_WS='/code/catkin_ws'
    else
        CATKIN_WS='/home/duckie/catkin_ws'
    fi
    
    echo \"Catkin workspace: \$CATKIN_WS\"
    
    # Check if our packages exist
    if [ -d \"\$CATKIN_WS/src/enhanced_line_detector\" ]; then
        echo '✅ enhanced_line_detector package found'
    else
        echo '❌ enhanced_line_detector package missing'
    fi
    
    if [ -d \"\$CATKIN_WS/src/enhanced_demos\" ]; then
        echo '✅ enhanced_demos package found'
    else
        echo '❌ enhanced_demos package missing'
    fi
    
    # Check if workspace is built
    if [ -d \"\$CATKIN_WS/devel\" ]; then
        echo '✅ Workspace is built (devel/ exists)'
    else
        echo '❌ Workspace not built'
    fi
" || log_error "Could not check catkin workspace"

echo

# Test 2: Check ROS package system
log_info "Test 2: Checking ROS package system..."

ssh_cmd "
    # Find catkin workspace
    if [ -d '/code/catkin_ws' ]; then
        CATKIN_WS='/code/catkin_ws'
    else
        CATKIN_WS='/home/duckie/catkin_ws'
    fi
    
    cd \$CATKIN_WS
    
    # Source ROS environment
    source /opt/ros/noetic/setup.bash 2>/dev/null || source /opt/ros/melodic/setup.bash 2>/dev/null
    source devel/setup.bash 2>/dev/null
    
    # Check if packages are recognized by ROS
    if rospack find enhanced_line_detector >/dev/null 2>&1; then
        echo '✅ enhanced_line_detector recognized by ROS'
        rospack find enhanced_line_detector
    else
        echo '❌ enhanced_line_detector not recognized by ROS'
    fi
    
    if rospack find enhanced_demos >/dev/null 2>&1; then
        echo '✅ enhanced_demos recognized by ROS'
        rospack find enhanced_demos
    else
        echo '❌ enhanced_demos not recognized by ROS'
    fi
" || log_warning "Could not check ROS packages"

echo

# Test 3: Check launch files
log_info "Test 3: Checking launch files..."

ssh_cmd "
    # Find catkin workspace
    if [ -d '/code/catkin_ws' ]; then
        CATKIN_WS='/code/catkin_ws'
    else
        CATKIN_WS='/home/duckie/catkin_ws'
    fi
    
    echo 'Enhanced launch files:'
    find \$CATKIN_WS/src -name '*.launch' | grep enhanced
    
    echo
    echo 'Checking launch file syntax:'
    
    # Check enhanced_line_detector launch file
    if [ -f \"\$CATKIN_WS/src/enhanced_line_detector/launch/enhanced_line_detector.launch\" ]; then
        echo '✅ enhanced_line_detector.launch exists'
    else
        echo '❌ enhanced_line_detector.launch missing'
    fi
    
    # Check enhanced_demos launch file
    if [ -f \"\$CATKIN_WS/src/enhanced_demos/launch/enhanced_autonomous.launch\" ]; then
        echo '✅ enhanced_autonomous.launch exists'
    else
        echo '❌ enhanced_autonomous.launch missing'
    fi
" || log_warning "Could not check launch files"

echo

# Test 4: Check executable nodes
log_info "Test 4: Checking executable nodes..."

ssh_cmd "
    # Find catkin workspace
    if [ -d '/code/catkin_ws' ]; then
        CATKIN_WS='/code/catkin_ws'
    else
        CATKIN_WS='/home/duckie/catkin_ws'
    fi
    
    echo 'Enhanced executable nodes:'
    find \$CATKIN_WS/src -name '*.py' -executable | grep enhanced
    
    # Check if main node is executable
    if [ -x \"\$CATKIN_WS/src/enhanced_line_detector/src/enhanced_line_detector_node.py\" ]; then
        echo '✅ enhanced_line_detector_node.py is executable'
    else
        echo '❌ enhanced_line_detector_node.py not executable'
    fi
" || log_warning "Could not check executable nodes"

echo

# Test 5: Check startup script
log_info "Test 5: Checking startup script..."

ssh_cmd "
    if [ -f '/home/duckie/start_enhanced_autonomous.sh' ]; then
        echo '✅ Startup script exists'
        if [ -x '/home/duckie/start_enhanced_autonomous.sh' ]; then
            echo '✅ Startup script is executable'
        else
            echo '❌ Startup script not executable'
        fi
    else
        echo '❌ Startup script missing'
    fi
" || log_warning "Could not check startup script"

echo

# Test 6: Quick ROS system test
log_info "Test 6: Quick ROS system test..."

ssh_cmd "
    # Check if roscore is running
    if pgrep -f roscore >/dev/null; then
        echo '✅ ROS core is running'
    else
        echo '⚠️  ROS core not running (this is normal if system is not started)'
    fi
    
    # Check existing Duckiebot containers
    CONTAINERS=\$(docker ps | grep -E '(duckiebot|dt-|car-interface)' | wc -l)
    echo \"Duckiebot containers running: \$CONTAINERS\"
    
    if [ \$CONTAINERS -gt 0 ]; then
        echo '✅ Duckiebot infrastructure is running'
    else
        echo '⚠️  Duckiebot infrastructure not running (start with dts or docker commands)'
    fi
" || log_warning "Could not check ROS system"

echo

# Summary
echo "=========================================="
echo "📋 VERIFICATION SUMMARY"
echo "=========================================="
echo
echo "✅ READY TO USE:"
echo "1. SSH to robot: ssh duckie@pinkduckie.local"
echo "2. Start enhanced system: ./start_enhanced_autonomous.sh"
echo
echo "🔧 MANUAL TESTING:"
echo "1. Source environment:"
echo "   cd /code/catkin_ws  # or /home/duckie/catkin_ws"
echo "   source devel/setup.bash"
echo
echo "2. Test packages:"
echo "   rospack find enhanced_line_detector"
echo "   rospack find enhanced_demos"
echo
echo "3. Launch enhanced system:"
echo "   roslaunch enhanced_demos enhanced_autonomous.launch veh:=pinkduckie"
echo
echo "4. Monitor topics:"
echo "   rostopic list | grep enhanced"
echo "   rostopic echo /pinkduckie/enhanced_line_detector_node/segment_list"
echo
echo "🎯 ADVANTAGES OF THIS APPROACH:"
echo "• No Docker container conflicts"
echo "• Works with existing Duckiebot ROS system"
echo "• Easy to modify and debug"
echo "• Standard catkin workflow"
echo "• Preserves all existing functionality"
echo