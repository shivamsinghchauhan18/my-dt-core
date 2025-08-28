#!/bin/bash

# Pre-deployment Check Script for Advanced Autonomous Duckietown System
# Validates system readiness before deployment to pinkduckie (DB21J)

set -e

ROBOT_NAME="pinkduckie"
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(dirname "$SCRIPT_DIR")"

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

check_count=0
pass_count=0
warning_count=0
error_count=0

run_check() {
    local check_name="$1"
    local check_command="$2"
    
    ((check_count++))
    echo -n "[$check_count] Checking $check_name... "
    
    if eval "$check_command" &>/dev/null; then
        echo -e "${GREEN}PASS${NC}"
        ((pass_count++))
        return 0
    else
        echo -e "${RED}FAIL${NC}"
        ((error_count++))
        return 1
    fi
}

run_check_with_warning() {
    local check_name="$1"
    local check_command="$2"
    
    ((check_count++))
    echo -n "[$check_count] Checking $check_name... "
    
    if eval "$check_command" &>/dev/null; then
        echo -e "${GREEN}PASS${NC}"
        ((pass_count++))
        return 0
    else
        echo -e "${YELLOW}WARNING${NC}"
        ((warning_count++))
        return 1
    fi
}

echo "=============================================="
echo "Pre-deployment System Check"
echo "Robot: $ROBOT_NAME (DB21J)"
echo "=============================================="
echo

# Development environment checks
echo "=== Development Environment ==="
run_check "macOS system" "[[ \"\$OSTYPE\" == \"darwin\"* ]]"
run_check "Docker installation" "command -v docker"
run_check "Docker daemon running" "docker info"
run_check "Docker buildx available" "docker buildx version"
run_check "SSH client available" "command -v ssh"
run_check "rsync available" "command -v rsync"
echo

# Project structure checks
echo "=== Project Structure ==="
run_check "Project root directory" "test -d \"$PROJECT_ROOT\""
run_check "Packages directory" "test -d \"$PROJECT_ROOT/packages\""
run_check "Scripts directory" "test -d \"$PROJECT_ROOT/scripts\""
run_check "Robot configs directory" "test -d \"$PROJECT_ROOT/robot_configs\""
run_check "pinkduckie config file" "test -f \"$PROJECT_ROOT/robot_configs/pinkduckie.yaml\""
echo

# Dependency files
echo "=== Dependency Files ==="
run_check "Python dependencies" "test -f \"$PROJECT_ROOT/dependencies-py3.txt\""
run_check "Duckietown dependencies" "test -f \"$PROJECT_ROOT/dependencies-py3.dt.txt\""
run_check "APT dependencies" "test -f \"$PROJECT_ROOT/dependencies-apt.txt\""
run_check "Configuration file" "test -f \"$PROJECT_ROOT/configurations.yaml\""
run_check "Dockerfile" "test -f \"$PROJECT_ROOT/Dockerfile\""
run_check "Makefile" "test -f \"$PROJECT_ROOT/Makefile\""
echo

# Advanced dependencies
echo "=== Advanced Dependencies ==="
run_check "PyTorch dependency" "grep -q 'torch' \"$PROJECT_ROOT/dependencies-py3.txt\""
run_check "YOLO dependency" "grep -q 'ultralytics' \"$PROJECT_ROOT/dependencies-py3.txt\""
run_check "SciPy dependency" "grep -q 'scipy' \"$PROJECT_ROOT/dependencies-py3.txt\""
run_check "OpenCV dependency" "grep -q 'opencv-python' \"$PROJECT_ROOT/dependencies-py3.txt\""
echo

# Key packages
echo "=== Key Packages ==="
run_check "duckietown_msgs package" "test -d \"$PROJECT_ROOT/packages/duckietown_msgs\""
run_check "line_detector package" "test -d \"$PROJECT_ROOT/packages/line_detector\""
run_check "lane_control package" "test -d \"$PROJECT_ROOT/packages/lane_control\""
run_check "vehicle_detection package" "test -d \"$PROJECT_ROOT/packages/vehicle_detection\""
run_check "navigation package" "test -d \"$PROJECT_ROOT/packages/navigation\""
run_check "fsm package" "test -d \"$PROJECT_ROOT/packages/fsm\""
echo

# Enhanced components
echo "=== Enhanced Components ==="
run_check "Enhanced line detector" "grep -q 'AdaptiveThresholdDetector' \"$PROJECT_ROOT/packages/line_detector/src/line_detector_node.py\""
run_check "Temporal filtering" "grep -q 'TemporalConsistencyFilter' \"$PROJECT_ROOT/packages/line_detector/src/line_detector_node.py\""
run_check_with_warning "Enhanced AprilTag detection" "test -d \"$PROJECT_ROOT/packages/enhanced_apriltag\""
run_check_with_warning "YOLO object detection" "find \"$PROJECT_ROOT/packages\" -name \"*yolo*\" -o -name \"*object*\" | grep -q ."
echo

# Network connectivity
echo "=== Network Connectivity ==="
run_check "Robot hostname resolution" "ping -c 1 ${ROBOT_NAME}.local"
run_check "SSH connectivity" "ssh -o ConnectTimeout=5 -o BatchMode=yes duckie@${ROBOT_NAME}.local 'echo test'"
echo

# Robot system checks
echo "=== Robot System Checks ==="
if ssh -o ConnectTimeout=5 duckie@${ROBOT_NAME}.local "true" &>/dev/null; then
    run_check "Robot ROS installation" "ssh duckie@${ROBOT_NAME}.local 'test -d /opt/ros/noetic'"
    run_check "Robot Docker installation" "ssh duckie@${ROBOT_NAME}.local 'command -v docker'"
    run_check "Robot workspace directory" "ssh duckie@${ROBOT_NAME}.local 'test -d /code/catkin_ws'"
    run_check_with_warning "Robot calibration directory" "ssh duckie@${ROBOT_NAME}.local 'test -d /data/config/calibrations'"
    
    # Get robot system info
    echo
    echo "=== Robot System Information ==="
    ssh duckie@${ROBOT_NAME}.local "
        echo 'Hostname: '\$(hostname)
        echo 'OS: '\$(cat /etc/os-release | grep PRETTY_NAME | cut -d'\"' -f2 2>/dev/null || echo 'Unknown')
        echo 'Architecture: '\$(uname -m)
        echo 'Kernel: '\$(uname -r)
        echo 'Uptime: '\$(uptime -p 2>/dev/null || uptime)
        echo 'Memory: '\$(free -h | grep Mem | awk '{print \$3\"/\"\$2}' 2>/dev/null || echo 'Unknown')
        echo 'Disk: '\$(df -h / | tail -1 | awk '{print \$3\"/\"\$2\" (\"\$5\" used)\"}' 2>/dev/null || echo 'Unknown')
        echo 'Temperature: '\$(vcgencmd measure_temp 2>/dev/null || echo 'N/A')
        echo 'Docker: '\$(docker --version 2>/dev/null || echo 'Not installed')
        echo 'ROS: '\$(rosversion -d 2>/dev/null || echo 'Not found')
    " 2>/dev/null | while IFS= read -r line; do
        log_info "  $line"
    done
else
    log_error "Cannot connect to robot - skipping robot system checks"
    ((error_count += 4))
    ((check_count += 4))
fi

echo
echo "=============================================="
echo "Pre-deployment Check Summary"
echo "=============================================="
echo "Total checks: $check_count"
echo -e "Passed: ${GREEN}$pass_count${NC}"
echo -e "Warnings: ${YELLOW}$warning_count${NC}"
echo -e "Errors: ${RED}$error_count${NC}"
echo

if [[ $error_count -eq 0 ]]; then
    log_success "All critical checks passed! System is ready for deployment."
    echo
    echo "To deploy the system, run:"
    echo "  ./scripts/deploy_pinkduckie.sh"
    exit 0
elif [[ $error_count -le 2 ]]; then
    log_warning "Some checks failed, but deployment may still be possible."
    echo "Please review the failed checks and consider fixing them before deployment."
    echo
    echo "To proceed with deployment anyway, run:"
    echo "  ./scripts/deploy_pinkduckie.sh"
    exit 1
else
    log_error "Multiple critical checks failed. Please fix these issues before deployment."
    echo
    echo "Common issues and solutions:"
    echo "1. Robot not accessible: Check network connection and SSH keys"
    echo "2. Missing dependencies: Run 'make build-validate' to check build system"
    echo "3. Docker issues: Ensure Docker Desktop is running"
    echo "4. Missing packages: Ensure you're in the correct dt-core directory"
    exit 2
fi