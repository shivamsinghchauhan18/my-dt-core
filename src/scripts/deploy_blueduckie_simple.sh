#!/bin/bash

# Simple Deployment Script for blueduckie (DB21J)
# Handles both SSH key and password authentication

set -e

ROBOT_NAME="blueduckie"
ROBOT_HOST="${ROBOT_NAME}.local"
ROBOT_USER="duckie"
ROBOT_PASSWORD="quackquack"

# Colors
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
CYAN='\033[0;36m'
NC='\033[0m'

log_info() { echo -e "${BLUE}[INFO]${NC} $1"; }
log_success() { echo -e "${GREEN}[SUCCESS]${NC} $1"; }
log_warning() { echo -e "${YELLOW}[WARNING]${NC} $1"; }
log_error() { echo -e "${RED}[ERROR]${NC} $1"; }
log_step() { echo -e "${CYAN}[STEP]${NC} $1"; }

# SSH helper function
ssh_cmd() {
    local cmd="$1"
    
    # Try SSH key authentication first
    if ssh -o ConnectTimeout=5 -o BatchMode=yes "$ROBOT_USER@$ROBOT_HOST" "$cmd" 2>/dev/null; then
        return 0
    fi
    
    # Fall back to password authentication
    if command -v sshpass &> /dev/null; then
        sshpass -p "$ROBOT_PASSWORD" ssh -o ConnectTimeout=5 -o StrictHostKeyChecking=no "$ROBOT_USER@$ROBOT_HOST" "$cmd"
    else
        log_error "sshpass not available and SSH keys not configured"
        log_info "Please install sshpass: brew install hudochenkov/sshpass/sshpass"
        log_info "Or set up SSH keys: ./src/scripts/setup_ssh_keys.sh"
        return 1
    fi
}

# SCP helper function
scp_cmd() {
    local src="$1"
    local dst="$2"
    
    # Try SSH key authentication first
    if scp -o ConnectTimeout=5 -o BatchMode=yes "$src" "$ROBOT_USER@$ROBOT_HOST:$dst" 2>/dev/null; then
        return 0
    fi
    
    # Fall back to password authentication
    if command -v sshpass &> /dev/null; then
        sshpass -p "$ROBOT_PASSWORD" scp -o ConnectTimeout=5 -o StrictHostKeyChecking=no "$src" "$ROBOT_USER@$ROBOT_HOST:$dst"
    else
        log_error "sshpass not available and SSH keys not configured"
        return 1
    fi
}

# RSYNC helper function
rsync_cmd() {
    local src="$1"
    local dst="$2"
    
    # Try SSH key authentication first
    if rsync -avz --progress -e "ssh -o ConnectTimeout=5 -o BatchMode=yes" "$src" "$ROBOT_USER@$ROBOT_HOST:$dst" 2>/dev/null; then
        return 0
    fi
    
    # Fall back to password authentication
    if command -v sshpass &> /dev/null; then
        rsync -avz --progress -e "sshpass -p $ROBOT_PASSWORD ssh -o ConnectTimeout=5 -o StrictHostKeyChecking=no" "$src" "$ROBOT_USER@$ROBOT_HOST:$dst"
    else
        log_error "sshpass not available and SSH keys not configured"
        return 1
    fi
}

echo "=========================================="
echo "Simple Deployment for blueduckie (DB21J)"
echo "=========================================="
echo

# Check prerequisites
log_step "Checking prerequisites..."

if ! command -v docker &> /dev/null; then
    log_error "Docker not found. Please install Docker Desktop."
    exit 1
fi

if ! docker info &> /dev/null; then
    log_error "Docker not running. Please start Docker Desktop."
    exit 1
fi

# Test connectivity
log_step "Testing robot connectivity..."

if ! ping -c 1 "$ROBOT_HOST" &>/dev/null; then
    log_error "Cannot reach robot at $ROBOT_HOST"
    log_error "Please ensure robot is powered on and connected to network"
    exit 1
fi

log_success "Robot is reachable"

# Test SSH connection
log_info "Testing SSH connection..."
if ssh_cmd "echo 'SSH connection successful'"; then
    log_success "SSH connection established"
else
    log_error "SSH connection failed"
    log_info "Troubleshooting steps:"
    log_info "1. Ensure robot is fully booted (wait 2-3 minutes after power on)"
    log_info "2. Try manual SSH: ssh duckie@blueduckie.local (password: quackquack)"
    log_info "3. Set up SSH keys: ./src/scripts/setup_ssh_keys.sh"
    exit 1
fi

# Build Docker image (using existing Duckietown approach)
log_step "Building Docker image..."

log_info "Building production image..."
docker build \
    --target production \
    --build-arg BUILD_TARGET=production \
    --build-arg ARCH=amd64 \
    --build-arg DISTRO=daffy \
    -t duckietown/dt-core:daffy-amd64 \
    . || {
    log_error "Docker build failed"
    exit 1
}

log_success "Docker image built successfully"

# Deploy to robot
log_step "Deploying to robot..."

# Create workspace on robot
log_info "Creating workspace on robot..."
ssh_cmd "mkdir -p /code/catkin_ws/src/dt-core"

# Sync packages
log_info "Syncing packages directory..."
rsync_cmd "src/packages/" "/code/catkin_ws/src/dt-core/src/packages/"

# Sync configuration files
log_info "Syncing configuration files..."
scp_cmd "dependencies-py3.txt" "/code/catkin_ws/src/dt-core/"
scp_cmd "dependencies-py3.dt.txt" "/code/catkin_ws/src/dt-core/"
scp_cmd "configurations.yaml" "/code/catkin_ws/src/dt-core/"
scp_cmd "src/robot_configs/blueduckie.yaml" "/code/catkin_ws/src/dt-core/"

# Transfer Docker image
log_info "Transferring Docker image to robot..."
docker save duckietown/dt-core:daffy-amd64 | ssh_cmd "docker load"

log_success "Deployment completed successfully!"

# Start system
log_step "Starting enhanced autonomous system..."

ssh_cmd "
    export VEHICLE_NAME=blueduckie
    export ROS_MASTER_URI=http://blueduckie.local:11311
    export ROS_HOSTNAME=blueduckie.local
    
    # Kill any existing ROS processes
    pkill -f ros || true
    
    # Source ROS environment
    source /opt/ros/noetic/setup.bash
    
    # Start roscore in background
    nohup roscore > /tmp/roscore.log 2>&1 &
    sleep 3
    
    # Check if roscore is running
    if rostopic list &>/dev/null; then
        echo 'ROS master started successfully'
    else
        echo 'Failed to start ROS master'
        exit 1
    fi
"

log_success "System started successfully!"

# Monitor system
log_step "Monitoring system (30 seconds)..."

for i in {1..6}; do
    echo "--- Status check $i/6 ---"
    ssh_cmd "
        echo 'Time:' \$(date)
        echo 'ROS Master:' \$(rostopic list &>/dev/null && echo 'Running' || echo 'Not running')
        echo 'CPU Load:' \$(uptime | awk -F'load average:' '{print \$2}' | awk '{print \$1}')
        echo 'Memory:' \$(free | grep Mem | awk '{printf \"%.1f%%\", \$3/\$2 * 100.0}')
        echo 'Temperature:' \$(vcgencmd measure_temp 2>/dev/null || echo 'N/A')
        echo
    " 2>/dev/null || log_warning "Could not retrieve status"
    
    sleep 5
done

echo
echo "=========================================="
log_success "Deployment completed successfully!"
echo "=========================================="
echo
echo "Next steps:"
echo "1. SSH to robot: ssh duckie@blueduckie.local"
echo "2. Check ROS topics: rostopic list"
echo "3. Start camera: roslaunch duckietown camera_node.launch veh:=blueduckie"
echo "4. Start autonomous system: roslaunch duckietown_demos enhanced_autonomous_blueduckie.launch veh:=blueduckie"
echo
echo "Safety reminders:"
echo "- Always supervise the robot"
echo "- Test in a safe, controlled environment"
echo "- Keep emergency stop accessible"
echo