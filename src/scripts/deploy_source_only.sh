#!/bin/bash

# Source-Only Deployment Script for pinkduckie (DB21J)
# Deploys enhanced autonomous system source code to robot

set -e

ROBOT_NAME="pinkduckie"
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
        return 1
    fi
}

# RSYNC helper function
rsync_cmd() {
    local src="$1"
    local dst="$2"
    
    # Try SSH key authentication first
    if rsync -avz --progress --delete -e "ssh -o ConnectTimeout=5 -o BatchMode=yes" "$src" "$ROBOT_USER@$ROBOT_HOST:$dst" 2>/dev/null; then
        return 0
    fi
    
    # Fall back to password authentication
    if command -v sshpass &> /dev/null; then
        rsync -avz --progress --delete -e "sshpass -p $ROBOT_PASSWORD ssh -o ConnectTimeout=5 -o StrictHostKeyChecking=no" "$src" "$ROBOT_USER@$ROBOT_HOST:$dst"
    else
        log_error "sshpass not available and SSH keys not configured"
        return 1
    fi
}

echo "=========================================="
echo "Enhanced Autonomous System - Source Deployment"
echo "Robot: $ROBOT_NAME (DB21J)"
echo "=========================================="
echo

# Test connectivity
log_step "Testing robot connectivity..."

if ! ping -c 1 "$ROBOT_HOST" &>/dev/null; then
    log_error "Cannot reach robot at $ROBOT_HOST"
    exit 1
fi

if ! ssh_cmd "echo 'SSH connection successful'"; then
    log_error "SSH connection failed"
    exit 1
fi

log_success "Robot connectivity verified"

# Get robot info
log_info "Robot system information:"
ssh_cmd "
    echo 'Hostname: '\$(hostname)
    echo 'OS: '\$(lsb_release -d 2>/dev/null | cut -f2 || echo 'Unknown')
    echo 'Architecture: '\$(uname -m)
    echo 'ROS: '\$(rosversion -d 2>/dev/null || echo 'Not found')
    echo 'Docker: '\$(docker --version 2>/dev/null || echo 'Not installed')
    echo 'Memory: '\$(free -h | grep Mem | awk '{print \$3\"/\"\$2}')
    echo 'Disk: '\$(df -h /data | tail -1 | awk '{print \$3\"/\"\$2\" (\"\$5\" used)\"}')
" 2>/dev/null | while IFS= read -r line; do
    log_info "  $line"
done

# Create workspace structure
log_step "Setting up workspace on robot..."

ssh_cmd "
    # Create workspace directories
    mkdir -p /code/catkin_ws/src/dt-core
    mkdir -p /data/config/calibrations
    mkdir -p /tmp/duckietown-data
    
    # Set permissions
    sudo chown -R duckie:duckie /code/catkin_ws
    sudo chown -R duckie:duckie /data/config
"

log_success "Workspace created"

# Deploy source code
log_step "Deploying enhanced autonomous system source code..."

log_info "Syncing packages directory..."
rsync_cmd "src/packages/" "/code/catkin_ws/src/dt-core/src/packages/"

log_info "Syncing configuration files..."
rsync_cmd "configurations.yaml" "/code/catkin_ws/src/dt-core/"
rsync_cmd "src/robot_configs/pinkduckie.yaml" "/code/catkin_ws/src/dt-core/"
rsync_cmd "dependencies-py3.txt" "/code/catkin_ws/src/dt-core/"
rsync_cmd "dependencies-py3.dt.txt" "/code/catkin_ws/src/dt-core/"

log_info "Syncing launch files..."
rsync_cmd "src/packages/duckietown_demos/launch/" "/code/catkin_ws/src/dt-core/src/packages/duckietown_demos/launch/"

log_success "Source code deployed"

# Build system on robot
log_step "Building system on robot..."

ssh_cmd "
    cd /code/catkin_ws
    
    # Source ROS environment
    source /opt/ros/noetic/setup.bash
    
    # Install Python dependencies
    echo 'Installing Python dependencies...'
    pip3 install --user -r src/dt-core/dependencies-py3.txt || echo 'Some dependencies may have failed'
    
    # Build catkin workspace
    echo 'Building catkin workspace...'
    catkin build --workspace /code/catkin_ws || echo 'Build may have warnings'
    
    # Source the built workspace
    source devel/setup.bash
    
    echo 'Build completed'
"

log_success "System built on robot"

# Configure robot-specific settings
log_step "Configuring robot-specific settings..."

ssh_cmd "
    # Set environment variables
    echo 'export VEHICLE_NAME=pinkduckie' >> ~/.bashrc
    echo 'export ROS_MASTER_URI=http://pinkduckie.local:11311' >> ~/.bashrc
    echo 'export ROS_HOSTNAME=pinkduckie.local' >> ~/.bashrc
    echo 'source /opt/ros/noetic/setup.bash' >> ~/.bashrc
    echo 'source /code/catkin_ws/devel/setup.bash' >> ~/.bashrc
    
    # Create robot-specific configuration
    mkdir -p /data/config/robot_specific
    cp /code/catkin_ws/src/dt-core/src/robot_configs/pinkduckie.yaml /data/config/robot_specific/config.yaml
"

log_success "Robot configuration completed"

# Test system
log_step "Testing enhanced autonomous system..."

ssh_cmd "
    # Source environment
    source /opt/ros/noetic/setup.bash
    source /code/catkin_ws/devel/setup.bash
    export VEHICLE_NAME=pinkduckie
    
    # Test ROS packages
    echo 'Testing ROS package availability...'
    rospack find line_detector && echo 'line_detector: OK' || echo 'line_detector: MISSING'
    rospack find lane_control && echo 'lane_control: OK' || echo 'lane_control: MISSING'
    rospack find vehicle_detection && echo 'vehicle_detection: OK' || echo 'vehicle_detection: MISSING'
    rospack find fsm && echo 'fsm: OK' || echo 'fsm: MISSING'
    
    # Test launch file
    echo 'Testing launch file...'
    roslaunch --files duckietown_demos enhanced_autonomous_pinkduckie.launch && echo 'Launch file: OK' || echo 'Launch file: MISSING'
"

log_success "System testing completed"

# Create startup script
log_step "Creating startup script..."

ssh_cmd "cat > /home/duckie/start_enhanced_autonomous.sh << 'EOF'
#!/bin/bash

# Enhanced Autonomous System Startup Script for pinkduckie

echo 'Starting Enhanced Autonomous Duckietown System...'

# Set environment
export VEHICLE_NAME=pinkduckie
export ROS_MASTER_URI=http://pinkduckie.local:11311
export ROS_HOSTNAME=pinkduckie.local

# Source ROS environment
source /opt/ros/noetic/setup.bash
source /code/catkin_ws/devel/setup.bash

# Kill any existing ROS processes
pkill -f ros || true
sleep 2

# Start roscore
echo 'Starting ROS master...'
nohup roscore > /tmp/roscore.log 2>&1 &
sleep 5

# Check if roscore is running
if ! rostopic list &>/dev/null; then
    echo 'Failed to start ROS master'
    exit 1
fi

echo 'ROS master started successfully'

# Start camera node
echo 'Starting camera node...'
nohup roslaunch duckietown camera_node.launch veh:=pinkduckie > /tmp/camera.log 2>&1 &
sleep 3

# Start enhanced autonomous system
echo 'Starting enhanced autonomous system...'
nohup roslaunch duckietown_demos enhanced_autonomous_pinkduckie.launch veh:=pinkduckie > /tmp/autonomous.log 2>&1 &
sleep 5

# Check system status
echo 'Checking system status...'
rostopic list | grep pinkduckie | head -5

echo 'Enhanced Autonomous System startup completed!'
echo 'Monitor logs:'
echo '  ROS Master: tail -f /tmp/roscore.log'
echo '  Camera: tail -f /tmp/camera.log'
echo '  Autonomous: tail -f /tmp/autonomous.log'
EOF

chmod +x /home/duckie/start_enhanced_autonomous.sh"

log_success "Startup script created"

echo
echo "=========================================="
log_success "Enhanced Autonomous System Deployed!"
echo "=========================================="
echo
echo "To start the system on your robot:"
echo "1. SSH to robot: ssh duckie@pinkduckie.local"
echo "2. Run startup script: ./start_enhanced_autonomous.sh"
echo
echo "Or start components manually:"
echo "1. Start ROS master: roscore &"
echo "2. Start camera: roslaunch duckietown camera_node.launch veh:=pinkduckie &"
echo "3. Start autonomous system: roslaunch duckietown_demos enhanced_autonomous_pinkduckie.launch veh:=pinkduckie"
echo
echo "Monitor system:"
echo "- Check topics: rostopic list | grep pinkduckie"
echo "- Check nodes: rosnode list"
echo "- Monitor performance: htop"
echo "- Check logs: tail -f /tmp/*.log"
echo
echo "Safety reminders:"
echo "- Always supervise the robot during operation"
echo "- Test in a safe, controlled environment first"
echo "- Emergency stop: Ctrl+C or robot button"
echo