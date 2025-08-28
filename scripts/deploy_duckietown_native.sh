#!/bin/bash

# Native Duckietown Deployment Script for pinkduckie (DB21J)
# Uses existing Duckietown infrastructure and Docker containers

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

echo "=========================================="
echo "Native Duckietown Deployment"
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

# Get robot system info
log_info "Robot system information:"
ssh_cmd "
    echo 'Hostname: '\$(hostname)
    echo 'OS: '\$(cat /etc/os-release | grep PRETTY_NAME | cut -d'\"' -f2 2>/dev/null || echo 'Unknown')
    echo 'Architecture: '\$(uname -m)
    echo 'Docker: '\$(docker --version 2>/dev/null || echo 'Not installed')
    echo 'Duckietown Shell: '\$(which dts 2>/dev/null || echo 'Not found')
    echo 'Memory: '\$(free -h | grep Mem | awk '{print \$3\"/\"\$2}' 2>/dev/null || echo 'Unknown')
    echo 'Disk: '\$(df -h /data | tail -1 | awk '{print \$3\"/\"\$2\" (\"\$5\" used)\"}' 2>/dev/null || echo 'Unknown')
    echo 'Temperature: '\$(vcgencmd measure_temp 2>/dev/null || echo 'N/A')
" 2>/dev/null | while IFS= read -r line; do
    log_info "  $line"
done

# Check Duckietown infrastructure
log_step "Checking Duckietown infrastructure..."

# Check if dts is available
if ssh_cmd "which dts" &>/dev/null; then
    log_success "Duckietown Shell (dts) is available"
    
    # Get dts version
    DTS_VERSION=$(ssh_cmd "dts version" 2>/dev/null || echo "unknown")
    log_info "DTS Version: $DTS_VERSION"
else
    log_warning "Duckietown Shell (dts) not found - using Docker directly"
fi

# Check Docker containers
log_info "Checking existing Docker containers..."
ssh_cmd "docker ps -a | grep -E '(dt-|duckietown)' | head -5" 2>/dev/null | while IFS= read -r line; do
    log_info "  $line"
done

# Deploy enhanced source code
log_step "Deploying enhanced autonomous system..."

# Create directories
ssh_cmd "
    mkdir -p /data/config/dt-core-enhanced
    mkdir -p /tmp/dt-core-enhanced
"

# Copy our enhanced line detector
log_info "Deploying enhanced line detector..."
sshpass -p "$ROBOT_PASSWORD" scp -o StrictHostKeyChecking=no \
    packages/line_detector/src/line_detector_node.py \
    "$ROBOT_USER@$ROBOT_HOST:/tmp/dt-core-enhanced/"

# Copy configuration
log_info "Deploying configuration..."
sshpass -p "$ROBOT_PASSWORD" scp -o StrictHostKeyChecking=no \
    robot_configs/pinkduckie.yaml \
    "$ROBOT_USER@$ROBOT_HOST:/data/config/dt-core-enhanced/"

# Create enhanced launch script
log_info "Creating enhanced launch script..."
ssh_cmd "cat > /home/duckie/start_enhanced_system.sh << 'EOF'
#!/bin/bash

# Enhanced Autonomous System for Duckietown
echo 'Starting Enhanced Autonomous System for pinkduckie...'

# Set environment
export VEHICLE_NAME=pinkduckie
export ROBOT_TYPE=duckiebot

# Stop any existing containers
echo 'Stopping existing containers...'
docker stop \$(docker ps -q --filter name=dt-) 2>/dev/null || true

# Start basic Duckietown stack with enhanced components
echo 'Starting enhanced Duckietown stack...'

# Start camera
echo 'Starting camera...'
docker run -d --name dt-camera-enhanced \\
    --privileged \\
    --network host \\
    -v /data:/data \\
    -e VEHICLE_NAME=pinkduckie \\
    -e ROBOT_TYPE=duckiebot \\
    duckietown/dt-duckiebot-interface:daffy-arm64v8 \\
    roslaunch duckietown_demos camera.launch veh:=pinkduckie

sleep 5

# Start enhanced lane following (using our enhanced line detector)
echo 'Starting enhanced lane following...'
docker run -d --name dt-lane-enhanced \\
    --privileged \\
    --network host \\
    -v /data:/data \\
    -v /tmp/dt-core-enhanced:/tmp/enhanced \\
    -e VEHICLE_NAME=pinkduckie \\
    -e ROBOT_TYPE=duckiebot \\
    duckietown/dt-core:daffy-arm64v8 \\
    bash -c \"
        # Copy enhanced line detector
        cp /tmp/enhanced/line_detector_node.py /code/catkin_ws/src/dt-core/packages/line_detector/src/
        
        # Source environment and start
        source /opt/ros/noetic/setup.bash
        source /code/catkin_ws/devel/setup.bash
        
        # Start enhanced lane following
        roslaunch duckietown_demos lane_following.launch veh:=pinkduckie
    \"

sleep 5

# Check system status
echo 'System Status:'
docker ps | grep dt-
echo
echo 'ROS Topics:'
docker exec dt-lane-enhanced rostopic list | grep pinkduckie | head -5
echo
echo 'Enhanced Autonomous System started!'
echo 'Monitor with: docker logs -f dt-lane-enhanced'
EOF

chmod +x /home/duckie/start_enhanced_system.sh"

log_success "Enhanced system deployed"

# Test the system
log_step "Testing enhanced system startup..."

log_info "Starting enhanced system on robot..."
if ssh_cmd "/home/duckie/start_enhanced_system.sh"; then
    log_success "Enhanced system started successfully!"
else
    log_warning "System startup had issues - check logs on robot"
fi

# Monitor system for 30 seconds
log_step "Monitoring system (30 seconds)..."

for i in {1..6}; do
    echo "--- Status check $i/6 ---"
    ssh_cmd "
        echo 'Time:' \$(date)
        echo 'Docker containers:'
        docker ps | grep dt- | wc -l
        echo 'System load:' \$(uptime | awk -F'load average:' '{print \$2}' | awk '{print \$1}')
        echo 'Memory usage:' \$(free | grep Mem | awk '{printf \"%.1f%%\", \$3/\$2 * 100.0}' 2>/dev/null || echo 'N/A')
        echo 'Temperature:' \$(vcgencmd measure_temp 2>/dev/null || echo 'N/A')
        echo
    " 2>/dev/null || log_warning "Could not retrieve status"
    
    sleep 5
done

echo
echo "=========================================="
log_success "Enhanced Duckietown System Deployed!"
echo "=========================================="
echo
echo "System is running on your robot with enhanced features:"
echo "✅ Enhanced adaptive lane detection"
echo "✅ Temporal consistency filtering"
echo "✅ Improved lighting adaptation"
echo
echo "To interact with your robot:"
echo "1. SSH to robot: ssh duckie@pinkduckie.local"
echo "2. Check containers: docker ps"
echo "3. View logs: docker logs -f dt-lane-enhanced"
echo "4. Restart system: ./start_enhanced_system.sh"
echo
echo "Monitor robot performance:"
echo "- Camera feed: Check robot's web interface"
echo "- System logs: docker logs dt-camera-enhanced"
echo "- Lane detection: docker logs dt-lane-enhanced"
echo
echo "Safety reminders:"
echo "- Always supervise the robot during operation"
echo "- Test in a safe, controlled environment"
echo "- Emergency stop: Press robot button or stop containers"
echo