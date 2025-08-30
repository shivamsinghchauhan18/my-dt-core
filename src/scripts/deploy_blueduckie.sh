#!/bin/bash

# Deployment Script for blueduckie (DB21J) - Advanced Autonomous Duckietown System
# Optimized for macOS M2 development environment to DB21J robot deployment

set -e

# Configuration
ROBOT_NAME="blueduckie"
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(dirname "$SCRIPT_DIR")"

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
CYAN='\033[0;36m'
NC='\033[0m' # No Color

# Logging functions
log_info() {
    echo -e "${BLUE}[INFO]${NC} $1"
}

log_success() {
    echo -e "${GREEN}[SUCCESS]${NC} $1"
}

log_warning() {
    echo -e "${YELLOW}[WARNING]${NC} $1"
}

log_error() {
    echo -e "${RED}[ERROR]${NC} $1"
}

log_step() {
    echo -e "${CYAN}[STEP]${NC} $1"
}

# Pre-deployment checks
pre_deployment_checks() {
    log_step "Performing pre-deployment checks..."
    
    # Check if we're on macOS
    if [[ "$OSTYPE" != "darwin"* ]]; then
        log_warning "This script is optimized for macOS M2. Some features may not work as expected."
    fi
    
    # Check Docker availability
    if ! command -v docker &> /dev/null; then
        log_error "Docker is required but not installed. Please install Docker Desktop for Mac."
        exit 1
    fi
    
    # Check if Docker is running
    if ! docker info &> /dev/null; then
        log_error "Docker is not running. Please start Docker Desktop."
        exit 1
    fi
    
    # Check SSH connectivity to robot
    log_info "Testing SSH connection to $ROBOT_NAME..."
    if ! ssh -o ConnectTimeout=5 -o BatchMode=yes duckie@${ROBOT_NAME}.local "echo 'SSH connection successful'" &>/dev/null; then
        log_error "Cannot connect to $ROBOT_NAME via SSH. Please ensure:"
        log_error "  1. Robot is powered on and connected to network"
        log_error "  2. SSH keys are properly configured"
        log_error "  3. Robot hostname is correct (${ROBOT_NAME}.local)"
        exit 1
    fi
    
    log_success "Pre-deployment checks completed"
}

# Build optimized image for ARM64
build_arm64_image() {
    log_step "Building ARM64 image for DB21J deployment..."
    
    cd "$PROJECT_ROOT"
    
    # Check if buildx is available and setup
    if ! docker buildx inspect dt-core-builder &>/dev/null; then
        log_info "Setting up Docker buildx for multi-architecture builds..."
        docker buildx create --name dt-core-builder --driver docker-container --bootstrap
        docker buildx use dt-core-builder
    fi
    
    # Build ARM64 production image
    log_info "Building production image for ARM64 architecture..."
    docker buildx build \
        --platform linux/arm64 \
        --target production \
        --build-arg BUILD_TARGET=production \
        --build-arg ARCH=arm64 \
        --build-arg DISTRO=daffy \
        --load \
        -t duckietown/dt-core:daffy-arm64 \
        .
    
    if [[ $? -eq 0 ]]; then
        log_success "ARM64 image built successfully"
    else
        log_error "Failed to build ARM64 image"
        exit 1
    fi
}

# Validate system before deployment
validate_system() {
    log_step "Validating system before deployment..."
    
    # Run system validation
    if [[ -f "$PROJECT_ROOT/src/scripts/run_system_validation.sh" ]]; then
        log_info "Running comprehensive system validation..."
        if "$PROJECT_ROOT/src/scripts/run_system_validation.sh" --functional-only --timeout 600; then
            log_success "System validation passed"
        else
            log_warning "System validation had issues. Continuing with deployment..."
        fi
    else
        log_warning "System validation script not found. Skipping validation."
    fi
}

# Deploy to robot
deploy_to_robot() {
    log_step "Deploying to $ROBOT_NAME..."
    
    # Use the enhanced deployment script
    if [[ -f "$PROJECT_ROOT/src/scripts/enhanced_deploy.sh" ]]; then
        log_info "Using enhanced deployment script..."
        "$PROJECT_ROOT/src/scripts/enhanced_deploy.sh" "$ROBOT_NAME" all
    else
        log_error "Enhanced deployment script not found"
        exit 1
    fi
}

# Post-deployment verification
post_deployment_verification() {
    log_step "Performing post-deployment verification..."
    
    # Check if the system is running on the robot
    log_info "Checking system status on robot..."
    
    # Test ROS connectivity
    ssh duckie@${ROBOT_NAME}.local "
        source /opt/ros/noetic/setup.bash
        timeout 10 rostopic list | grep -E '(camera|wheels|led)' || echo 'ROS topics not available'
    " 2>/dev/null || log_warning "Could not verify ROS topics"
    
    # Check Docker containers
    ssh duckie@${ROBOT_NAME}.local "
        docker ps | grep dt-core || echo 'dt-core container not running'
    " 2>/dev/null || log_warning "Could not verify Docker containers"
    
    # Check system resources
    log_info "Checking system resources..."
    ssh duckie@${ROBOT_NAME}.local "
        echo 'CPU Load:' \$(uptime | awk -F'load average:' '{print \$2}')
        echo 'Memory Usage:' \$(free | grep Mem | awk '{printf \"%.1f%%\", \$3/\$2 * 100.0}')
        echo 'Disk Usage:' \$(df / | tail -1 | awk '{print \$5}')
        echo 'Temperature:' \$(vcgencmd measure_temp 2>/dev/null || echo 'N/A')
    " 2>/dev/null || log_warning "Could not retrieve system metrics"
    
    log_success "Post-deployment verification completed"
}

# Start autonomous system
start_autonomous_system() {
    log_step "Starting advanced autonomous system on $ROBOT_NAME..."
    
    # Create startup script on robot
    ssh duckie@${ROBOT_NAME}.local "cat > /tmp/start_autonomous.sh << 'EOF'
#!/bin/bash

# Advanced Autonomous Duckietown System Startup Script
echo 'Starting Advanced Autonomous Duckietown System...'

# Source ROS environment
source /opt/ros/noetic/setup.bash
source /code/catkin_ws/devel/setup.bash

# Set robot-specific environment variables
export VEHICLE_NAME=blueduckie
export ROS_MASTER_URI=http://blueduckie.local:11311
export ROS_HOSTNAME=blueduckie.local

# Start the enhanced autonomous system
roslaunch duckietown_demos enhanced_autonomous_system.launch veh:=blueduckie &

# Wait for system to initialize
sleep 10

# Check if system is running
if rostopic list | grep -q '/blueduckie/'; then
    echo 'Advanced Autonomous System started successfully!'
    rostopic echo /blueduckie/fsm_node/mode -n 1
else
    echo 'Failed to start Advanced Autonomous System'
    exit 1
fi
EOF

chmod +x /tmp/start_autonomous.sh
"
    
    # Execute startup script
    log_info "Executing startup script on robot..."
    if ssh duckie@${ROBOT_NAME}.local "/tmp/start_autonomous.sh"; then
        log_success "Advanced autonomous system started successfully!"
    else
        log_error "Failed to start autonomous system"
        exit 1
    fi
}

# Monitor system performance
monitor_system() {
    log_step "Monitoring system performance..."
    
    log_info "System monitoring for 30 seconds..."
    log_info "Press Ctrl+C to stop monitoring early"
    
    for i in {1..6}; do
        echo "--- Monitor cycle $i/6 ---"
        ssh duckie@${ROBOT_NAME}.local "
            echo 'Time:' \$(date)
            echo 'CPU:' \$(top -bn1 | grep 'Cpu(s)' | awk '{print \$2}' | cut -d'%' -f1)%
            echo 'Memory:' \$(free | grep Mem | awk '{printf \"%.1f%%\", \$3/\$2 * 100.0}')
            echo 'Temperature:' \$(vcgencmd measure_temp 2>/dev/null || echo 'N/A')
            echo 'Active ROS nodes:' \$(rosnode list 2>/dev/null | wc -l || echo '0')
            echo
        " 2>/dev/null || log_warning "Could not retrieve monitoring data"
        
        sleep 5
    done
    
    log_success "System monitoring completed"
}

# Main deployment workflow
main() {
    echo "=========================================="
    echo "Advanced Autonomous Duckietown Deployment"
    echo "Robot: $ROBOT_NAME (DB21J)"
    echo "Development: macOS M2"
    echo "=========================================="
    echo
    
    # Execute deployment steps
    pre_deployment_checks
    build_arm64_image
    validate_system
    deploy_to_robot
    post_deployment_verification
    start_autonomous_system
    monitor_system
    
    echo
    echo "=========================================="
    log_success "Deployment completed successfully!"
    echo "=========================================="
    echo
    echo "Next steps:"
    echo "1. Monitor the robot's performance using: ssh duckie@${ROBOT_NAME}.local"
    echo "2. Check ROS topics: rostopic list"
    echo "3. View system logs: docker logs \$(docker ps -q --filter ancestor=duckietown/dt-core:daffy-arm64)"
    echo "4. Access the robot's camera stream at: http://${ROBOT_NAME}.local:8080"
    echo "5. Use the Duckietown Shell for advanced operations: dts start_gui_tools ${ROBOT_NAME}"
    echo
    echo "Safety reminders:"
    echo "- Always supervise the robot during autonomous operation"
    echo "- Keep the emergency stop button accessible"
    echo "- Monitor system performance and temperature"
    echo "- Test in a safe, controlled environment first"
    echo
}

# Handle interrupts gracefully
trap 'echo; log_warning "Deployment interrupted by user"; exit 1' INT

# Execute main function
main "$@"