#!/bin/bash

# Enhanced Deployment Script for Advanced Autonomous Duckietown System
# Provides SSH-based file transfer, robot-specific configuration, health checks, and deployment verification

set -e

# Configuration
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(dirname "$SCRIPT_DIR")"
LOG_DIR="$PROJECT_ROOT/deployment_logs"
DEPLOY_LOG="$LOG_DIR/deploy_$(date +%Y%m%d_%H%M%S).log"

# Deployment configuration
DEFAULT_USER="duckie"
DEFAULT_PORT="22"
DEPLOYMENT_TIMEOUT="300"
HEALTH_CHECK_TIMEOUT="60"
RETRY_COUNT="3"

# Robot configuration
ROBOT_CONFIG_DIR="$PROJECT_ROOT/robot_configs"
CONFIGURATIONS_FILE="$PROJECT_ROOT/configurations.yaml"

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
CYAN='\033[0;36m'
NC='\033[0m' # No Color

# Logging functions
log_info() {
    local timestamp=$(date '+%Y-%m-%d %H:%M:%S')
    echo -e "${BLUE}[$timestamp] [DEPLOY]${NC} $1" | tee -a "$DEPLOY_LOG"
}

log_warn() {
    local timestamp=$(date '+%Y-%m-%d %H:%M:%S')
    echo -e "${YELLOW}[$timestamp] [WARN]${NC} $1" | tee -a "$DEPLOY_LOG"
}

log_error() {
    local timestamp=$(date '+%Y-%m-%d %H:%M:%S')
    echo -e "${RED}[$timestamp] [ERROR]${NC} $1" | tee -a "$DEPLOY_LOG"
}

log_success() {
    local timestamp=$(date '+%Y-%m-%d %H:%M:%S')
    echo -e "${GREEN}[$timestamp] [SUCCESS]${NC} $1" | tee -a "$DEPLOY_LOG"
}

log_progress() {
    local timestamp=$(date '+%Y-%m-%d %H:%M:%S')
    echo -e "${CYAN}[$timestamp] [PROGRESS]${NC} $1" | tee -a "$DEPLOY_LOG"
}

# Setup logging
setup_logging() {
    mkdir -p "$LOG_DIR"
    log_info "Enhanced deployment script started"
    log_info "Project root: $PROJECT_ROOT"
    log_info "Log directory: $LOG_DIR"
    log_info "Deployment log: $DEPLOY_LOG"
}

# Validate prerequisites
check_prerequisites() {
    log_info "Checking deployment prerequisites..."
    
    # Check SSH client
    if ! command -v ssh &> /dev/null; then
        log_error "SSH client is not installed or not in PATH"
        exit 1
    fi
    
    # Check rsync
    if ! command -v rsync &> /dev/null; then
        log_error "rsync is not installed or not in PATH"
        exit 1
    fi
    
    # Check scp
    if ! command -v scp &> /dev/null; then
        log_error "scp is not installed or not in PATH"
        exit 1
    fi
    
    # Check Docker (for image operations)
    if ! command -v docker &> /dev/null; then
        log_warn "Docker is not available - image operations will be limited"
    fi
    
    log_success "Prerequisites check completed"
}

# Load robot configuration
load_robot_config() {
    local robot_name=$1
    
    log_info "Loading configuration for robot: $robot_name"
    
    # Check for robot-specific config file
    local robot_config_file="$ROBOT_CONFIG_DIR/${robot_name}.yaml"
    
    if [[ -f "$robot_config_file" ]]; then
        log_info "Found robot-specific configuration: $robot_config_file"
        
        # Extract configuration using Python (more reliable than shell parsing)
        python3 -c "
import yaml
import sys

try:
    with open('$robot_config_file', 'r') as f:
        config = yaml.safe_load(f)
    
    robot_config = config.get('robot', {})
    
    # Export configuration as environment variables
    print('export ROBOT_HOST=\"{}\"'.format(robot_config.get('host', '')))
    print('export ROBOT_USER=\"{}\"'.format(robot_config.get('user', '$DEFAULT_USER')))
    print('export ROBOT_PORT=\"{}\"'.format(robot_config.get('port', '$DEFAULT_PORT')))
    print('export ROBOT_ARCH=\"{}\"'.format(robot_config.get('architecture', 'arm64')))
    print('export ROBOT_DISTRO=\"{}\"'.format(robot_config.get('distro', 'daffy')))
    print('export ROBOT_WORKSPACE=\"{}\"'.format(robot_config.get('workspace', '/code/catkin_ws/src/dt-core')))
    
    # Calibration settings
    calibration = robot_config.get('calibration', {})
    print('export ROBOT_CALIBRATION_DIR=\"{}\"'.format(calibration.get('directory', '/data/config/calibrations')))
    
    # Network settings
    network = robot_config.get('network', {})
    print('export ROBOT_WIFI_SSID=\"{}\"'.format(network.get('wifi_ssid', '')))
    
    # Hardware settings
    hardware = robot_config.get('hardware', {})
    print('export ROBOT_CAMERA_TOPIC=\"{}\"'.format(hardware.get('camera_topic', '/camera/image_raw')))
    
except Exception as e:
    print('echo \"Error loading robot configuration: {}\"'.format(e), file=sys.stderr)
    sys.exit(1)
" 2>/dev/null || {
            log_error "Failed to parse robot configuration file: $robot_config_file"
            return 1
        }
    else
        log_warn "No robot-specific configuration found for: $robot_name"
        log_info "Using default configuration values"
        
        # Set default values
        export ROBOT_HOST=""
        export ROBOT_USER="$DEFAULT_USER"
        export ROBOT_PORT="$DEFAULT_PORT"
        export ROBOT_ARCH="arm64"
        export ROBOT_DISTRO="daffy"
        export ROBOT_WORKSPACE="/code/catkin_ws/src/dt-core"
        export ROBOT_CALIBRATION_DIR="/data/config/calibrations"
        export ROBOT_WIFI_SSID=""
        export ROBOT_CAMERA_TOPIC="/camera/image_raw"
    fi
    
    # Validate required configuration
    if [[ -z "$ROBOT_HOST" ]]; then
        log_error "Robot host not specified in configuration"
        return 1
    fi
    
    log_success "Robot configuration loaded successfully"
    log_info "  Host: $ROBOT_HOST"
    log_info "  User: $ROBOT_USER"
    log_info "  Port: $ROBOT_PORT"
    log_info "  Architecture: $ROBOT_ARCH"
    log_info "  Distribution: $ROBOT_DISTRO"
    log_info "  Workspace: $ROBOT_WORKSPACE"
}

# Test SSH connectivity
test_ssh_connection() {
    local host=$1
    local user=$2
    local port=$3
    
    log_info "Testing SSH connection to $user@$host:$port..."
    
    # Test SSH connection with timeout
    if timeout 10 ssh -o ConnectTimeout=5 -o BatchMode=yes -p "$port" "$user@$host" "echo 'SSH connection successful'" &>/dev/null; then
        log_success "SSH connection to $user@$host:$port successful"
        return 0
    else
        log_error "SSH connection to $user@$host:$port failed"
        log_info "Please ensure:"
        log_info "  1. Robot is powered on and connected to network"
        log_info "  2. SSH keys are properly configured"
        log_info "  3. Robot hostname/IP is correct"
        log_info "  4. SSH service is running on robot"
        return 1
    fi
}

# Get system information from robot
get_robot_system_info() {
    local host=$1
    local user=$2
    local port=$3
    
    log_info "Gathering system information from robot..."
    
    # Execute remote commands to get system info
    local system_info
    system_info=$(ssh -p "$port" "$user@$host" "
        echo '=== System Information ==='
        echo 'Hostname: \$(hostname)'
        echo 'OS: \$(cat /etc/os-release | grep PRETTY_NAME | cut -d'\"' -f2)'
        echo 'Architecture: \$(uname -m)'
        echo 'Kernel: \$(uname -r)'
        echo 'Uptime: \$(uptime -p)'
        echo 'Memory: \$(free -h | grep Mem | awk '{print \$3\"/\"\$2}')'
        echo 'Disk: \$(df -h / | tail -1 | awk '{print \$3\"/\"\$2\" (\"\$5\" used)\"}')'
        echo 'Docker: \$(docker --version 2>/dev/null || echo 'Not installed')'
        echo 'ROS: \$(rosversion -d 2>/dev/null || echo 'Not found')'
        echo '=== Network Information ==='
        echo 'IP Address: \$(hostname -I | awk '{print \$1}')'
        echo 'WiFi Status: \$(iwconfig 2>/dev/null | grep ESSID | head -1 || echo 'Not connected')'
    " 2>/dev/null)
    
    if [[ $? -eq 0 ]]; then
        log_success "System information retrieved successfully"
        echo "$system_info" | while IFS= read -r line; do
            log_info "  $line"
        done
    else
        log_warn "Failed to retrieve complete system information"
    fi
}

# Sync source code to robot
sync_source_code() {
    local host=$1
    local user=$2
    local port=$3
    local workspace=$4
    
    log_progress "Syncing source code to robot..."
    
    # Create workspace directory on robot
    ssh -p "$port" "$user@$host" "mkdir -p $workspace" || {
        log_error "Failed to create workspace directory on robot"
        return 1
    }
    
    # Sync packages directory
    log_info "Syncing packages directory..."
    rsync -avz --progress --delete \
        -e "ssh -p $port" \
        "$PROJECT_ROOT/src/packages/" \
        "$user@$host:$workspace/src/packages/" | while IFS= read -r line; do
        log_progress "  $line"
    done
    
    if [[ ${PIPESTATUS[0]} -ne 0 ]]; then
        log_error "Failed to sync packages directory"
        return 1
    fi
    
    # Sync configuration files
    log_info "Syncing configuration files..."
    local config_files=(
        "dependencies-py3.txt"
        "dependencies-py3.dt.txt"
        "dependencies-py3.mock.txt"
        "dependencies-apt.txt"
        "configurations.yaml"
    )
    
    for config_file in "${config_files[@]}"; do
        if [[ -f "$PROJECT_ROOT/$config_file" ]]; then
            scp -P "$port" "$PROJECT_ROOT/$config_file" "$user@$host:$workspace/" || {
                log_warn "Failed to sync $config_file"
            }
        fi
    done
    
    # Sync scripts
    log_info "Syncing deployment scripts..."
    rsync -avz --progress \
        -e "ssh -p $port" \
        "$PROJECT_ROOT/src/scripts/" \
        "$user@$host:$workspace/src/scripts/" | while IFS= read -r line; do
        log_progress "  $line"
    done
    
    # Make scripts executable
    ssh -p "$port" "$user@$host" "chmod +x $workspace/src/scripts/*.sh" || {
        log_warn "Failed to make scripts executable"
    }
    
    log_success "Source code sync completed"
}

# Deploy Docker image to robot
deploy_docker_image() {
    local host=$1
    local user=$2
    local port=$3
    local arch=$4
    local distro=$5
    
    log_progress "Deploying Docker image to robot..."
    
    local image_tag="duckietown/dt-core:${distro}-${arch}"
    
    # Check if image exists locally
    if docker images | grep -q "duckietown/dt-core"; then
        log_info "Found local Docker image, transferring to robot..."
        
        # Save image to tar file
        local image_file="/tmp/dt-core-${distro}-${arch}.tar"
        docker save "$image_tag" > "$image_file" || {
            log_error "Failed to save Docker image"
            return 1
        }
        
        # Transfer image to robot
        log_info "Transferring image file to robot (this may take several minutes)..."
        scp -P "$port" "$image_file" "$user@$host:/tmp/" || {
            log_error "Failed to transfer image file"
            rm -f "$image_file"
            return 1
        }
        
        # Load image on robot
        log_info "Loading Docker image on robot..."
        ssh -p "$port" "$user@$host" "docker load < /tmp/$(basename $image_file)" || {
            log_error "Failed to load Docker image on robot"
            return 1
        }
        
        # Cleanup
        rm -f "$image_file"
        ssh -p "$port" "$user@$host" "rm -f /tmp/$(basename $image_file)"
        
        log_success "Docker image deployed successfully"
    else
        log_warn "No local Docker image found, robot will need to pull from registry"
        
        # Pull image on robot
        log_info "Pulling Docker image on robot..."
        ssh -p "$port" "$user@$host" "docker pull $image_tag" || {
            log_error "Failed to pull Docker image on robot"
            return 1
        }
        
        log_success "Docker image pulled successfully on robot"
    fi
}

# Deploy robot-specific configuration
deploy_robot_configuration() {
    local host=$1
    local user=$2
    local port=$3
    local robot_name=$4
    
    log_progress "Deploying robot-specific configuration..."
    
    # Create configuration directories on robot
    ssh -p "$port" "$user@$host" "
        mkdir -p /data/config/calibrations
        mkdir -p /data/config/robot_specific
        mkdir -p ~/.dt-shell
    " || {
        log_warn "Failed to create some configuration directories"
    }
    
    # Deploy robot-specific configuration file
    local robot_config_file="$ROBOT_CONFIG_DIR/${robot_name}.yaml"
    if [[ -f "$robot_config_file" ]]; then
        scp -P "$port" "$robot_config_file" "$user@$host:/data/config/robot_specific/config.yaml" || {
            log_warn "Failed to deploy robot-specific configuration"
        }
    fi
    
    # Deploy calibration files if they exist
    local calibration_dir="$PROJECT_ROOT/calibrations/$robot_name"
    if [[ -d "$calibration_dir" ]]; then
        log_info "Deploying calibration files..."
        rsync -avz --progress \
            -e "ssh -p $port" \
            "$calibration_dir/" \
            "$user@$host:$ROBOT_CALIBRATION_DIR/" | while IFS= read -r line; do
            log_progress "  $line"
        done
    else
        log_warn "No calibration files found for robot: $robot_name"
    fi
    
    log_success "Robot configuration deployment completed"
}

# Perform health checks
perform_health_checks() {
    local host=$1
    local user=$2
    local port=$3
    local workspace=$4
    
    log_progress "Performing deployment health checks..."
    
    # Check workspace structure
    log_info "Checking workspace structure..."
    local workspace_check
    workspace_check=$(ssh -p "$port" "$user@$host" "
        echo 'Workspace directory: \$(ls -la $workspace 2>/dev/null | wc -l) items'
        echo 'Packages directory: \$(ls -la $workspace/packages 2>/dev/null | wc -l) items'
        echo 'Scripts directory: \$(ls -la $workspace/scripts 2>/dev/null | wc -l) items'
        echo 'Configuration files:'
        ls -la $workspace/*.txt $workspace/*.yaml 2>/dev/null || echo 'No configuration files found'
    " 2>/dev/null)
    
    echo "$workspace_check" | while IFS= read -r line; do
        log_info "  $line"
    done
    
    # Check Docker image
    log_info "Checking Docker image availability..."
    local docker_check
    docker_check=$(ssh -p "$port" "$user@$host" "
        docker images | grep duckietown/dt-core || echo 'No dt-core images found'
    " 2>/dev/null)
    
    if [[ "$docker_check" == *"No dt-core images found"* ]]; then
        log_warn "Docker image not found on robot"
    else
        log_success "Docker image available on robot"
        echo "$docker_check" | while IFS= read -r line; do
            log_info "  $line"
        done
    fi
    
    # Check ROS environment
    log_info "Checking ROS environment..."
    local ros_check
    ros_check=$(ssh -p "$port" "$user@$host" "
        source /opt/ros/noetic/setup.bash 2>/dev/null && echo 'ROS Noetic available' || echo 'ROS not found'
        roscore --version 2>/dev/null || echo 'roscore not available'
    " 2>/dev/null)
    
    echo "$ros_check" | while IFS= read -r line; do
        log_info "  $line"
    done
    
    # Check system resources
    log_info "Checking system resources..."
    local resource_check
    resource_check=$(ssh -p "$port" "$user@$host" "
        echo 'CPU Load: \$(uptime | awk -F'load average:' '{print \$2}')'
        echo 'Memory Usage: \$(free | grep Mem | awk '{printf \"%.1f%%\", \$3/\$2 * 100.0}')'
        echo 'Disk Usage: \$(df / | tail -1 | awk '{print \$5}')'
        echo 'Docker Status: \$(systemctl is-active docker 2>/dev/null || echo 'unknown')'
    " 2>/dev/null)
    
    echo "$resource_check" | while IFS= read -r line; do
        log_info "  $line"
    done
    
    log_success "Health checks completed"
}

# Verify deployment
verify_deployment() {
    local host=$1
    local user=$2
    local port=$3
    local workspace=$4
    
    log_progress "Verifying deployment..."
    
    local verification_errors=0
    
    # Verify workspace exists
    if ! ssh -p "$port" "$user@$host" "test -d $workspace" 2>/dev/null; then
        log_error "Workspace directory not found: $workspace"
        ((verification_errors++))
    fi
    
    # Verify packages directory
    if ! ssh -p "$port" "$user@$host" "test -d $workspace/packages" 2>/dev/null; then
        log_error "Packages directory not found: $workspace/packages"
        ((verification_errors++))
    fi
    
    # Verify key packages exist
    local key_packages=(
        "duckietown_msgs"
        "lane_control"
        "lane_filter"
        "line_detector"
        "vehicle_detection"
        "navigation"
        "fsm"
    )
    
    for package in "${key_packages[@]}"; do
        if ! ssh -p "$port" "$user@$host" "test -d $workspace/src/packages/$package" 2>/dev/null; then
            log_warn "Package not found: $package"
        fi
    done
    
    # Verify configuration files
    local config_files=(
        "dependencies-py3.txt"
        "configurations.yaml"
    )
    
    for config_file in "${config_files[@]}"; do
        if ! ssh -p "$port" "$user@$host" "test -f $workspace/$config_file" 2>/dev/null; then
            log_warn "Configuration file not found: $config_file"
        fi
    done
    
    # Test basic ROS functionality
    log_info "Testing basic ROS functionality..."
    local ros_test
    ros_test=$(ssh -p "$port" "$user@$host" "
        timeout 10 bash -c '
            source /opt/ros/noetic/setup.bash
            roscore &
            sleep 3
            rostopic list
            pkill -f roscore
        ' 2>/dev/null || echo 'ROS test failed'
    ")
    
    if [[ "$ros_test" == *"ROS test failed"* ]]; then
        log_warn "Basic ROS functionality test failed"
    else
        log_success "Basic ROS functionality verified"
    fi
    
    if [[ $verification_errors -eq 0 ]]; then
        log_success "Deployment verification completed successfully"
        return 0
    else
        log_error "Deployment verification failed with $verification_errors errors"
        return 1
    fi
}

# Main deployment function
deploy_to_robot() {
    local robot_name=$1
    local deploy_options=$2
    
    log_info "Starting deployment to robot: $robot_name"
    log_info "Deployment options: $deploy_options"
    
    # Load robot configuration
    if ! load_robot_config "$robot_name"; then
        log_error "Failed to load robot configuration"
        return 1
    fi
    
    # Test SSH connection
    if ! test_ssh_connection "$ROBOT_HOST" "$ROBOT_USER" "$ROBOT_PORT"; then
        log_error "SSH connection test failed"
        return 1
    fi
    
    # Get system information
    get_robot_system_info "$ROBOT_HOST" "$ROBOT_USER" "$ROBOT_PORT"
    
    # Sync source code
    if [[ "$deploy_options" == *"source"* ]] || [[ "$deploy_options" == *"all"* ]]; then
        if ! sync_source_code "$ROBOT_HOST" "$ROBOT_USER" "$ROBOT_PORT" "$ROBOT_WORKSPACE"; then
            log_error "Source code sync failed"
            return 1
        fi
    fi
    
    # Deploy Docker image
    if [[ "$deploy_options" == *"image"* ]] || [[ "$deploy_options" == *"all"* ]]; then
        if ! deploy_docker_image "$ROBOT_HOST" "$ROBOT_USER" "$ROBOT_PORT" "$ROBOT_ARCH" "$ROBOT_DISTRO"; then
            log_error "Docker image deployment failed"
            return 1
        fi
    fi
    
    # Deploy configuration
    if [[ "$deploy_options" == *"config"* ]] || [[ "$deploy_options" == *"all"* ]]; then
        if ! deploy_robot_configuration "$ROBOT_HOST" "$ROBOT_USER" "$ROBOT_PORT" "$robot_name"; then
            log_error "Configuration deployment failed"
            return 1
        fi
    fi
    
    # Perform health checks
    perform_health_checks "$ROBOT_HOST" "$ROBOT_USER" "$ROBOT_PORT" "$ROBOT_WORKSPACE"
    
    # Verify deployment
    if ! verify_deployment "$ROBOT_HOST" "$ROBOT_USER" "$ROBOT_PORT" "$ROBOT_WORKSPACE"; then
        log_error "Deployment verification failed"
        return 1
    fi
    
    log_success "Deployment to robot '$robot_name' completed successfully"
    return 0
}

# Usage information
show_usage() {
    echo "Enhanced Deployment Script for Advanced Autonomous Duckietown System"
    echo
    echo "Usage: $0 [OPTIONS] ROBOT_NAME [DEPLOY_OPTIONS]"
    echo
    echo "Arguments:"
    echo "  ROBOT_NAME          Name of the robot to deploy to (must have config in src/robot_configs/)"
    echo
    echo "Deploy Options:"
    echo "  source              Deploy source code only"
    echo "  image               Deploy Docker image only"
    echo "  config              Deploy configuration only"
    echo "  all                 Deploy everything (default)"
    echo
    echo "Options:"
    echo "  -h, --help          Show this help message"
    echo "  -v, --verbose       Enable verbose logging"
    echo "  --dry-run           Show what would be deployed without actually deploying"
    echo "  --timeout SECONDS   Set deployment timeout (default: $DEPLOYMENT_TIMEOUT)"
    echo "  --retry COUNT       Set retry count for failed operations (default: $RETRY_COUNT)"
    echo
    echo "Examples:"
    echo "  $0 duckiebot01                    # Deploy everything to duckiebot01"
    echo "  $0 duckiebot01 source             # Deploy only source code"
    echo "  $0 --verbose duckiebot01 all      # Deploy everything with verbose output"
    echo "  $0 --dry-run duckiebot01          # Show deployment plan without executing"
    echo
    echo "Robot Configuration:"
    echo "  Robot configurations should be placed in: src/robot_configs/ROBOT_NAME.yaml"
    echo "  See src/robot_configs/example.yaml for configuration format"
}

# Create example robot configuration
create_example_config() {
    mkdir -p "$ROBOT_CONFIG_DIR"
    
    local example_config="$ROBOT_CONFIG_DIR/example.yaml"
    
    if [[ ! -f "$example_config" ]]; then
        cat > "$example_config" << 'EOF'
# Example Robot Configuration
# Copy this file to ROBOT_NAME.yaml and customize for your robot

robot:
  # Network configuration
  host: "duckiebot01.local"  # Robot hostname or IP address
  user: "duckie"             # SSH username
  port: 22                   # SSH port
  
  # System configuration
  architecture: "arm64"      # Robot architecture (arm64, amd64)
  distro: "daffy"           # Duckietown distribution
  workspace: "/code/catkin_ws/src/dt-core"  # Workspace path on robot
  
  # Calibration configuration
  calibration:
    directory: "/data/config/calibrations"
    camera_intrinsic: "camera_intrinsic.yaml"
    camera_extrinsic: "camera_extrinsic.yaml"
    kinematics: "kinematics.yaml"
  
  # Network configuration
  network:
    wifi_ssid: "Duckietown"
    wifi_password: ""  # Leave empty if using key-based auth
  
  # Hardware configuration
  hardware:
    camera_topic: "/camera/image_raw"
    motor_topic: "/wheels_driver_node/wheels_cmd"
    led_topic: "/led_emitter_node/led_pattern"
    
  # Advanced configuration
  advanced:
    ros_master_uri: "http://localhost:11311"
    ros_hostname: "localhost"
    log_level: "INFO"
    
  # Deployment preferences
  deployment:
    auto_start: true
    health_check_interval: 30
    restart_policy: "unless-stopped"
EOF
        
        log_info "Created example robot configuration: $example_config"
    fi
}

# Main execution
main() {
    local verbose=false
    local dry_run=false
    local robot_name=""
    local deploy_options="all"
    
    # Parse arguments
    while [[ $# -gt 0 ]]; do
        case $1 in
            -h|--help)
                show_usage
                exit 0
                ;;
            -v|--verbose)
                verbose=true
                shift
                ;;
            --dry-run)
                dry_run=true
                shift
                ;;
            --timeout)
                DEPLOYMENT_TIMEOUT="$2"
                shift 2
                ;;
            --retry)
                RETRY_COUNT="$2"
                shift 2
                ;;
            -*)
                log_error "Unknown option: $1"
                show_usage
                exit 1
                ;;
            *)
                if [[ -z "$robot_name" ]]; then
                    robot_name="$1"
                else
                    deploy_options="$1"
                fi
                shift
                ;;
        esac
    done
    
    # Validate arguments
    if [[ -z "$robot_name" ]]; then
        log_error "Robot name is required"
        show_usage
        exit 1
    fi
    
    # Setup
    setup_logging
    create_example_config
    check_prerequisites
    
    # Dry run mode
    if [[ $dry_run == true ]]; then
        log_info "DRY RUN MODE - No actual deployment will be performed"
        log_info "Would deploy to robot: $robot_name"
        log_info "Would deploy: $deploy_options"
        
        if load_robot_config "$robot_name"; then
            log_info "Would connect to: $ROBOT_USER@$ROBOT_HOST:$ROBOT_PORT"
            log_info "Would deploy to workspace: $ROBOT_WORKSPACE"
        fi
        
        exit 0
    fi
    
    # Execute deployment
    if deploy_to_robot "$robot_name" "$deploy_options"; then
        log_success "Enhanced deployment script completed successfully"
        exit 0
    else
        log_error "Enhanced deployment script failed"
        exit 1
    fi
}

# Execute main function
main "$@"