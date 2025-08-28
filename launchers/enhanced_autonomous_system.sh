#!/bin/bash

# Enhanced Autonomous System Launcher
# Comprehensive launcher for the enhanced autonomous Duckietown system
# with proper dependency management and startup sequencing

set -e

# Configuration
VEHICLE_NAME=${VEHICLE_NAME:-"blueduckie"}
CONFIG_MODE=${CONFIG_MODE:-"baseline"}
ENABLE_MONITORING=${ENABLE_MONITORING:-"true"}
ENABLE_LOGGING=${ENABLE_LOGGING:-"true"}
ENABLE_VALIDATION=${ENABLE_VALIDATION:-"true"}
VERBOSE=${VERBOSE:-"false"}

# Feature toggles
ENABLE_ENHANCED_LANE_FOLLOWING=${ENABLE_ENHANCED_LANE_FOLLOWING:-"true"}
ENABLE_APRILTAG_DETECTION=${ENABLE_APRILTAG_DETECTION:-"true"}
ENABLE_OBJECT_DETECTION=${ENABLE_OBJECT_DETECTION:-"true"}
ENABLE_LANE_CHANGING=${ENABLE_LANE_CHANGING:-"true"}
ENABLE_SAFETY_MONITORING=${ENABLE_SAFETY_MONITORING:-"true"}
ENABLE_COORDINATION=${ENABLE_COORDINATION:-"true"}
ENABLE_PERFORMANCE_OPTIMIZATION=${ENABLE_PERFORMANCE_OPTIMIZATION:-"true"}

# Directories
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT_DIR="$(dirname "$SCRIPT_DIR")"
LOG_DIR="/tmp/enhanced_autonomous_system_logs"

# Logging setup
mkdir -p "$LOG_DIR"
TIMESTAMP=$(date +"%Y%m%d_%H%M%S")
LAUNCH_LOG="$LOG_DIR/launch_${TIMESTAMP}.log"

# Logging functions
log_info() {
    echo "[$(date '+%Y-%m-%d %H:%M:%S')] [INFO] $1" | tee -a "$LAUNCH_LOG"
}

log_warn() {
    echo "[$(date '+%Y-%m-%d %H:%M:%S')] [WARN] $1" | tee -a "$LAUNCH_LOG"
}

log_error() {
    echo "[$(date '+%Y-%m-%d %H:%M:%S')] [ERROR] $1" | tee -a "$LAUNCH_LOG"
}

log_debug() {
    if [ "$VERBOSE" = "true" ]; then
        echo "[$(date '+%Y-%m-%d %H:%M:%S')] [DEBUG] $1" | tee -a "$LAUNCH_LOG"
    fi
}

# Cleanup function
cleanup() {
    log_info "Cleaning up Enhanced Autonomous System..."
    
    # Kill all ROS nodes
    if command -v rosnode &> /dev/null; then
        log_info "Stopping ROS nodes..."
        rosnode kill -a 2>/dev/null || true
    fi
    
    # Kill roscore if we started it
    if [ -n "$ROSCORE_PID" ]; then
        log_info "Stopping roscore..."
        kill $ROSCORE_PID 2>/dev/null || true
    fi
    
    log_info "Enhanced Autonomous System cleanup completed"
}

# Set up signal handlers
trap cleanup EXIT INT TERM

# Validation functions
validate_environment() {
    log_info "Validating environment..."
    
    # Check ROS installation
    if ! command -v roscore &> /dev/null; then
        log_error "ROS not found. Please install ROS Noetic."
        exit 1
    fi
    
    # Check Python dependencies
    if ! python3 -c "import rospy, cv2, numpy, yaml" 2>/dev/null; then
        log_error "Required Python dependencies not found."
        exit 1
    fi
    
    # Check vehicle name
    if [ -z "$VEHICLE_NAME" ]; then
        log_error "VEHICLE_NAME environment variable not set."
        exit 1
    fi
    
    # Check configuration file
    CONFIG_FILE="/code/catkin_ws/src/dt-duckiebot-interface/my-dt-core/configurations.yaml"
    if [ ! -f "$CONFIG_FILE" ]; then
        log_error "Configuration file not found: $CONFIG_FILE"
        exit 1
    fi
    
    log_info "✓ Environment validation passed"
}

validate_system_resources() {
    log_info "Validating system resources..."
    
    # Check available memory
    AVAILABLE_MEMORY=$(free -m | awk 'NR==2{printf "%.0f", $7}')
    if [ "$AVAILABLE_MEMORY" -lt 1024 ]; then
        log_warn "Low available memory: ${AVAILABLE_MEMORY}MB (recommended: >1GB)"
    fi
    
    # Check CPU load
    CPU_LOAD=$(uptime | awk -F'load average:' '{ print $2 }' | cut -d, -f1 | xargs)
    CPU_CORES=$(nproc)
    if (( $(echo "$CPU_LOAD > $CPU_CORES" | bc -l) )); then
        log_warn "High CPU load: $CPU_LOAD (cores: $CPU_CORES)"
    fi
    
    # Check disk space
    DISK_USAGE=$(df / | awk 'NR==2 {print $5}' | sed 's/%//')
    if [ "$DISK_USAGE" -gt 90 ]; then
        log_warn "High disk usage: ${DISK_USAGE}%"
    fi
    
    log_info "✓ System resources validated"
}

start_roscore() {
    log_info "Starting ROS core..."
    
    # Check if roscore is already running
    if rostopic list &>/dev/null; then
        log_info "ROS core already running"
        return 0
    fi
    
    # Start roscore in background
    roscore &
    ROSCORE_PID=$!
    
    # Wait for roscore to be ready
    local timeout=30
    local count=0
    while ! rostopic list &>/dev/null; do
        if [ $count -ge $timeout ]; then
            log_error "ROS core failed to start within $timeout seconds"
            exit 1
        fi
        sleep 1
        ((count++))
    done
    
    log_info "✓ ROS core started successfully"
}

load_system_configuration() {
    log_info "Loading system configuration..."
    
    # Load global configuration
    CONFIG_FILE="/code/catkin_ws/src/dt-duckiebot-interface/my-dt-core/configurations.yaml"
    if [ -f "$CONFIG_FILE" ]; then
        log_debug "Loading configuration from: $CONFIG_FILE"
        # Configuration is loaded by individual nodes
    fi
    
    # Set ROS parameters
    rosparam set /vehicle_name "$VEHICLE_NAME"
    rosparam set /config_mode "$CONFIG_MODE"
    rosparam set /enable_monitoring "$ENABLE_MONITORING"
    rosparam set /enable_logging "$ENABLE_LOGGING"
    
    # Feature parameters
    rosparam set /enhanced_lane_following_enabled "$ENABLE_ENHANCED_LANE_FOLLOWING"
    rosparam set /apriltag_detection_enabled "$ENABLE_APRILTAG_DETECTION"
    rosparam set /object_detection_enabled "$ENABLE_OBJECT_DETECTION"
    rosparam set /lane_changing_enabled "$ENABLE_LANE_CHANGING"
    rosparam set /safety_monitoring_enabled "$ENABLE_SAFETY_MONITORING"
    rosparam set /coordination_enabled "$ENABLE_COORDINATION"
    rosparam set /performance_optimization_enabled "$ENABLE_PERFORMANCE_OPTIMIZATION"
    
    log_info "✓ System configuration loaded"
}

start_startup_manager() {
    log_info "Starting Enhanced System Startup Manager..."
    
    # Start the startup manager
    STARTUP_SCRIPT="$ROOT_DIR/packages/duckietown_demos/scripts/enhanced_system_startup.py"
    
    if [ -f "$STARTUP_SCRIPT" ]; then
        log_debug "Starting startup manager: $STARTUP_SCRIPT"
        python3 "$STARTUP_SCRIPT" &
        STARTUP_MANAGER_PID=$!
        
        # Wait for startup manager to complete
        local timeout=300  # 5 minutes
        local count=0
        
        while kill -0 $STARTUP_MANAGER_PID 2>/dev/null; do
            if [ $count -ge $timeout ]; then
                log_error "Startup manager timeout after $timeout seconds"
                kill $STARTUP_MANAGER_PID 2>/dev/null || true
                exit 1
            fi
            sleep 1
            ((count++))
        done
        
        # Check startup manager exit status
        wait $STARTUP_MANAGER_PID
        local exit_status=$?
        
        if [ $exit_status -eq 0 ]; then
            log_info "✓ Enhanced System Startup Manager completed successfully"
        else
            log_error "Enhanced System Startup Manager failed with exit status: $exit_status"
            exit 1
        fi
    else
        log_error "Startup manager script not found: $STARTUP_SCRIPT"
        exit 1
    fi
}

launch_enhanced_system() {
    log_info "Launching Enhanced Autonomous System..."
    
    # Launch the complete enhanced system
    LAUNCH_FILE="$ROOT_DIR/packages/duckietown_demos/launch/enhanced_autonomous_system.launch"
    
    if [ -f "$LAUNCH_FILE" ]; then
        log_debug "Launching system with: $LAUNCH_FILE"
        
        # Build roslaunch command
        ROSLAUNCH_CMD="roslaunch $LAUNCH_FILE"
        ROSLAUNCH_CMD="$ROSLAUNCH_CMD veh:=$VEHICLE_NAME"
        ROSLAUNCH_CMD="$ROSLAUNCH_CMD config:=$CONFIG_MODE"
        ROSLAUNCH_CMD="$ROSLAUNCH_CMD enable_enhanced_lane_following:=$ENABLE_ENHANCED_LANE_FOLLOWING"
        ROSLAUNCH_CMD="$ROSLAUNCH_CMD enable_apriltag_detection:=$ENABLE_APRILTAG_DETECTION"
        ROSLAUNCH_CMD="$ROSLAUNCH_CMD enable_object_detection:=$ENABLE_OBJECT_DETECTION"
        ROSLAUNCH_CMD="$ROSLAUNCH_CMD enable_lane_changing:=$ENABLE_LANE_CHANGING"
        ROSLAUNCH_CMD="$ROSLAUNCH_CMD enable_safety_monitoring:=$ENABLE_SAFETY_MONITORING"
        ROSLAUNCH_CMD="$ROSLAUNCH_CMD enable_coordination:=$ENABLE_COORDINATION"
        ROSLAUNCH_CMD="$ROSLAUNCH_CMD enable_performance_optimization:=$ENABLE_PERFORMANCE_OPTIMIZATION"
        ROSLAUNCH_CMD="$ROSLAUNCH_CMD enable_monitoring:=$ENABLE_MONITORING"
        ROSLAUNCH_CMD="$ROSLAUNCH_CMD enable_logging:=$ENABLE_LOGGING"
        ROSLAUNCH_CMD="$ROSLAUNCH_CMD verbose:=$VERBOSE"
        
        log_debug "Executing: $ROSLAUNCH_CMD"
        
        # Launch system
        $ROSLAUNCH_CMD &
        ROSLAUNCH_PID=$!
        
        # Wait for system to be ready
        wait_for_system_ready
        
        log_info "✓ Enhanced Autonomous System launched successfully"
    else
        log_error "Launch file not found: $LAUNCH_FILE"
        exit 1
    fi
}

wait_for_system_ready() {
    log_info "Waiting for system to be ready..."
    
    local timeout=180  # 3 minutes
    local count=0
    
    # Check for key system topics
    local key_topics=(
        "/$VEHICLE_NAME/camera_node/image/compressed"
        "/$VEHICLE_NAME/lane_filter_node/lane_pose"
        "/$VEHICLE_NAME/safety_status_publisher/safety_status"
        "/$VEHICLE_NAME/master_integration_coordinator/system_status"
    )
    
    while [ $count -lt $timeout ]; do
        local all_ready=true
        
        for topic in "${key_topics[@]}"; do
            if ! rostopic list | grep -q "$topic"; then
                all_ready=false
                break
            fi
        done
        
        if [ "$all_ready" = true ]; then
            log_info "✓ All key system topics are available"
            break
        fi
        
        sleep 1
        ((count++))
        
        # Log progress every 30 seconds
        if [ $((count % 30)) -eq 0 ]; then
            log_info "Still waiting for system to be ready... ($count/$timeout seconds)"
        fi
    done
    
    if [ $count -ge $timeout ]; then
        log_error "System not ready within $timeout seconds"
        exit 1
    fi
}

validate_system_startup() {
    if [ "$ENABLE_VALIDATION" = "true" ]; then
        log_info "Validating system startup..."
        
        VALIDATION_SCRIPT="$ROOT_DIR/packages/duckietown_demos/scripts/validate_enhanced_system.py"
        
        if [ -f "$VALIDATION_SCRIPT" ]; then
            log_debug "Running validation script: $VALIDATION_SCRIPT"
            
            python3 "$VALIDATION_SCRIPT" &
            VALIDATION_PID=$!
            
            # Wait for validation to complete
            local timeout=300  # 5 minutes
            local count=0
            
            while kill -0 $VALIDATION_PID 2>/dev/null; do
                if [ $count -ge $timeout ]; then
                    log_error "System validation timeout after $timeout seconds"
                    kill $VALIDATION_PID 2>/dev/null || true
                    exit 1
                fi
                sleep 1
                ((count++))
            done
            
            # Check validation exit status
            wait $VALIDATION_PID
            local exit_status=$?
            
            if [ $exit_status -eq 0 ]; then
                log_info "✓ System validation passed"
            else
                log_error "System validation failed with exit status: $exit_status"
                exit 1
            fi
        else
            log_warn "Validation script not found: $VALIDATION_SCRIPT"
        fi
    else
        log_info "System validation skipped (ENABLE_VALIDATION=false)"
    fi
}

monitor_system() {
    log_info "Starting system monitoring..."
    
    # Monitor system health
    while true; do
        # Check if roslaunch is still running
        if ! kill -0 $ROSLAUNCH_PID 2>/dev/null; then
            log_error "ROS launch process died unexpectedly"
            exit 1
        fi
        
        # Check system resources
        if command -v free &> /dev/null; then
            MEMORY_USAGE=$(free | awk 'NR==2{printf "%.0f", $3/$2*100}')
            if [ "$MEMORY_USAGE" -gt 90 ]; then
                log_warn "High memory usage: ${MEMORY_USAGE}%"
            fi
        fi
        
        # Check for ROS errors
        if rostopic list &>/dev/null; then
            log_debug "ROS system healthy"
        else
            log_error "ROS system not responding"
            exit 1
        fi
        
        sleep 30  # Check every 30 seconds
    done
}

print_system_status() {
    log_info "Enhanced Autonomous System Status:"
    log_info "=================================="
    log_info "Vehicle Name: $VEHICLE_NAME"
    log_info "Configuration Mode: $CONFIG_MODE"
    log_info "Enhanced Lane Following: $ENABLE_ENHANCED_LANE_FOLLOWING"
    log_info "AprilTag Detection: $ENABLE_APRILTAG_DETECTION"
    log_info "Object Detection: $ENABLE_OBJECT_DETECTION"
    log_info "Lane Changing: $ENABLE_LANE_CHANGING"
    log_info "Safety Monitoring: $ENABLE_SAFETY_MONITORING"
    log_info "Coordination: $ENABLE_COORDINATION"
    log_info "Performance Optimization: $ENABLE_PERFORMANCE_OPTIMIZATION"
    log_info "Monitoring: $ENABLE_MONITORING"
    log_info "Logging: $ENABLE_LOGGING"
    log_info "Validation: $ENABLE_VALIDATION"
    log_info "Log Directory: $LOG_DIR"
    log_info "Launch Log: $LAUNCH_LOG"
    log_info "=================================="
}

show_usage() {
    echo "Enhanced Autonomous System Launcher"
    echo "Usage: $0 [OPTIONS]"
    echo ""
    echo "Environment Variables:"
    echo "  VEHICLE_NAME                     Vehicle name (default: duckiebot)"
    echo "  CONFIG_MODE                      Configuration mode (default: baseline)"
    echo "  ENABLE_ENHANCED_LANE_FOLLOWING   Enable enhanced lane following (default: true)"
    echo "  ENABLE_APRILTAG_DETECTION        Enable AprilTag detection (default: true)"
    echo "  ENABLE_OBJECT_DETECTION          Enable object detection (default: true)"
    echo "  ENABLE_LANE_CHANGING             Enable lane changing (default: true)"
    echo "  ENABLE_SAFETY_MONITORING         Enable safety monitoring (default: true)"
    echo "  ENABLE_COORDINATION              Enable coordination (default: true)"
    echo "  ENABLE_PERFORMANCE_OPTIMIZATION  Enable performance optimization (default: true)"
    echo "  ENABLE_MONITORING                Enable system monitoring (default: true)"
    echo "  ENABLE_LOGGING                   Enable comprehensive logging (default: true)"
    echo "  ENABLE_VALIDATION                Enable system validation (default: true)"
    echo "  VERBOSE                          Enable verbose logging (default: false)"
    echo ""
    echo "Options:"
    echo "  -h, --help                       Show this help message"
    echo "  -v, --verbose                    Enable verbose logging"
    echo "  --no-validation                  Disable system validation"
    echo "  --no-monitoring                  Disable system monitoring"
    echo "  --config MODE                    Set configuration mode"
    echo ""
    echo "Examples:"
    echo "  $0                               Launch with default settings"
    echo "  $0 --verbose                     Launch with verbose logging"
    echo "  $0 --config debug                Launch in debug mode"
    echo "  VEHICLE_NAME=duckiebot01 $0      Launch for specific vehicle"
}

# Parse command line arguments
while [[ $# -gt 0 ]]; do
    case $1 in
        -h|--help)
            show_usage
            exit 0
            ;;
        -v|--verbose)
            VERBOSE="true"
            shift
            ;;
        --no-validation)
            ENABLE_VALIDATION="false"
            shift
            ;;
        --no-monitoring)
            ENABLE_MONITORING="false"
            shift
            ;;
        --config)
            CONFIG_MODE="$2"
            shift 2
            ;;
        *)
            log_error "Unknown option: $1"
            show_usage
            exit 1
            ;;
    esac
done

# Main execution
main() {
    log_info "Starting Enhanced Autonomous Duckietown System..."
    log_info "Timestamp: $(date)"
    
    # Print system configuration
    print_system_status
    
    # Validation and setup
    validate_environment
    validate_system_resources
    
    # Start ROS infrastructure
    start_roscore
    load_system_configuration
    
    # Start enhanced system
    start_startup_manager
    launch_enhanced_system
    
    # Validate system
    validate_system_startup
    
    log_info "✓ Enhanced Autonomous System startup completed successfully"
    log_info "System is ready for autonomous operation"
    log_info "Press Ctrl+C to shutdown the system"
    
    # Monitor system
    monitor_system
}

# Run main function
main "$@"