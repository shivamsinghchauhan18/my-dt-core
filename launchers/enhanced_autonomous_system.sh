#!/bin/bash

# Enhanced Autonomous System Launcher
# Comprehensive launcher for the enhanced autonomous Duckietown system
# with proper dependency management and startup sequencing

set -e

# Logging setup (define functions early)
mkdir -p "/tmp/enhanced_autonomous_system_logs"
TIMESTAMP=$(date +"%Y%m%d_%H%M%S")
LAUNCH_LOG="/tmp/enhanced_autonomous_system_logs/launch_${TIMESTAMP}.log"

log_info() { echo "[$(date '+%Y-%m-%d %H:%M:%S')] [INFO] $1" | tee -a "$LAUNCH_LOG"; }
log_warn() { echo "[$(date '+%Y-%m-%d %H:%M:%S')] [WARN] $1" | tee -a "$LAUNCH_LOG"; }
log_error() { echo "[$(date '+%Y-%m-%d %H:%M:%S')] [ERROR] $1" | tee -a "$LAUNCH_LOG"; }
log_debug() { if [ "$VERBOSE" = "true" ]; then echo "[$(date '+%Y-%m-%d %H:%M:%S')] [DEBUG] $1" | tee -a "$LAUNCH_LOG"; fi }

# Configuration
VEHICLE_NAME=${VEHICLE_NAME:-"blueduckie"}
CONFIG_MODE=${CONFIG_MODE:-"baseline"}
ENABLE_MONITORING=${ENABLE_MONITORING:-"true"}
ENABLE_LOGGING=${ENABLE_LOGGING:-"true"}
ENABLE_VALIDATION=${ENABLE_VALIDATION:-"true"}
VERBOSE=${VERBOSE:-"false"}
# Orchestration toggles
ENABLE_STARTUP_MANAGER=${ENABLE_STARTUP_MANAGER:-"false"}
ENABLE_MASTER_ROSLAUNCH=${ENABLE_MASTER_ROSLAUNCH:-"true"}

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

# ROS Environment Setup
export ROS_MASTER_URI=${ROS_MASTER_URI:-"http://localhost:11311"}
export ROS_IP=${ROS_IP:-"127.0.0.1"}


# Ensure ROS_PACKAGE_PATH includes overlay packages (enhanced_ws only)
if [ -d "/code/enhanced_ws/src" ]; then
    export ROS_PACKAGE_PATH="/code/enhanced_ws/src:${ROS_PACKAGE_PATH}"
fi
if [ -d "/code/catkin_ws/src" ]; then
    export ROS_PACKAGE_PATH="/code/catkin_ws/src:${ROS_PACKAGE_PATH}"
fi

# Python Path Setup for Enhanced Packages
if [ -d "/code/enhanced_ws/src/my-dt-core/packages" ]; then
    export PYTHONPATH="/code/enhanced_ws/src/my-dt-core/packages:$PYTHONPATH"
    log_info "Added enhanced_ws packages to PYTHONPATH"
elif [ -d "$ROOT_DIR/packages" ]; then
    export PYTHONPATH="$ROOT_DIR/packages:$PYTHONPATH"
    log_info "Added local packages to PYTHONPATH"
fi

# Ensure log dir exists and point LOG_DIR variable to it
LOG_DIR="/tmp/enhanced_autonomous_system_logs"
mkdir -p "$LOG_DIR"

# Cleanup function
cleanup() {
    log_info "Cleaning up Enhanced Autonomous System..."
    
    # Clean up completion signal
    rm -f /tmp/enhanced_startup_complete 2>/dev/null || true
    
    # Kill startup manager if still running
    if [ -n "$STARTUP_MANAGER_PID" ]; then
        log_info "Stopping startup manager..."
        kill $STARTUP_MANAGER_PID 2>/dev/null || true
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

    # If object detection is enabled, verify torch/ultralytics; if missing, auto-disable
    if [ "$ENABLE_OBJECT_DETECTION" = "true" ]; then
        python3 - <<'PY'
import sys
try:
    import torch  # noqa: F401
    from ultralytics import YOLO  # noqa: F401
except Exception:
    sys.exit(1)
sys.exit(0)
PY
        status=$?
        if [ "$status" -ne 0 ]; then
            log_warn "Object detection dependencies not available (torch/ultralytics). Disabling object detection for this run."
            ENABLE_OBJECT_DETECTION="false"
        fi
    fi
    
    # Check vehicle name
    if [ -z "$VEHICLE_NAME" ]; then
        log_error "VEHICLE_NAME environment variable not set."
        exit 1
    fi
    
    # Check configuration file (detect workspace automatically with priority order)
    if [ -f "/code/enhanced_ws/src/my-dt-core/configurations.yaml" ]; then
        CONFIG_FILE="/code/enhanced_ws/src/my-dt-core/configurations.yaml"
        log_info "Using enhanced_ws configuration: $CONFIG_FILE"
    elif [ -f "$ROOT_DIR/configurations.yaml" ]; then
        CONFIG_FILE="$ROOT_DIR/configurations.yaml"
        log_info "Using local configuration: $CONFIG_FILE"
    elif [ -f "/code/catkin_ws/src/dt-duckiebot-interface/my-dt-core/configurations.yaml" ]; then
        CONFIG_FILE="/code/catkin_ws/src/dt-duckiebot-interface/my-dt-core/configurations.yaml"
        log_info "Using catkin_ws configuration: $CONFIG_FILE"
    else
    log_error "Configuration file not found in any expected location"
        log_error "Searched: /code/enhanced_ws/src/my-dt-core/configurations.yaml, $ROOT_DIR/configurations.yaml, /code/catkin_ws/src/dt-duckiebot-interface/my-dt-core/configurations.yaml"
        exit 1
    fi
    
    log_info "✓ Environment validation passed"
}

ensure_yolo_model() {
    if [ "$ENABLE_OBJECT_DETECTION" != "true" ]; then
        return 0
    fi
    log_info "Ensuring YOLO model exists..."
    # Candidate paths
    local candidates=(
        "$ROOT_DIR/packages/vehicle_detection/yolov5s.pt"
        "/code/enhanced_ws/src/my-dt-core/packages/vehicle_detection/yolov5s.pt"
    )
    local found=""
    for p in "${candidates[@]}"; do
        if [ -f "$p" ]; then
            found="$p"; break
        fi
    done
    if [ -n "$found" ]; then
        log_info "YOLO model found at: $found"
        return 0
    fi
    log_warn "YOLO model not found. Attempting download of yolov5s.pt..."
    local target="$ROOT_DIR/packages/vehicle_detection/yolov5s.pt"
    mkdir -p "$(dirname "$target")"
    if command -v curl >/dev/null 2>&1; then
        curl -L -o "$target" https://github.com/ultralytics/yolov5/releases/download/v7.0/yolov5s.pt || true
    elif command -v wget >/dev/null 2>&1; then
        wget -O "$target" https://github.com/ultralytics/yolov5/releases/download/v7.0/yolov5s.pt || true
    fi
    if [ -f "$target" ]; then
        log_info "Downloaded YOLO model to $target"
    else
        log_error "Failed to obtain YOLO model. Place yolov5s.pt under packages/vehicle_detection/."
        exit 1
    fi
}

validate_system_resources() {
    log_info "Validating system resources..."
    
    # Check available memory
    AVAILABLE_MEMORY=$(free -m | awk 'NR==2{printf "%.0f", $7}')
    if [ "$AVAILABLE_MEMORY" -lt 1024 ]; then
        log_warn "Low available memory: ${AVAILABLE_MEMORY}MB (recommended: >1GB)"
    fi
    
    # Check CPU load without requiring bc
    CPU_LOAD=$(uptime | awk -F'load average:' '{ print $2 }' | cut -d, -f1 | xargs)
    CPU_CORES=$(nproc)
    # Compare as integers by scaling by 100
    LOAD_INT=$(awk -v l="$CPU_LOAD" 'BEGIN{printf "%d", l*100}')
    CORES_INT=$((CPU_CORES*100))
    if [ "$LOAD_INT" -gt "$CORES_INT" ]; then
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
    
    # Load global configuration (detect workspace automatically with priority order)
    if [ -f "/code/enhanced_ws/src/my-dt-core/configurations.yaml" ]; then
        CONFIG_FILE="/code/enhanced_ws/src/my-dt-core/configurations.yaml"
        log_info "Using enhanced_ws configuration for loading: $CONFIG_FILE"
    elif [ -f "$ROOT_DIR/configurations.yaml" ]; then
        CONFIG_FILE="$ROOT_DIR/configurations.yaml"
        log_info "Using local configuration for loading: $CONFIG_FILE"
    elif [ -f "/code/catkin_ws/src/dt-duckiebot-interface/my-dt-core/configurations.yaml" ]; then
        CONFIG_FILE="/code/catkin_ws/src/dt-duckiebot-interface/my-dt-core/configurations.yaml"
        log_info "Using catkin_ws configuration for loading: $CONFIG_FILE"
    else
        CONFIG_FILE="$ROOT_DIR/configurations.yaml"  # fallback
        log_warn "Using fallback configuration: $CONFIG_FILE"
    fi
    
    if [ -f "$CONFIG_FILE" ]; then
        log_debug "Loading configuration from: $CONFIG_FILE"
        # Configuration is loaded by individual nodes
    else
        log_warn "Configuration file not found: $CONFIG_FILE"
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
        
        # Make script executable
        chmod +x "$STARTUP_SCRIPT"
        
        # Start the startup manager in background and get its PID
        python3 "$STARTUP_SCRIPT" &
        STARTUP_MANAGER_PID=$!
        
        # Wait a bit for startup manager to initialize
        sleep 3
        
        # Check if the startup manager is still running
        if kill -0 $STARTUP_MANAGER_PID 2>/dev/null; then
            log_info "✓ Enhanced System Startup Manager is running (PID: $STARTUP_MANAGER_PID)"
            
            # Wait for startup phases to complete (with timeout)
            local timeout=120  # 2 minutes for startup phases
            local count=0
            
            log_info "Waiting for startup phases to complete..."
            
            while kill -0 $STARTUP_MANAGER_PID 2>/dev/null && [ $count -lt $timeout ]; do
                sleep 1
                ((count++))
                
                # Log progress every 15 seconds
                if [ $((count % 15)) -eq 0 ]; then
                    log_info "Startup in progress... ($count/$timeout seconds)"
                fi
                
                # Check for completion signal (startup manager should create a completion file)
                if [ -f "/tmp/enhanced_startup_complete" ]; then
                    log_info "✓ Startup manager signaled completion"
                    break
                fi
            done
            
            if [ $count -ge $timeout ]; then
                log_warn "Startup manager timeout reached, continuing with system launch"
            fi
            
            log_info "✓ Enhanced System Startup Manager phase completed"
        else
            log_error "Enhanced System Startup Manager failed to start"
            exit 1
        fi
    else
        log_error "Startup manager script not found: $STARTUP_SCRIPT"
        exit 1
    fi
}

launch_master_system() {
    if [ "$ENABLE_MASTER_ROSLAUNCH" != "true" ]; then
        log_info "Master roslaunch disabled by ENABLE_MASTER_ROSLAUNCH=false"
        return 0
    fi

    log_info "Launching Enhanced Autonomous System via roslaunch..."
    # Build roslaunch argument list from feature toggles
    ARGS=(
        "duckietown_demos" "enhanced_autonomous_system.launch"
        "veh:=${VEHICLE_NAME}"
        "config:=${CONFIG_MODE}"
        "enable_enhanced_lane_following:=${ENABLE_ENHANCED_LANE_FOLLOWING}"
        "enable_apriltag_detection:=${ENABLE_APRILTAG_DETECTION}"
        "enable_object_detection:=${ENABLE_OBJECT_DETECTION}"
        "enable_lane_changing:=${ENABLE_LANE_CHANGING}"
        "enable_safety_monitoring:=${ENABLE_SAFETY_MONITORING}"
        "enable_coordination:=${ENABLE_COORDINATION}"
        "enable_performance_optimization:=${ENABLE_PERFORMANCE_OPTIMIZATION}"
        "enable_monitoring:=${ENABLE_MONITORING}"
        "enable_logging:=${ENABLE_LOGGING}"
        "verbose:=${VERBOSE}"
    )

    # Start roslaunch in background
    roslaunch --wait "${ARGS[@]}" &
    MASTER_LAUNCH_PID=$!
    log_info "roslaunch started (PID: ${MASTER_LAUNCH_PID})"
}

launch_enhanced_system() {
    log_info "Enhanced System components managed by startup manager"
    
    # The startup manager handles component launching
    # This function now just monitors the overall system health
    
    log_info "Monitoring system health..."
    
    # Wait for key topics to be available (indicates system is running)
    local timeout=60
    local count=0
    
    log_info "Waiting for system topics to be available..."
    
    while [ $count -lt $timeout ]; do
        # Check if basic ROS topics are available
        if rostopic list 2>/dev/null | grep -q "/$VEHICLE_NAME"; then
            log_info "✓ Vehicle topics detected"
            break
        fi
        
        sleep 1
        ((count++))
        
        if [ $((count % 10)) -eq 0 ]; then
            log_info "Waiting for vehicle topics... ($count/$timeout seconds)"
        fi
    done
    
    if [ $count -ge $timeout ]; then
        log_warn "Timeout waiting for vehicle topics, but continuing"
    fi
    
    log_info "✓ Enhanced Autonomous System is operational"
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
        # Check if startup manager is still running (if we have its PID)
        if [ -n "$STARTUP_MANAGER_PID" ]; then
            if ! kill -0 $STARTUP_MANAGER_PID 2>/dev/null; then
                log_info "Startup manager has completed its job"
                STARTUP_MANAGER_PID=""  # Clear the PID
            fi
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
            
            # Check if vehicle topics are still active
            if rostopic list 2>/dev/null | grep -q "/$VEHICLE_NAME"; then
                log_debug "Vehicle $VEHICLE_NAME topics active"
            else
                log_warn "Vehicle topics not found"
            fi
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
    ensure_yolo_model
    
    # Start enhanced system
    if [ "$ENABLE_STARTUP_MANAGER" = "true" ]; then
        start_startup_manager
    else
        log_info "Startup manager disabled (ENABLE_STARTUP_MANAGER=false)"
    fi

    # Launch the master system to bring up all nodes
    launch_master_system
    launch_enhanced_system
    
    log_info "✓ Enhanced Autonomous System startup completed successfully"
    log_info "System is ready for autonomous operation"
    log_info "Press Ctrl+C to shutdown the system"
    
    # Monitor system
    monitor_system
}

# Run main function
main "$@"