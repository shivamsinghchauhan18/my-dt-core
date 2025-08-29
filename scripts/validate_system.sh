#!/bin/bash
# Pre-launch validation script for Enhanced Autonomous System
# Run this script before launching the autonomous system to check for common issues

set -e

# Function to log with timestamp
log_info() {
    echo "[$(date '+%Y-%m-%d %H:%M:%S')] [INFO] $1"
}

log_warn() {
    echo "[$(date '+%Y-%m-%d %H:%M:%S')] [WARN] $1"
}

log_error() {
    echo "[$(date '+%Y-%m-%d %H:%M:%S')] [ERROR] $1"
}

log_success() {
    echo "[$(date '+%Y-%m-%d %H:%M:%S')] [SUCCESS] $1"
}

log_check() {
    echo "[$(date '+%Y-%m-%d %H:%M:%S')] [CHECK] $1"
}

ERRORS=0
WARNINGS=0

# Source ROS and workspaces to ensure rospack can discover packages
source_envs() {
    # Source ROS distro
    if [ -f /opt/ros/noetic/setup.bash ]; then
        # shellcheck disable=SC1091
        source /opt/ros/noetic/setup.bash
    elif [ -f /opt/ros/melodic/setup.bash ]; then
        # shellcheck disable=SC1091
        source /opt/ros/melodic/setup.bash
    fi
    # Source base then overlay
    if [ -f "/code/catkin_ws/devel/setup.bash" ]; then
        # shellcheck disable=SC1091
        source "/code/catkin_ws/devel/setup.bash"
    fi
    if [ -f "/code/enhanced_ws/devel/setup.bash" ]; then
        # shellcheck disable=SC1091
        source "/code/enhanced_ws/devel/setup.bash"
    fi
}

# Validation functions
check_ros_environment() {
    log_check "Checking ROS environment..."
    
    if [ -z "$ROS_MASTER_URI" ]; then
        log_warn "ROS_MASTER_URI not set, will use default"
        export ROS_MASTER_URI="http://localhost:11311"
    else
        log_info "ROS_MASTER_URI: $ROS_MASTER_URI"
    fi
    
    if [ -z "$ROS_IP" ]; then
        log_warn "ROS_IP not set, will use default"
        export ROS_IP="127.0.0.1"
    else
        log_info "ROS_IP: $ROS_IP"
    fi
    
    if command -v roscore &> /dev/null; then
        log_success "ROS found"
    else
        log_error "ROS not found"
        ERRORS=$((ERRORS + 1))
    fi
}

check_workspace() {
    log_check "Checking workspace setup..."
    
    WORKSPACE_DIRS=(
        "/code/enhanced_ws"
        "/code/catkin_ws"
    )
    
    FOUND_WORKSPACE=false
    for ws_dir in "${WORKSPACE_DIRS[@]}"; do
        if [ -d "$ws_dir" ]; then
            log_info "Found workspace: $ws_dir"
            
            if [ -f "$ws_dir/devel/setup.bash" ]; then
                log_success "Workspace is built: $ws_dir"
                FOUND_WORKSPACE=true
                
                # Check if workspace is sourced
                if [ -n "$CMAKE_PREFIX_PATH" ] && [[ "$CMAKE_PREFIX_PATH" == *"$ws_dir"* ]]; then
                    log_success "Workspace is sourced"
                else
                    log_warn "Workspace not sourced. Run: source $ws_dir/devel/setup.bash"
                    WARNINGS=$((WARNINGS + 1))
                fi
            else
                log_warn "Workspace not built: $ws_dir"
                WARNINGS=$((WARNINGS + 1))
            fi
        fi
    done
    
    if [ "$FOUND_WORKSPACE" = false ]; then
        log_error "No valid workspace found"
        ERRORS=$((ERRORS + 1))
    fi
}

check_vehicle_name() {
    log_check "Checking vehicle name..."
    
    if [ -z "$VEHICLE_NAME" ]; then
        log_warn "VEHICLE_NAME not set, will use default 'blueduckie'"
        WARNINGS=$((WARNINGS + 1))
    else
        log_info "VEHICLE_NAME: $VEHICLE_NAME"
    fi
}

check_packages() {
    log_check "Checking enhanced packages..."
    
    REQUIRED_PACKAGES=(
        "duckietown_enhanced_msgs"
        "vehicle_detection"
        "duckietown_demos"
        "navigation"
        "enhanced_apriltag"
    )
    
    for package in "${REQUIRED_PACKAGES[@]}"; do
        if rospack find "$package" &>/dev/null; then
            log_success "Package found: $package"
        else
            log_warn "Package not found: $package"
            WARNINGS=$((WARNINGS + 1))
        fi
    done
}

check_yolo_model() {
    log_check "Checking YOLO model..."
    
    MODEL_PATHS=(
        "/code/enhanced_ws/src/my-dt-core/packages/vehicle_detection/yolov5s.pt"
        "/code/catkin_ws/src/dt-duckiebot-interface/my-dt-core/packages/vehicle_detection/yolov5s.pt"
    )
    
    FOUND_MODEL=false
    for model_path in "${MODEL_PATHS[@]}"; do
        if [ -f "$model_path" ]; then
            log_success "YOLO model found: $model_path"
            MODEL_SIZE=$(du -h "$model_path" | cut -f1)
            log_info "Model size: $MODEL_SIZE"
            FOUND_MODEL=true
            break
        fi
    done
    
    if [ "$FOUND_MODEL" = false ]; then
        log_error "YOLO model not found. Run: scripts/download_yolo_model.sh"
        ERRORS=$((ERRORS + 1))
    fi
}

check_python_dependencies() {
    log_check "Checking Python dependencies..."
    
    # Always required
    REQUIRED_MODULES=(
        "cv2"
        "numpy"
        "rospy"
    )
    # Conditionally required for object detection
    if [ "${ENABLE_OBJECT_DETECTION:-true}" = "true" ]; then
        REQUIRED_MODULES+=("torch" "ultralytics")
    fi
    
    for module in "${REQUIRED_MODULES[@]}"; do
        if python3 -c "import $module" 2>/dev/null; then
            log_success "Python module available: $module"
        else
            log_error "Python module missing: $module"
            ERRORS=$((ERRORS + 1))
        fi
    done
}

check_permissions() {
    log_check "Checking file permissions..."
    
    EXECUTABLE_FILES=(
        "/code/enhanced_ws/src/my-dt-core/launchers/enhanced_autonomous_system.sh"
        "/code/enhanced_ws/src/my-dt-core/packages/vehicle_detection/src/enhanced_vehicle_detection_node.py"
        "/code/enhanced_ws/src/my-dt-core/packages/duckietown_demos/scripts/enhanced_system_startup.py"
    )
    
    for file in "${EXECUTABLE_FILES[@]}"; do
        if [ -f "$file" ]; then
            if [ -x "$file" ]; then
                log_success "Executable: $file"
            else
                log_warn "Not executable: $file"
                chmod +x "$file" 2>/dev/null && log_info "Fixed permissions: $file" || log_error "Cannot fix permissions: $file"
            fi
        else
            log_warn "File not found: $file"
            WARNINGS=$((WARNINGS + 1))
        fi
    done
}

check_roscore() {
    log_check "Checking if roscore is running..."
    
    if rostopic list &>/dev/null; then
        log_success "ROS core is running"
    else
        log_info "ROS core is not running (will be started by launcher)"
    fi
}

check_system_resources() {
    log_check "Checking system resources..."
    
    # Check memory
    if command -v free &> /dev/null; then
        AVAILABLE_MEMORY=$(free -m | awk 'NR==2{printf "%.0f", $7}')
        if [ "$AVAILABLE_MEMORY" -lt 1024 ]; then
            log_warn "Low available memory: ${AVAILABLE_MEMORY}MB (recommended: >1GB)"
            WARNINGS=$((WARNINGS + 1))
        else
            log_success "Available memory: ${AVAILABLE_MEMORY}MB"
        fi
    fi
    
    # Check disk space
    DISK_USAGE=$(df / | awk 'NR==2 {print $5}' | sed 's/%//')
    if [ "$DISK_USAGE" -gt 90 ]; then
        log_warn "High disk usage: ${DISK_USAGE}%"
        WARNINGS=$((WARNINGS + 1))
    else
        log_success "Disk usage: ${DISK_USAGE}%"
    fi
}

# Main validation
main() {
    log_info "=== Enhanced Autonomous System Pre-launch Validation ==="
    log_info "Starting validation checks..."
    echo
    
    # Ensure environment is sourced for reliable rospack detection
    source_envs

    check_ros_environment
    check_workspace
    check_vehicle_name
    check_packages
    check_yolo_model
    check_python_dependencies
    check_permissions
    check_roscore
    check_system_resources
    
    echo
    log_info "=== Validation Summary ==="
    
    if [ $ERRORS -eq 0 ] && [ $WARNINGS -eq 0 ]; then
        log_success "✅ All validation checks passed! System ready for launch."
        exit 0
    elif [ $ERRORS -eq 0 ]; then
        log_warn "⚠️  Validation completed with $WARNINGS warning(s). System should work but may have issues."
        exit 0
    else
        log_error "❌ Validation failed with $ERRORS error(s) and $WARNINGS warning(s)."
        log_error "Please fix the errors before launching the system."
        exit 1
    fi
}

# Run validation
main "$@"
