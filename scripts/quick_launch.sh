#!/bin/bash
# Quick launch script for Enhanced Autonomous System
# This script handles all the setup and launches the system

set -e

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# Function to log with colors
log_info() {
    echo -e "${BLUE}[INFO]${NC} $1"
}

log_success() {
    echo -e "${GREEN}[SUCCESS]${NC} $1"
}

log_warn() {
    echo -e "${YELLOW}[WARN]${NC} $1"
}

log_error() {
    echo -e "${RED}[ERROR]${NC} $1"
}

# Args
REBUILD=false
for arg in "$@"; do
    case "$arg" in
        --rebuild)
            REBUILD=true
            shift
            ;;
    esac
done

# Configuration
VEHICLE_NAME=${VEHICLE_NAME:-"blueduckie"}

echo -e "${GREEN}"
echo "╔══════════════════════════════════════════════════════════════╗"
echo "║                Enhanced Autonomous System                    ║"
echo "║                      Quick Launch                            ║"
echo "╚══════════════════════════════════════════════════════════════╝"
echo -e "${NC}"

log_info "Vehicle Name: $VEHICLE_NAME"
echo

# Step 1: Check if we're in the right directory
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT_DIR="$(dirname "$SCRIPT_DIR")"

if [ ! -f "$ROOT_DIR/launchers/enhanced_autonomous_system.sh" ]; then
    log_error "Cannot find enhanced_autonomous_system.sh launcher"
    log_error "Make sure you're running this from the my-dt-core directory"
    exit 1
fi

# Step 2: Set up workspace paths
log_info "Setting up workspace paths..."

WORKSPACE_PATHS=(
    "/code/enhanced_ws"
    "/code/catkin_ws"
    "$ROOT_DIR"
)

ACTIVE_WORKSPACE=""
for ws_path in "${WORKSPACE_PATHS[@]}"; do
    if [ -d "$ws_path" ] && [ -f "$ws_path/devel/setup.bash" ]; then
        ACTIVE_WORKSPACE="$ws_path"
        log_success "Found active workspace: $ACTIVE_WORKSPACE"
        break
    fi
done

if [ -z "$ACTIVE_WORKSPACE" ]; then
    # Try to find unbuilt workspace and build it
    for ws_path in "${WORKSPACE_PATHS[@]:0:2}"; do  # Only check container paths
        if [ -d "$ws_path/src" ]; then
            log_info "Found unbuilt workspace: $ws_path"
            log_info "Building workspace..."
            cd "$ws_path"
            source /opt/ros/noetic/setup.bash 2>/dev/null || source /opt/ros/melodic/setup.bash
            catkin build
            if [ -f "$ws_path/devel/setup.bash" ]; then
                ACTIVE_WORKSPACE="$ws_path"
                log_success "Workspace built successfully: $ACTIVE_WORKSPACE"
                break
            fi
        fi
    done
fi

if [ -z "$ACTIVE_WORKSPACE" ]; then
    log_error "No valid workspace found or buildable"
    exit 1
fi

# Step 3: Source workspaces (base then overlay)
log_info "Sourcing workspace environments..."
SOURCED_ANY=false
if [ -f "/code/catkin_ws/devel/setup.bash" ]; then
    # Source base workspace first for drivers and standard dt-core
    # shellcheck disable=SC1091
    source "/code/catkin_ws/devel/setup.bash"
    log_info "Sourced base workspace: /code/catkin_ws"
    SOURCED_ANY=true
fi
if [ -f "/code/enhanced_ws/devel/setup.bash" ]; then
    # shellcheck disable=SC1091
    source "/code/enhanced_ws/devel/setup.bash"
    log_info "Sourced overlay workspace: /code/enhanced_ws"
    SOURCED_ANY=true
fi
if [ "$SOURCED_ANY" = false ]; then
    # Fallback to whatever we detected as active
    # shellcheck disable=SC1091
    source "$ACTIVE_WORKSPACE/devel/setup.bash"
    log_info "Sourced active workspace: $ACTIVE_WORKSPACE"
fi
log_success "Environment sourced"

# Optional rebuild step
if [ "$REBUILD" = true ]; then
    echo
    log_warn "--rebuild specified: rebuilding workspaces before launch"
    if [ -d "/code/catkin_ws/src" ]; then
        log_info "Rebuilding base workspace (/code/catkin_ws)"
        pushd /code/catkin_ws >/dev/null
        catkin build
        popd >/dev/null
    fi
    if [ -d "/code/enhanced_ws/src" ]; then
        log_info "Rebuilding overlay workspace (/code/enhanced_ws)"
        pushd /code/enhanced_ws >/dev/null
        catkin build
        popd >/dev/null
    fi
    # Re-source after build
    if [ -f "/code/catkin_ws/devel/setup.bash" ]; then
        # shellcheck disable=SC1091
        source "/code/catkin_ws/devel/setup.bash"
    fi
    if [ -f "/code/enhanced_ws/devel/setup.bash" ]; then
        # shellcheck disable=SC1091
        source "/code/enhanced_ws/devel/setup.bash"
    fi
    log_success "Rebuild complete and environments re-sourced"
fi

# Step 4: Set environment variables
log_info "Setting environment variables..."
export VEHICLE_NAME="$VEHICLE_NAME"
export ROS_MASTER_URI="${ROS_MASTER_URI:-http://localhost:11311}"
export ROS_IP="${ROS_IP:-127.0.0.1}"

# Add packages to Python path (overlay first, then local)
if [ -d "/code/enhanced_ws/src/my-dt-core/packages" ]; then
    export PYTHONPATH="/code/enhanced_ws/src/my-dt-core/packages:$PYTHONPATH"
fi
if [ -d "$ROOT_DIR/packages" ]; then
    export PYTHONPATH="$ROOT_DIR/packages:$PYTHONPATH"
fi

log_success "Environment configured"

# Step 5: Download YOLO model if needed
log_info "Checking YOLO model..."
MODEL_PATHS=(
    "/code/enhanced_ws/src/my-dt-core/packages/vehicle_detection/yolov5s.pt"
    "/code/catkin_ws/src/dt-duckiebot-interface/my-dt-core/packages/vehicle_detection/yolov5s.pt"
    "$ROOT_DIR/packages/vehicle_detection/yolov5s.pt"
)

MODEL_EXISTS=false
for model_path in "${MODEL_PATHS[@]}"; do
    if [ -f "$model_path" ]; then
        log_success "YOLO model found: $model_path"
        MODEL_EXISTS=true
        break
    fi
done

if [ "$MODEL_EXISTS" = false ]; then
    log_warn "YOLO model not found, downloading..."
    if [ -f "$ROOT_DIR/scripts/download_yolo_model.sh" ]; then
        "$ROOT_DIR/scripts/download_yolo_model.sh"
    else
        # Fallback: download directly to vehicle_detection package
        TARGET_MODEL_PATH="$ROOT_DIR/packages/vehicle_detection/yolov5s.pt"
        mkdir -p "$(dirname "$TARGET_MODEL_PATH")"
        if command -v wget >/dev/null 2>&1; then
            wget -O "$TARGET_MODEL_PATH" https://github.com/ultralytics/yolov5/releases/download/v7.0/yolov5s.pt || true
        elif command -v curl >/dev/null 2>&1; then
            curl -L -o "$TARGET_MODEL_PATH" https://github.com/ultralytics/yolov5/releases/download/v7.0/yolov5s.pt || true
        fi
        if [ -f "$TARGET_MODEL_PATH" ]; then
            log_success "Downloaded YOLO model to $TARGET_MODEL_PATH"
        else
            log_error "Failed to download YOLO model. Please place yolov5s.pt in packages/vehicle_detection/"
            exit 1
        fi
    fi
fi

# Step 6: Fix permissions
log_info "Checking file permissions..."
EXECUTABLE_FILES=(
    "$ROOT_DIR/launchers/enhanced_autonomous_system.sh"
    "$ROOT_DIR/packages/vehicle_detection/src/enhanced_vehicle_detection_node.py"
    "$ROOT_DIR/packages/duckietown_demos/scripts/enhanced_system_startup.py"
)

for file in "${EXECUTABLE_FILES[@]}"; do
    if [ -f "$file" ] && [ ! -x "$file" ]; then
        chmod +x "$file"
        log_info "Fixed permissions: $(basename "$file")"
    fi
done

# Step 7: Run validation (optional)
if [ "$1" != "--skip-validation" ]; then
    log_info "Running system validation..."
    if [ -f "$ROOT_DIR/scripts/validate_system.sh" ]; then
        if "$ROOT_DIR/scripts/validate_system.sh"; then
            log_success "System validation passed"
        else
            log_warn "System validation had issues, but continuing..."
            read -p "Continue anyway? (y/N): " -n 1 -r
            echo
            if [[ ! $REPLY =~ ^[Yy]$ ]]; then
                log_info "Launch cancelled by user"
                exit 0
            fi
        fi
    fi
fi

# Step 8: Display pre-launch summary
echo
log_info "=== Launch Summary ==="
log_info "Vehicle Name: $VEHICLE_NAME"
if [ -d "/code/catkin_ws" ]; then
    log_info "Base Workspace: /code/catkin_ws"
fi
if [ -d "/code/enhanced_ws" ]; then
    log_info "Overlay Workspace: /code/enhanced_ws"
else
    log_info "Workspace: $ACTIVE_WORKSPACE"
fi
log_info "ROS Master: $ROS_MASTER_URI"
log_info "ROS IP: $ROS_IP"
echo

# Step 9: Launch the system
log_info "Launching Enhanced Autonomous System..."
echo
log_warn "Press Ctrl+C to stop the system"
echo

cd "$ROOT_DIR"
exec ./launchers/enhanced_autonomous_system.sh
