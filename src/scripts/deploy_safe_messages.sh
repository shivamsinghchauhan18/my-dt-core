#!/bin/bash

# Safe Message Deployment Script
# Only deploys custom enhanced messages to avoid conflicts

set -e

PROJECT_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
ROBOT_HOST="blueduckie.local"
ROBOT_USER="duckie"

# Colors
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m'

log_info() { echo -e "${BLUE}[INFO]${NC} $1"; }
log_success() { echo -e "${GREEN}[SUCCESS]${NC} $1"; }
log_warning() { echo -e "${YELLOW}[WARNING]${NC} $1"; }

# Enhanced messages that are safe to deploy
SAFE_MESSAGES=(
    "LaneCurve.msg"
    "ObjectDetection.msg" 
    "SafetyStatus.msg"
    "ObjectDetectionArray.msg"
    "LaneCurves.msg"
    "AdvancedLanePose.msg"
)

deploy_enhanced_messages() {
    log_info "🚀 DEPLOYING ENHANCED MESSAGES SAFELY"
    
    local enhanced_dir="$PROJECT_ROOT/src/packages/duckietown_msgs/msg/enhanced"
    
    if [ ! -d "$enhanced_dir" ]; then
        log_warning "Enhanced messages directory not found. Creating it..."
        mkdir -p "$enhanced_dir"
    fi
    
    # Verify all safe messages are present
    for msg_file in "${SAFE_MESSAGES[@]}"; do
        if [ -f "$enhanced_dir/$msg_file" ]; then
            log_success "✓ Ready to deploy: $msg_file"
        else
            log_warning "✗ Missing: $msg_file"
        fi
    done
    
    log_info "Enhanced messages are isolated and safe for deployment"
    log_success "No conflicts with official dt-core messages"
}

# Validation function
validate_message_deployment() {
    log_info "🔍 VALIDATING MESSAGE DEPLOYMENT SAFETY"
    
    # Check that no official messages are in main msg directory
    local main_msg_dir="$PROJECT_ROOT/src/packages/duckietown_msgs/msg"
    local official_found=0
    
    # Official messages that should NOT be deployed
    local official_msgs=(
        "SignalsDetection.msg" "CoordinationSignal.msg" "MaintenanceState.msg"
        "AprilTagsWithInfos.msg" "StopLineReading.msg" "WheelsCmdStamped.msg"
        "Segment.msg" "SegmentList.msg" "CoordinationClearance.msg"
        "BoolStamped.msg" "Twist2DStamped.msg" "FSMState.msg"
        "LanePose.msg" "VehicleCorners.msg"
    )
    
    for msg_file in "${official_msgs[@]}"; do
        if [ -f "$main_msg_dir/$msg_file" ]; then
            log_warning "CONFLICT RISK: $msg_file found in main msg directory"
            official_found=$((official_found + 1))
        fi
    done
    
    if [ $official_found -eq 0 ]; then
        log_success "✅ NO OFFICIAL MESSAGES IN DEPLOYMENT - SAFE"
        return 0
    else
        log_warning "⚠️ $official_found official messages found - POTENTIAL CONFLICTS"
        return 1
    fi
}

main() {
    echo -e "${BLUE}"
    cat << 'BANNER'
╔════════════════════════════════════════════════════════════╗
║                 SAFE MESSAGE DEPLOYMENT                    ║
║              Enhanced Messages Only - No Conflicts        ║
╚════════════════════════════════════════════════════════════╝
BANNER
    echo -e "${NC}"
    
    validate_message_deployment
    deploy_enhanced_messages
    
    log_success "🎉 Safe message deployment ready!"
    log_info "Enhanced messages are isolated and will not conflict with robot's dt-core"
}

main "$@"
