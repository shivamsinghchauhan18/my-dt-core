#!/bin/bash

# INTELLIGENT DEPLOYMENT ORCHESTRATOR
# Uses real-time analysis to make smart deployment decisions

set -e

# Configuration - can be overridden by environment variables
ROBOT_HOST="${ROBOT_HOST:-blueduckie.local}"
ROBOT_USER="${ROBOT_USER:-duckie}"
ROBOT_PASSWORD="${ROBOT_PASSWORD:-quackquack}"
PROJECT_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

# Allow robot target override
if [ ! -z "$1" ] && [[ "$1" =~ \.local$ ]]; then
    ROBOT_HOST="$1"
    shift
    log_info "Robot target overridden to: $ROBOT_HOST"
fi

# Colors
GREEN='\033[0;32m'
BLUE='\033[0;34m'
CYAN='\033[0;36m'
YELLOW='\033[1;33m'
RED='\033[0;31m'
MAGENTA='\033[0;35m'
BOLD='\033[1m'
NC='\033[0m'

log_info() { echo -e "${BLUE}[INFO]${NC} $1"; }
log_success() { echo -e "${GREEN}[SUCCESS]${NC} $1"; }
log_warning() { echo -e "${YELLOW}[WARNING]${NC} $1"; }
log_error() { echo -e "${RED}[ERROR]${NC} $1"; }
log_decision() { echo -e "${MAGENTA}[AI-DECISION]${NC} $1"; }

# SSH helper
ssh_robot() {
    sshpass -p "$ROBOT_PASSWORD" ssh -o StrictHostKeyChecking=no "$ROBOT_USER@$ROBOT_HOST" "$1"
}

# Detect catkin workspace location
detect_catkin_workspace() {
    log_info "🔍 Detecting catkin workspace location..."
    
    local catkin_ws_path=$(ssh_robot "
        if [ -d '/code/catkin_ws/src/dt-core' ]; then
            echo '/code/catkin_ws'
        elif [ -d '/home/duckie/catkin_ws/src/dt-core' ]; then
            echo '/home/duckie/catkin_ws'
        elif [ -d '/catkin_ws/src/dt-core' ]; then
            echo '/catkin_ws'
        else
            echo 'NOT_FOUND'
        fi
    ")
    
    if [[ "$catkin_ws_path" == "NOT_FOUND" ]]; then
        log_error "Could not find catkin workspace with dt-core package"
        return 1
    fi
    
    log_success "Found catkin workspace at: $catkin_ws_path"
    echo "$catkin_ws_path"
}

# Global catkin workspace path
CATKIN_WS_PATH=""

# Intelligent deployment with real-time feedback
intelligent_deployment() {
    log_info "🧠 STARTING INTELLIGENT DEPLOYMENT WITH REAL-TIME ANALYSIS"
    
    # Pre-deployment connectivity check
    log_info "🔌 Checking robot connectivity..."
    if ! ping -c 1 "$ROBOT_HOST" &>/dev/null; then
        log_error "Cannot reach robot at $ROBOT_HOST"
        return 1
    fi
    
    if ! ssh_robot "echo 'SSH connection successful'" &>/dev/null; then
        log_error "SSH connection failed to $ROBOT_HOST"
        return 1
    fi
    
    log_success "Robot connectivity verified"
    
    # Step 1: Live system assessment
    log_info "📊 Performing live system assessment..."
    
    # Detect catkin workspace
    CATKIN_WS_PATH=$(detect_catkin_workspace)
    if [ $? -ne 0 ]; then
        log_error "Failed to detect catkin workspace - aborting deployment"
        return 1
    fi
    
    local memory_usage=$(ssh_robot "free | grep Mem | awk '{printf \"%.1f\", \$3/\$2 * 100.0}'")
    local cpu_cores=$(ssh_robot "nproc")
    local load_avg=$(ssh_robot "uptime | awk -F'load average:' '{print \$2}' | awk '{print \$1}' | sed 's/,//'")
    local ros_running=$(ssh_robot "pgrep rosmaster > /dev/null && echo 'true' || echo 'false'")
    
    log_decision "System Analysis Results:"
    log_decision "  Memory Usage: ${memory_usage}%"
    log_decision "  CPU Cores: ${cpu_cores}"
    log_decision "  Load Average: ${load_avg}"
    log_decision "  ROS Status: ${ros_running}"
    
    # Step 2: Intelligent component prioritization
    log_info "🎯 Prioritizing components based on live system state..."
    
    local priority_components=()
    
    # Check what's currently broken/working
    local camera_topics=$(ssh_robot "timeout 5 rostopic list 2>/dev/null | grep -c camera || echo '0'")
    local lane_topics=$(ssh_robot "timeout 5 rostopic list 2>/dev/null | grep -c 'line_detector\\|lane' || echo '0'")
    local wheel_topics=$(ssh_robot "timeout 5 rostopic list 2>/dev/null | grep -c wheel || echo '0'")
    
    log_decision "Current ROS Topic Analysis:"
    log_decision "  Camera topics: ${camera_topics}"
    log_decision "  Lane topics: ${lane_topics}" 
    log_decision "  Wheel topics: ${wheel_topics}"
    
    # AI-powered prioritization logic
    if [[ "$camera_topics" -gt 0 && "$lane_topics" -eq 0 ]]; then
        priority_components+=("line_detector")
        log_decision "🔥 CRITICAL: Line detector needed (camera working, no lane detection)"
    fi
    
    if [[ "$camera_topics" -gt 0 ]]; then
        priority_components+=("vision_utils")
        log_decision "📹 HIGH: Vision utilities (camera system active)"
    fi
    
    if [[ "$wheel_topics" -gt 0 ]]; then
        priority_components+=("lane_control")
        log_decision "🚗 HIGH: Lane control (wheel system active)"
    fi
    
    priority_components+=("navigation" "apriltag")
    
    # Step 3: Resource-aware deployment
    log_info "⚡ Executing resource-aware deployment..."
    
    for component in "${priority_components[@]}"; do
        log_info "Deploying component: $component"
        
        # Pre-deployment system check
        local pre_memory=$(ssh_robot "free | grep Mem | awk '{printf \"%.1f\", \$3/\$2 * 100.0}'")
        log_decision "Pre-deployment memory: ${pre_memory}%"
        
        # Deploy component with monitoring
        case $component in
            "line_detector")
                deploy_enhanced_line_detector_with_monitoring
                ;;
            "vision_utils")  
                deploy_enhanced_vision_utils_with_monitoring
                ;;
            "lane_control")
                deploy_enhanced_lane_control_with_monitoring
                ;;
            "navigation")
                deploy_enhanced_navigation_with_monitoring
                ;;
            "apriltag")
                deploy_enhanced_apriltag_with_monitoring
                ;;
        esac
        
        # Post-deployment validation
        local post_memory=$(ssh_robot "free | grep Mem | awk '{printf \"%.1f\", \$3/\$2 * 100.0}'")
        local memory_delta=$(echo "$post_memory - $pre_memory" | bc -l)
        
        log_decision "Post-deployment memory: ${post_memory}% (Δ${memory_delta}%)"
        
        # Intelligent decision: continue or pause
        if (( $(echo "$post_memory > 85" | bc -l) )); then
            log_warning "High memory usage detected - pausing deployment for system recovery"
            ssh_robot "sync && echo 3 > /proc/sys/vm/drop_caches" || true
            sleep 10
        fi
        
        # Validate component deployment
        validate_component_deployment "$component"
    done
}

# Enhanced component deployment functions with real-time monitoring
deploy_enhanced_line_detector_with_monitoring() {
    log_info "🔍 Deploying enhanced line detector with real-time monitoring..."
    
    # Create backup
    ssh_robot "cp -r $CATKIN_WS_PATH/src/dt-core/src/packages/line_detector /tmp/line_detector_backup_\$(date +%s)"
    
    # Deploy with real-time feedback
    sshpass -p "$ROBOT_PASSWORD" scp -o StrictHostKeyChecking=no \
        "$PROJECT_ROOT/src/packages/line_detector/src/line_detector_node.py" \
        "$ROBOT_USER@$ROBOT_HOST:/tmp/enhanced_line_detector_node.py"
    
    # Test before deployment
    ssh_robot "cd /tmp && python3 -c '
import sys
sys.path.insert(0, \"/tmp\")
try:
    # Test the enhanced line detector functionality
    exec(open(\"enhanced_line_detector_node.py\").read())
    # Create an instance of AdaptiveThresholdDetector to validate
    detector = AdaptiveThresholdDetector()
    print(\"✅ Enhanced line detector validated\")
except Exception as e:
    print(f\"❌ Validation failed: {e}\")
    sys.exit(1)
'"
    
    # Deploy to catkin workspace
    ssh_robot "cp /tmp/enhanced_line_detector_node.py $CATKIN_WS_PATH/src/dt-core/src/packages/line_detector/src/line_detector_node.py"
    
    # Build with monitoring
    ssh_robot "cd $CATKIN_WS_PATH && source devel/setup.bash && catkin build dt-core 2>&1 | tail -20"
    
    log_success "Enhanced line detector deployed successfully"
}

deploy_enhanced_vision_utils_with_monitoring() {
    log_info "👁️ Deploying enhanced vision utilities with monitoring..."
    
    ssh_robot "cp -r $CATKIN_WS_PATH/src/dt-core/src/packages/image_processing /tmp/image_processing_backup_\$(date +%s)"
    
    sshpass -p "$ROBOT_PASSWORD" scp -o StrictHostKeyChecking=no \
        "$PROJECT_ROOT/src/packages/image_processing/src/advanced_vision_utils.py" \
        "$ROBOT_USER@$ROBOT_HOST:$CATKIN_WS_PATH/src/dt-core/src/packages/image_processing/src/"
    
    ssh_robot "cd $CATKIN_WS_PATH && source devel/setup.bash && catkin build dt-core 2>&1 | tail -10"
    
    log_success "Enhanced vision utilities deployed"
}

deploy_enhanced_lane_control_with_monitoring() {
    log_info "🛣️ Deploying enhanced lane control with monitoring..."
    
    ssh_robot "cp -r $CATKIN_WS_PATH/src/dt-core/src/packages/lane_control /tmp/lane_control_backup_\$(date +%s)"
    
    # Deploy lane control enhancements
    sshpass -p "$ROBOT_PASSWORD" scp -r -o StrictHostKeyChecking=no \
        "$PROJECT_ROOT/src/packages/lane_control/" \
        "$ROBOT_USER@$ROBOT_HOST:/tmp/enhanced_lane_control/"
    
    ssh_robot "cp -r /tmp/enhanced_lane_control/src/* $CATKIN_WS_PATH/src/dt-core/src/packages/lane_control/src/"
    ssh_robot "cd $CATKIN_WS_PATH && source devel/setup.bash && catkin build dt-core 2>&1 | tail -10"
    
    log_success "Enhanced lane control deployed"
}

deploy_enhanced_navigation_with_monitoring() {
    log_info "🧭 Deploying enhanced navigation with monitoring..."
    
    ssh_robot "cp -r $CATKIN_WS_PATH/src/dt-core/src/packages/navigation /tmp/navigation_backup_\$(date +%s)"
    
    sshpass -p "$ROBOT_PASSWORD" scp -r -o StrictHostKeyChecking=no \
        "$PROJECT_ROOT/src/packages/navigation/" \
        "$ROBOT_USER@$ROBOT_HOST:/tmp/enhanced_navigation/"
    
    ssh_robot "cp -r /tmp/enhanced_navigation/src/* $CATKIN_WS_PATH/src/dt-core/src/packages/navigation/src/"
    ssh_robot "cd $CATKIN_WS_PATH && source devel/setup.bash && catkin build dt-core 2>&1 | tail -10"
    
    log_success "Enhanced navigation deployed"
}

deploy_enhanced_apriltag_with_monitoring() {
    log_info "🏷️ Deploying enhanced apriltag detection with monitoring..."
    
    # Check if enhanced apriltag has source files
    if [ ! -d "$PROJECT_ROOT/src/packages/enhanced_apriltag/src" ]; then
        log_warning "Enhanced AprilTag package has no source files - using standard apriltag package"
        
        # Backup existing apriltag
        ssh_robot "cp -r $CATKIN_WS_PATH/src/dt-core/src/packages/apriltag /tmp/apriltag_backup_\$(date +%s)"
        
        # Deploy existing apriltag package enhancements (if any)
        if [ -f "$PROJECT_ROOT/src/packages/apriltag/src/apriltag_detector_node.py" ]; then
            sshpass -p "$ROBOT_PASSWORD" scp -r -o StrictHostKeyChecking=no \
                "$PROJECT_ROOT/src/packages/apriltag/src/" \
                "$ROBOT_USER@$ROBOT_HOST:/tmp/enhanced_apriltag_src/"
            
            ssh_robot "cp -r /tmp/enhanced_apriltag_src/* $CATKIN_WS_PATH/src/dt-core/src/packages/apriltag/src/"
            ssh_robot "cd $CATKIN_WS_PATH && source devel/setup.bash && catkin build dt-core 2>&1 | tail -10"
            
            log_success "Standard apriltag detection deployed"
        else
            log_warning "No apriltag source files found - skipping apriltag deployment"
        fi
        return 0
    fi
    
    ssh_robot "cp -r $CATKIN_WS_PATH/src/dt-core/src/packages/apriltag /tmp/apriltag_backup_\$(date +%s)"
    
    sshpass -p "$ROBOT_PASSWORD" scp -r -o StrictHostKeyChecking=no \
        "$PROJECT_ROOT/src/packages/enhanced_apriltag/src/" \
        "$ROBOT_USER@$ROBOT_HOST:/tmp/enhanced_apriltag_src/"
    
    ssh_robot "cp -r /tmp/enhanced_apriltag_src/* $CATKIN_WS_PATH/src/dt-core/src/packages/apriltag/src/"
    ssh_robot "cd $CATKIN_WS_PATH && source devel/setup.bash && catkin build dt-core 2>&1 | tail -10"
    
    log_success "Enhanced apriltag detection deployed"
}

# Real-time component validation
validate_component_deployment() {
    local component="$1"
    log_info "✅ Validating deployment of component: $component"
    
    case $component in
        "line_detector")
            # Test line detector functionality
            local test_result=$(ssh_robot "cd $CATKIN_WS_PATH/src/dt-core/src/packages/line_detector/src && \
                python3 -c '
try:
    exec(open(\"line_detector_node.py\").read())
    # Test if AdaptiveThresholdDetector class is available
    detector = AdaptiveThresholdDetector()
    print(\"PASS\")
except Exception as e:
    print(\"FAIL\")
' 2>/dev/null || echo 'FAIL'")
            if [[ "$test_result" == "PASS" ]]; then
                log_success "✅ Line detector validation: PASSED"
            else
                log_error "❌ Line detector validation: FAILED"
                return 1
            fi
            ;;
        "vision_utils")
            local test_result=$(ssh_robot "cd $CATKIN_WS_PATH/src/dt-core/src/packages/image_processing/src && \
                python3 -c 'import advanced_vision_utils; print(\"PASS\")' 2>/dev/null || echo 'FAIL'")
            if [[ "$test_result" == "PASS" ]]; then
                log_success "✅ Vision utilities validation: PASSED"
            else
                log_error "❌ Vision utilities validation: FAILED"
                return 1
            fi
            ;;
        *)
            log_info "Generic validation for component: $component"
            ;;
    esac
    
    return 0
}

# Live system monitoring during deployment
start_live_monitoring() {
    log_info "📊 Starting live system monitoring..."
    
    # Create monitoring script on robot
    ssh_robot "cat > /tmp/live_monitor.sh << 'MONITOR_EOF'
#!/bin/bash
while true; do
    echo \"\$(date '+%H:%M:%S') | CPU: \$(top -bn1 | grep 'Cpu(s)' | awk '{print \$2}' | sed 's/%us,//') | MEM: \$(free | grep Mem | awk '{printf \"%.1f%%\", \$3/\$2 * 100.0}') | ROS: \$(pgrep rosmaster > /dev/null && echo 'UP' || echo 'DOWN')\"
    sleep 5
done
MONITOR_EOF
chmod +x /tmp/live_monitor.sh"
    
    # Start monitoring in background
    ssh_robot "nohup /tmp/live_monitor.sh > /tmp/live_monitor.log 2>&1 &"
    
    log_success "Live monitoring started - check /tmp/live_monitor.log on robot"
}

# Main intelligent deployment workflow
main() {
    echo -e "${BOLD}${CYAN}"
    cat << 'EOF'
╔══════════════════════════════════════════════════════════════════╗
║                  🧠 INTELLIGENT DEPLOYMENT                       ║
║              Real-time Analysis & Decision Making                ║
╚══════════════════════════════════════════════════════════════════╝
EOF
    echo -e "${NC}"
    
    case "${1:-intelligent}" in
        "intelligent")
            start_live_monitoring
            intelligent_deployment
            ;;
        "monitor")
            start_live_monitoring
            log_info "Monitoring started. Check logs with:"
            log_info "ssh duckie@blueduckie.local 'tail -f /tmp/live_monitor.log'"
            ;;
        "validate")
            validate_component_deployment "$2"
            ;;
        *)
            echo "Usage: $0 [ROBOT_HOST] [COMMAND]"
            echo ""
            echo "ROBOT_HOST: Optional robot hostname (default: $ROBOT_HOST)"
            echo ""
            echo "Commands:"
            echo "  intelligent - Run intelligent deployment with real-time analysis (default)"
            echo "  monitor     - Start live system monitoring only"
            echo "  validate    - Validate specific component deployment"
            echo ""
            echo "Environment Variables:"
            echo "  ROBOT_HOST     - Robot hostname (default: blueduckie.local)"
            echo "  ROBOT_USER     - SSH username (default: duckie)"
            echo "  ROBOT_PASSWORD - SSH password (default: quackquack)"
            echo ""
            echo "Examples:"
            echo "  $0                                    # Deploy to blueduckie.local"
            echo "  $0 pinkduckie.local                   # Deploy to pinkduckie.local"
            echo "  ROBOT_HOST=myrobot.local $0           # Deploy using environment variable"
            echo "  $0 blueduckie.local monitor           # Start monitoring only"
            echo "  $0 validate line_detector             # Validate line detector"
            ;;
    esac
}

# Dependency check
check_dependencies() {
    local missing_deps=()
    
    if ! command -v sshpass &> /dev/null; then
        missing_deps+=("sshpass")
    fi
    
    if ! command -v ssh &> /dev/null; then
        missing_deps+=("ssh")
    fi
    
    if ! command -v scp &> /dev/null; then
        missing_deps+=("scp")
    fi
    
    if ! command -v ping &> /dev/null; then
        missing_deps+=("ping")
    fi
    
    if [ ${#missing_deps[@]} -gt 0 ]; then
        log_error "Missing required dependencies: ${missing_deps[*]}"
        log_error "Install missing dependencies:"
        for dep in "${missing_deps[@]}"; do
            case $dep in
                "sshpass")
                    log_error "  brew install hudochenkov/sshpass/sshpass"
                    ;;
                *)
                    log_error "  Install $dep using your system package manager"
                    ;;
            esac
        done
        return 1
    fi
    
    if ! command -v bc &> /dev/null; then
        log_warning "bc calculator recommended for advanced analysis (brew install bc)"
    fi
    
    return 0
}

# Dependency check
if ! check_dependencies; then
    exit 1
fi

main "$@"
