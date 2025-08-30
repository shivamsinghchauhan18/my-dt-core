#!/bin/bash

# REAL-TIME ROBOT ANALYSIS & DECISION MAKING FRAMEWORK
# Comprehensive diagnostic and deployment system for blueduckie.local

set -e

ROBOT_HOST="blueduckie.local"
ROBOT_USER="duckie"
ROBOT_PASSWORD="quackquack"
PROJECT_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
CYAN='\033[0;36m'
MAGENTA='\033[0;35m'
BOLD='\033[1m'
NC='\033[0m'

# Logging functions
log_header() { echo -e "\n${BOLD}${CYAN}━━━ $1 ━━━${NC}"; }
log_info() { echo -e "${BLUE}[INFO]${NC} $1"; }
log_success() { echo -e "${GREEN}[SUCCESS]${NC} $1"; }
log_warning() { echo -e "${YELLOW}[WARNING]${NC} $1"; }
log_error() { echo -e "${RED}[ERROR]${NC} $1"; }
log_analysis() { echo -e "${MAGENTA}[ANALYSIS]${NC} $1"; }

# SSH command helper with password
ssh_robot() {
    sshpass -p "$ROBOT_PASSWORD" ssh -o StrictHostKeyChecking=no "$ROBOT_USER@$ROBOT_HOST" "$1"
}

# SCP helper with password
scp_robot() {
    sshpass -p "$ROBOT_PASSWORD" scp -o StrictHostKeyChecking=no "$1" "$ROBOT_USER@$ROBOT_HOST:$2"
}

# Real-time system analysis
analyze_robot_system() {
    log_header "REAL-TIME ROBOT SYSTEM ANALYSIS"
    
    # Basic system info
    log_info "Gathering system information..."
    ssh_robot "echo '=== SYSTEM INFO ===' && \
               uname -a && \
               echo -e '\n=== UPTIME ===' && \
               uptime && \
               echo -e '\n=== MEMORY USAGE ===' && \
               free -h && \
               echo -e '\n=== DISK USAGE ===' && \
               df -h && \
               echo -e '\n=== CPU INFO ===' && \
               lscpu | head -10"
    
    # ROS environment analysis
    log_info "Analyzing ROS environment..."
    ssh_robot "echo '=== ROS MASTER STATUS ===' && \
               (pgrep rosmaster > /dev/null && echo 'ROS Master: RUNNING' || echo 'ROS Master: NOT RUNNING') && \
               echo -e '\n=== ROS ENVIRONMENT ===' && \
               env | grep ROS && \
               echo -e '\n=== ACTIVE ROS NODES ===' && \
               (rosnode list 2>/dev/null || echo 'No ROS nodes detected') && \
               echo -e '\n=== ACTIVE ROS TOPICS ===' && \
               (rostopic list 2>/dev/null | head -20 || echo 'No ROS topics detected')"
    
    # Catkin workspace analysis
    log_info "Analyzing catkin workspace..."
    ssh_robot "echo '=== CATKIN WORKSPACE ===' && \
               ls -la /code/catkin_ws/ && \
               echo -e '\n=== DT-CORE PACKAGES ===' && \
               ls -la /code/catkin_ws/src/dt-core/src/packages/ | head -20 && \
               echo -e '\n=== BUILD STATUS ===' && \
               (test -d /code/catkin_ws/build && echo 'Build directory exists' || echo 'No build directory') && \
               (test -d /code/catkin_ws/devel && echo 'Devel directory exists' || echo 'No devel directory')"
}

# Real-time performance monitoring
monitor_robot_performance() {
    log_header "REAL-TIME PERFORMANCE MONITORING"
    
    log_info "Monitoring system performance for 30 seconds..."
    ssh_robot "echo '=== CPU & MEMORY MONITORING ===' && \
               for i in {1..6}; do \
                   echo \"Sample \$i:\" && \
                   top -bn1 | head -15 && \
                   echo '---' && \
                   sleep 5; \
               done"
    
    log_info "Checking camera and sensor status..."
    ssh_robot "echo '=== CAMERA STATUS ===' && \
               (ls /dev/video* 2>/dev/null && echo 'Camera devices found' || echo 'No camera devices') && \
               echo -e '\n=== USB DEVICES ===' && \
               lsusb && \
               echo -e '\n=== NETWORK INTERFACES ===' && \
               ip addr show | grep -E '^[0-9]+:|inet '"
}

# Analyze current dt-core implementation
analyze_current_dtcore() {
    log_header "ANALYZING CURRENT DT-CORE IMPLEMENTATION"
    
    # Check line detector
    log_info "Analyzing line detector implementation..."
    ssh_robot "echo '=== LINE DETECTOR FILES ===' && \
               find /code/catkin_ws/src/dt-core/src/packages/line_detector -name '*.py' -exec echo 'File: {}' \; -exec head -20 {} \; -exec echo '---' \; 2>/dev/null"
    
    # Check launch files
    log_info "Checking launch files..."
    ssh_robot "echo '=== LAUNCH FILES ===' && \
               find /code/catkin_ws/src/dt-core -name '*.launch' | head -10 | while read file; do \
                   echo \"File: \$file\" && \
                   head -10 \"\$file\" && \
                   echo '---'; \
               done"
    
    # Check running processes
    log_info "Checking Duckiebot processes..."
    ssh_robot "echo '=== DUCKIEBOT PROCESSES ===' && \
               ps aux | grep -E '(ros|duckie|python)' | grep -v grep"
}

# Test enhanced component deployment
test_enhanced_deployment() {
    log_header "TESTING ENHANCED COMPONENT DEPLOYMENT"
    
    # Create staging area
    log_info "Creating staging area on robot..."
    ssh_robot "mkdir -p /tmp/enhanced_staging && \
               mkdir -p /tmp/backups && \
               mkdir -p /tmp/test_results"
    
    # Deploy test component (line detector)
    log_info "Deploying enhanced line detector for testing..."
    scp_robot "$PROJECT_ROOT/src/packages/line_detector/src/line_detector_node.py" "/tmp/enhanced_staging/"
    
    # Test the enhanced component
    log_info "Testing enhanced line detector..."
    ssh_robot "cd /tmp/enhanced_staging && \
               python3 -c '
import sys
sys.path.insert(0, \"/tmp/enhanced_staging\")
try:
    from line_detector_node import AdaptiveThresholdDetector, TemporalConsistencyFilter
    print(\"✅ Enhanced components imported successfully\")
    
    # Test adaptive detector
    detector = AdaptiveThresholdDetector()
    print(f\"✅ AdaptiveThresholdDetector initialized: base_low={detector.base_canny_low}, base_high={detector.base_canny_high}\")
    
    # Test temporal filter
    filter = TemporalConsistencyFilter()
    print(f\"✅ TemporalConsistencyFilter initialized: history_size={filter.history_size}\")
    
    print(\"✅ All enhanced components functional\")
except Exception as e:
    print(f\"❌ Error testing enhanced components: {e}\")
    import traceback
    traceback.print_exc()
'"
}

# Real-time decision making based on analysis
make_deployment_decisions() {
    log_header "REAL-TIME DEPLOYMENT DECISION MAKING"
    
    # Analyze system readiness
    log_analysis "Analyzing system readiness for deployment..."
    
    local system_ready=true
    local memory_usage=$(ssh_robot "free | grep Mem | awk '{printf \"%.1f\", \$3/\$2 * 100.0}'")
    local cpu_load=$(ssh_robot "uptime | awk -F'load average:' '{print \$2}' | awk '{print \$1}' | sed 's/,//'")
    local disk_usage=$(ssh_robot "df /code | tail -1 | awk '{print \$5}' | sed 's/%//'")
    
    log_analysis "System metrics:"
    log_analysis "  Memory usage: ${memory_usage}%"
    log_analysis "  CPU load: ${cpu_load}"
    log_analysis "  Disk usage: ${disk_usage}%"
    
    # Decision logic
    if (( $(echo "$memory_usage > 80" | bc -l) )); then
        log_warning "High memory usage detected (${memory_usage}%) - recommend cleanup before deployment"
        system_ready=false
    fi
    
    if (( disk_usage > 90 )); then
        log_warning "High disk usage detected (${disk_usage}%) - recommend cleanup before deployment"
        system_ready=false
    fi
    
    # Check ROS status
    if ! ssh_robot "pgrep rosmaster > /dev/null"; then
        log_warning "ROS master not running - will need to start before deployment"
        system_ready=false
    fi
    
    # Make deployment recommendation
    if [ "$system_ready" = true ]; then
        log_success "✅ SYSTEM READY FOR DEPLOYMENT"
        log_info "Recommended deployment strategy: Progressive component-by-component"
        return 0
    else
        log_error "❌ SYSTEM NOT READY FOR DEPLOYMENT"
        log_info "Recommended actions: Address warnings above before proceeding"
        return 1
    fi
}

# Intelligent component prioritization
prioritize_deployment_components() {
    log_header "INTELLIGENT COMPONENT PRIORITIZATION"
    
    # Analyze current system performance
    local camera_active=$(ssh_robot "rostopic echo -n1 /blueduckie/camera_node/image/compressed --timeout=3 > /dev/null 2>&1 && echo 'true' || echo 'false'")
    local wheels_active=$(ssh_robot "rostopic echo -n1 /blueduckie/wheels_driver_node/wheels_cmd_executed --timeout=3 > /dev/null 2>&1 && echo 'true' || echo 'false'")
    local lane_detection_active=$(ssh_robot "rostopic echo -n1 /blueduckie/line_detector_node/segment_list --timeout=3 > /dev/null 2>&1 && echo 'true' || echo 'false'")
    
    log_analysis "Current system capabilities:"
    log_analysis "  Camera: $camera_active"
    log_analysis "  Wheels: $wheels_active"  
    log_analysis "  Lane Detection: $lane_detection_active"
    
    # Prioritize based on what's working
    local deployment_priority=()
    
    if [ "$camera_active" = "true" ] && [ "$lane_detection_active" = "false" ]; then
        deployment_priority+=("line_detector")
        log_analysis "🎯 HIGH PRIORITY: Line detector (camera working, lane detection broken)"
    fi
    
    if [ "$camera_active" = "true" ]; then
        deployment_priority+=("vision_utils")
        log_analysis "🎯 MEDIUM PRIORITY: Vision utilities (camera available)"
    fi
    
    if [ "$wheels_active" = "true" ]; then
        deployment_priority+=("lane_control")
        log_analysis "🎯 MEDIUM PRIORITY: Lane control (wheels working)"
    fi
    
    deployment_priority+=("navigation")
    deployment_priority+=("apriltag")
    
    log_success "Recommended deployment order: ${deployment_priority[*]}"
    
    # Save priority to file for later use
    printf '%s\n' "${deployment_priority[@]}" > /tmp/deployment_priority.txt
}

# Continuous monitoring during deployment
monitor_deployment_progress() {
    log_header "CONTINUOUS DEPLOYMENT MONITORING"
    
    local component="$1"
    log_info "Starting continuous monitoring for component: $component"
    
    # Monitor in background
    ssh_robot "mkdir -p /tmp/monitoring && \
               cd /tmp/monitoring && \
               while true; do \
                   echo \"\$(date): \$(free -m | grep Mem | awk '{print \$3\"MB/\"\$2\"MB\"}') memory, \$(uptime | awk -F'load average:' '{print \$2}' | awk '{print \$1}')\" load >> deployment_monitor.log; \
                   sleep 5; \
               done" &
    
    local monitor_pid=$!
    echo $monitor_pid > /tmp/monitor_pid.txt
    
    log_success "Deployment monitoring started (PID: $monitor_pid)"
}

# Complete real-time analysis workflow
run_complete_analysis() {
    log_header "COMPLETE REAL-TIME ROBOT ANALYSIS & DEPLOYMENT PLANNING"
    
    echo -e "${BOLD}${CYAN}"
    cat << 'EOF'
╔═══════════════════════════════════════════════════════════════╗
║                    LIVE ROBOT ANALYSIS                        ║
║             Leveraging Full AI Capabilities                   ║
╚═══════════════════════════════════════════════════════════════╝
EOF
    echo -e "${NC}"
    
    # Step 1: System Analysis
    analyze_robot_system
    
    # Step 2: Performance Monitoring  
    monitor_robot_performance
    
    # Step 3: Current Implementation Analysis
    analyze_current_dtcore
    
    # Step 4: Test Enhanced Components
    test_enhanced_deployment
    
    # Step 5: Make Deployment Decisions
    make_deployment_decisions
    
    # Step 6: Prioritize Components
    prioritize_deployment_components
    
    # Step 7: Generate Real-time Report
    generate_realtime_report
}

# Generate comprehensive real-time analysis report
generate_realtime_report() {
    log_header "GENERATING REAL-TIME ANALYSIS REPORT"
    
    local report_file="$PROJECT_ROOT/realtime_analysis_$(date +%Y%m%d_%H%M%S).md"
    
    cat > "$report_file" << 'EOF'
# Real-Time Robot Analysis Report

## Executive Summary
This report contains live analysis of blueduckie.local performed using direct SSH access
and real-time data collection for intelligent deployment decision making.

## System Status
EOF
    
    # Add live system data
    ssh_robot "echo '- Hostname: '\$(hostname) && \
               echo '- Uptime: '\$(uptime | awk '{print \$3, \$4}' | sed 's/,//') && \
               echo '- Memory: '\$(free -h | grep Mem | awk '{print \$3\"/\"\$2}') && \
               echo '- Disk: '\$(df -h /code | tail -1 | awk '{print \$5}') usage" >> "$report_file"
    
    cat >> "$report_file" << 'EOF'

## ROS Environment Status
EOF
    
    ssh_robot "(pgrep rosmaster > /dev/null && echo '✅ ROS Master running' || echo '❌ ROS Master not running') && \
               echo '- Active nodes: '\$(rosnode list 2>/dev/null | wc -l || echo '0') && \
               echo '- Active topics: '\$(rostopic list 2>/dev/null | wc -l || echo '0')" >> "$report_file"
    
    cat >> "$report_file" << 'EOF'

## Deployment Recommendations
Based on real-time analysis, the following deployment strategy is recommended:

EOF
    
    if [ -f /tmp/deployment_priority.txt ]; then
        echo "### Component Deployment Priority:" >> "$report_file"
        cat /tmp/deployment_priority.txt | nl | sed 's/^/  /' >> "$report_file"
    fi
    
    cat >> "$report_file" << 'EOF'

## Live Monitoring Data
Real-time monitoring logs and performance metrics are available in:
- Robot: /tmp/monitoring/deployment_monitor.log
- Local: This analysis session

## Next Steps
1. Address any system warnings identified
2. Execute progressive deployment using prioritized component list
3. Monitor deployment progress with real-time feedback
4. Validate enhanced functionality on live robot

---
*Report generated by AI-powered real-time analysis system*
EOF
    
    log_success "Real-time analysis report generated: $report_file"
}

# Main execution
main() {
    case "${1:-complete}" in
        "system")
            analyze_robot_system
            ;;
        "performance")
            monitor_robot_performance
            ;;
        "dtcore")
            analyze_current_dtcore
            ;;
        "test")
            test_enhanced_deployment
            ;;
        "decide")
            make_deployment_decisions
            ;;
        "prioritize")
            prioritize_deployment_components
            ;;
        "monitor")
            monitor_deployment_progress "$2"
            ;;
        "complete")
            run_complete_analysis
            ;;
        *)
            echo "Usage: $0 [system|performance|dtcore|test|decide|prioritize|monitor|complete]"
            echo ""
            echo "Commands:"
            echo "  system     - Analyze robot system status"
            echo "  performance - Monitor real-time performance"  
            echo "  dtcore     - Analyze current dt-core implementation"
            echo "  test       - Test enhanced component deployment"
            echo "  decide     - Make deployment decisions based on analysis"
            echo "  prioritize - Prioritize deployment components intelligently"
            echo "  monitor    - Start continuous deployment monitoring"
            echo "  complete   - Run complete analysis workflow (default)"
            ;;
    esac
}

# Check dependencies
check_dependencies() {
    if ! command -v sshpass &> /dev/null; then
        log_error "sshpass is required but not installed"
        log_info "Install with: brew install hudochenkov/sshpass/sshpass"
        exit 1
    fi
    
    if ! command -v bc &> /dev/null; then
        log_warning "bc calculator not found - some analysis features may be limited"
    fi
}

# Only run dependency check if not sourced
if [[ "${BASH_SOURCE[0]}" == "${0}" ]]; then
    check_dependencies
    main "$@"
fi
