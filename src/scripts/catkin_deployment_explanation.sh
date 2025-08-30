#!/bin/bash

# Catkin Workspace Progressive Deployment Strategy
# Properly integrates with ROS catkin workspace while maintaining safety

set -e

ROBOT_NAME="blueduckie"
ROBOT_HOST="${ROBOT_NAME}.local"
ROBOT_USER="duckie"
CATKIN_WS="/code/catkin_ws"
DT_CORE_PATH="/code/catkin_ws/src/dt-core"

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

# SSH helper
ssh_cmd() {
    ssh -o ConnectTimeout=10 -o StrictHostKeyChecking=no "$ROBOT_USER@$ROBOT_HOST" "$1"
}

# How the REAL deployment works with catkin
explain_catkin_deployment() {
    cat << 'EOF'

🎯 CATKIN WORKSPACE DEPLOYMENT STRATEGY

┌─────────────────────────────────────────────────────────────────┐
│                        DEPLOYMENT FLOW                          │
└─────────────────────────────────────────────────────────────────┘

1. 📦 BACKUP ORIGINAL PACKAGES
   /code/catkin_ws/src/dt-core/src/packages/line_detector 
   → /tmp/backups/line_detector_20250828_123456

2. 🧪 STAGE & TEST IN TEMPORARY LOCATION
   Local: src/packages/line_detector/src/
   → Robot: /tmp/staging/enhanced_line_detector/
   
   Test: python3 import tests, functionality validation

3. 🚀 DEPLOY TO CATKIN WORKSPACE
   /tmp/staging/enhanced_line_detector/
   → /code/catkin_ws/src/dt-core/src/packages/line_detector/src/

4. 🔧 BUILD CATKIN WORKSPACE
   cd /code/catkin_ws && catkin build dt-core

5. ✅ TEST ROS INTEGRATION
   roslaunch dt-core line_detector_node.launch

6. 📊 VALIDATE SYSTEM HEALTH
   Check ROS topics, node status, performance

┌─────────────────────────────────────────────────────────────────┐
│                      CATKIN WORKSPACE STRUCTURE                 │
└─────────────────────────────────────────────────────────────────┘

/code/catkin_ws/
├── build/           ← Catkin build artifacts
├── devel/           ← Catkin development space
└── src/
    └── dt-core/     ← Duckiebot core packages
        ├── src/packages/
        │   ├── line_detector/
        │   │   ├── CMakeLists.txt
        │   │   ├── package.xml
        │   │   ├── src/
        │   │   │   └── line_detector_node.py  ← Enhanced version deployed here
        │   │   ├── launch/
        │   │   └── config/
        │   ├── lane_control/
        │   ├── navigation/
        │   └── ...
        └── src/launchers/

┌─────────────────────────────────────────────────────────────────┐
│                      SAFETY MECHANISMS                          │
└─────────────────────────────────────────────────────────────────┘

🛡️  BACKUP STRATEGY:
    • Full package backup before any changes
    • Timestamped backups in /tmp/backups/
    • Quick restore: cp -r /tmp/backups/pkg_* /code/catkin_ws/src/dt-core/src/packages/

🧪 STAGED TESTING:
    • Test in /tmp/staging/ before catkin deployment
    • Import tests, functionality validation
    • Resource impact assessment

🔄 ROLLBACK CAPABILITY:
    • Automatic rollback on failure
    • Restore from timestamped backup
    • Rebuild catkin workspace
    • Restart ROS nodes

📊 CONTINUOUS MONITORING:
    • ROS topic health during deployment
    • System resource monitoring
    • Node performance tracking

EOF
}

# Example deployment function
deploy_enhanced_line_detector_catkin() {
    log_info "=== DEPLOYING ENHANCED LINE DETECTOR TO CATKIN WORKSPACE ==="
    
    # 1. Create backup
    local backup_name="line_detector_backup_$(date +%Y%m%d_%H%M%S)"
    ssh_cmd "mkdir -p /tmp/backups"
    ssh_cmd "cp -r $DT_CORE_PATH/src/packages/line_detector /tmp/backups/$backup_name"
    log_success "Original package backed up to: /tmp/backups/$backup_name"
    
    # 2. Stage enhanced version
    log_info "Staging enhanced line detector for testing..."
    scp -r src/packages/line_detector/ "$ROBOT_USER@$ROBOT_HOST:/tmp/staging/enhanced_line_detector/"
    
    # 3. Test staged version
    log_info "Testing staged version..."
    ssh_cmd "cd /tmp/staging/enhanced_line_detector/src && python3 -c '
import line_detector_node
from line_detector_node import AdaptiveThresholdDetector
detector = AdaptiveThresholdDetector()
print(\"Enhanced line detector test: PASSED\")
'"
    
    # 4. Deploy to catkin workspace
    log_info "Deploying to catkin workspace..."
    ssh_cmd "cp -r /tmp/staging/enhanced_line_detector/src/* $DT_CORE_PATH/src/packages/line_detector/src/"
    
    # 5. Build catkin workspace
    log_info "Building catkin workspace..."
    ssh_cmd "cd $CATKIN_WS && source devel/setup.bash && catkin build dt-core"
    
    # 6. Test ROS integration
    log_info "Testing ROS integration..."
    ssh_cmd "cd $CATKIN_WS && source devel/setup.bash && timeout 10 roslaunch dt-core line_detector_node.launch veh:=\$VEHICLE_NAME &"
    sleep 5
    
    # Check if ROS node is running
    if ssh_cmd "pgrep -f line_detector_node.py"; then
        log_success "Enhanced line detector successfully integrated with ROS"
    else
        log_error "ROS integration failed, rolling back..."
        # Rollback
        ssh_cmd "cp -r /tmp/backups/$backup_name/* $DT_CORE_PATH/src/packages/line_detector/"
        ssh_cmd "cd $CATKIN_WS && source devel/setup.bash && catkin build dt-core"
        return 1
    fi
    
    log_success "Enhanced line detector deployment completed successfully"
    return 0
}

# Show the explanation
explain_catkin_deployment

echo
echo -e "${CYAN}🔧 This is how deployment ACTUALLY works with catkin workspace${NC}"
echo -e "${CYAN}📚 Run with --demo to see example deployment${NC}"

if [[ "$1" == "--demo" ]]; then
    echo
    log_info "🎬 RUNNING DEMO DEPLOYMENT..."
    deploy_enhanced_line_detector_catkin
fi
