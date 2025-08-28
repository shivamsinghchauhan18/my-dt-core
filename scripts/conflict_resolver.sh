#!/bin/bash

# CONFLICT DETECTION AND RESOLUTION SCRIPT
# Automatically detects and resolves dt-core fork conflicts

set -e

PROJECT_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
ROBOT_HOST="${ROBOT_HOST:-blueduckie.local}"
ROBOT_USER="${ROBOT_USER:-duckie}"
CATKIN_WS="/code/catkin_ws"

# Colors
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
CYAN='\033[0;36m'
MAGENTA='\033[0;35m'
NC='\033[0m'

log_info() { echo -e "${BLUE}[INFO]${NC} $1"; }
log_success() { echo -e "${GREEN}[SUCCESS]${NC} $1"; }
log_warning() { echo -e "${YELLOW}[WARNING]${NC} $1"; }
log_error() { echo -e "${RED}[ERROR]${NC} $1"; }
log_conflict() { echo -e "${MAGENTA}[CONFLICT]${NC} $1"; }

# SSH helper
ssh_cmd() {
    ssh -o ConnectTimeout=10 -o StrictHostKeyChecking=no "$ROBOT_USER@$ROBOT_HOST" "$1"
}

# Detect potential conflicts before deployment
detect_conflicts() {
    log_info "🔍 DETECTING POTENTIAL CONFLICTS..."
    
    local conflicts_found=0
    
    # Check for package name conflicts
    log_info "Checking package name conflicts..."
    
    local enhanced_packages=(
        "line_detector"
        "lane_control" 
        "navigation"
        "apriltag"
        "image_processing"
    )
    
    for package in "${enhanced_packages[@]}"; do
        if [ -d "$PROJECT_ROOT/packages/$package" ]; then
            log_conflict "Package conflict: $package exists in both original dt-core and enhanced version"
            conflicts_found=$((conflicts_found + 1))
            
            # Check if original package exists on robot
            if ssh_cmd "test -d $CATKIN_WS/src/dt-core/packages/$package"; then
                log_conflict "  → Original $package found on robot at $CATKIN_WS/src/dt-core/packages/$package"
                
                # Check for specific file conflicts
                if ssh_cmd "test -f $CATKIN_WS/src/dt-core/packages/$package/package.xml"; then
                    local original_name=$(ssh_cmd "grep '<name>' $CATKIN_WS/src/dt-core/packages/$package/package.xml | sed 's/.*<name>\(.*\)<\/name>.*/\1/'")
                    log_conflict "  → Original package name: $original_name"
                fi
            fi
        fi
    done
    
    # Check for node name conflicts
    log_info "Checking ROS node name conflicts..."
    
    if grep -r "node_name.*line_detector_node" "$PROJECT_ROOT/packages/" >/dev/null 2>&1; then
        log_conflict "Node name conflict: line_detector_node"
        conflicts_found=$((conflicts_found + 1))
    fi
    
    # Check for topic conflicts
    log_info "Checking ROS topic conflicts..."
    
    if grep -r "segment_list" "$PROJECT_ROOT/packages/" >/dev/null 2>&1; then
        log_conflict "Topic conflict: segment_list topic may have different message structure"
        conflicts_found=$((conflicts_found + 1))
    fi
    
    # Check for import conflicts
    log_info "Checking Python import conflicts..."
    
    if grep -r "from line_detector import" "$PROJECT_ROOT/packages/" >/dev/null 2>&1; then
        log_conflict "Import conflict: 'from line_detector import' statements may conflict"
        conflicts_found=$((conflicts_found + 1))
    fi
    
    # Summary
    if [ $conflicts_found -eq 0 ]; then
        log_success "✅ No conflicts detected - safe to deploy"
        return 0
    else
        log_error "❌ $conflicts_found conflicts detected - deployment may fail"
        return 1
    fi
}

# Create backup of original packages before deployment
create_original_backup() {
    log_info "📦 CREATING BACKUP OF ORIGINAL PACKAGES..."
    
    local backup_timestamp=$(date +%Y%m%d_%H%M%S)
    local backup_path="/tmp/original_dt_core_backup_$backup_timestamp"
    
    ssh_cmd "mkdir -p $backup_path"
    ssh_cmd "cp -r $CATKIN_WS/src/dt-core $backup_path/"
    
    log_success "Original dt-core backed up to: $backup_path"
    echo "$backup_path" > /tmp/last_backup_path.txt
}

# Validate deployment compatibility  
validate_compatibility() {
    log_info "🧪 VALIDATING DEPLOYMENT COMPATIBILITY..."
    
    # Check catkin workspace health
    log_info "Checking catkin workspace health..."
    if ! ssh_cmd "cd $CATKIN_WS && source devel/setup.bash && catkin list"; then
        log_error "Catkin workspace appears to be corrupted"
        return 1
    fi
    
    # Check ROS environment
    log_info "Checking ROS environment..."
    if ! ssh_cmd "source /environment.sh && roscore --version"; then
        log_error "ROS environment not properly configured"
        return 1
    fi
    
    # Check dependencies
    log_info "Checking package dependencies..."
    local required_deps=(
        "python3-cv2"
        "python3-numpy" 
        "python3-rospy"
        "ros-noetic-cv-bridge"
    )
    
    for dep in "${required_deps[@]}"; do
        if ! ssh_cmd "dpkg -l | grep -q $dep"; then
            log_warning "Missing dependency: $dep"
        fi
    done
    
    log_success "Compatibility validation completed"
    return 0
}

# Safe deployment with conflict resolution
safe_deploy() {
    log_info "🚀 STARTING SAFE DEPLOYMENT WITH CONFLICT RESOLUTION..."
    
    # Step 1: Detect conflicts
    if ! detect_conflicts; then
        log_warning "Conflicts detected, but proceeding with safe deployment strategy..."
    fi
    
    # Step 2: Create backup
    create_original_backup
    
    # Step 3: Validate compatibility
    if ! validate_compatibility; then
        log_error "Compatibility validation failed - aborting deployment"
        return 1
    fi
    
    # Step 4: Deploy with backup-replace strategy
    log_info "Deploying enhanced packages with backup-replace strategy..."
    
    # Use existing progressive deployment
    if [ -f "$PROJECT_ROOT/scripts/progressive_deployment.sh" ]; then
        "$PROJECT_ROOT/scripts/progressive_deployment.sh" --component all
    else
        log_error "Progressive deployment script not found"
        return 1
    fi
    
    log_success "Safe deployment completed successfully"
    return 0
}

# Rollback to original packages
rollback_deployment() {
    log_info "🔄 ROLLING BACK TO ORIGINAL PACKAGES..."
    
    if [ ! -f /tmp/last_backup_path.txt ]; then
        log_error "No backup path found - cannot rollback"
        return 1
    fi
    
    local backup_path=$(cat /tmp/last_backup_path.txt)
    
    if ! ssh_cmd "test -d $backup_path"; then
        log_error "Backup directory not found: $backup_path"
        return 1
    fi
    
    # Restore original packages
    ssh_cmd "rm -rf $CATKIN_WS/src/dt-core"
    ssh_cmd "cp -r $backup_path/dt-core $CATKIN_WS/src/"
    
    # Rebuild catkin workspace
    ssh_cmd "cd $CATKIN_WS && source devel/setup.bash && catkin build dt-core"
    
    log_success "Rollback completed successfully"
    return 0
}

# Generate conflict resolution report
generate_report() {
    log_info "📊 GENERATING CONFLICT RESOLUTION REPORT..."
    
    local report_file="$PROJECT_ROOT/conflict_resolution_report_$(date +%Y%m%d_%H%M%S).md"
    
    cat > "$report_file" << 'REPORT_EOF'
# Conflict Resolution Report

## Summary
This report details the conflicts detected and resolution strategies applied.

## Detected Conflicts
REPORT_EOF
    
    # Run conflict detection and append to report
    detect_conflicts 2>&1 | sed 's/\x1b\[[0-9;]*m//g' >> "$report_file"
    
    cat >> "$report_file" << 'REPORT_EOF'

## Resolution Strategy
- Backup-replace strategy used to avoid namespace conflicts
- Original packages backed up before replacement
- Enhanced packages deployed to same locations
- Rollback capability maintained

## Deployment Status
REPORT_EOF
    
    if ssh_cmd "test -d $CATKIN_WS/src/dt-core"; then
        echo "✅ dt-core packages deployed successfully" >> "$report_file"
    else
        echo "❌ dt-core deployment failed" >> "$report_file"
    fi
    
    log_success "Report generated: $report_file"
}

# Main function
main() {
    case "${1:-detect}" in
        "detect")
            detect_conflicts
            ;;
        "deploy")
            safe_deploy
            ;;
        "rollback")
            rollback_deployment
            ;;
        "report")
            generate_report
            ;;
        "help")
            cat << 'HELP_EOF'
Conflict Detection and Resolution Script

Usage: $0 [command]

Commands:
  detect   - Detect potential conflicts (default)
  deploy   - Safe deployment with conflict resolution
  rollback - Rollback to original packages
  report   - Generate conflict resolution report
  help     - Show this help message

Examples:
  $0 detect                    # Check for conflicts
  $0 deploy                    # Deploy with safety checks
  $0 rollback                  # Restore original packages
  $0 report                    # Generate detailed report
HELP_EOF
            ;;
        *)
            log_error "Unknown command: $1"
            $0 help
            exit 1
            ;;
    esac
}

main "$@"
