#!/bin/bash

# Smart Progressive Deployment Wrapper
# Simple interface for the comprehensive deployment system

set -e

# Configuration
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(dirname "$SCRIPT_DIR")"
ROBOT_NAME="${ROBOT_NAME:-blueduckie}"
ROBOT_HOST="${ROBOT_HOST:-${ROBOT_NAME}.local}"
ROBOT_USER="${ROBOT_USER:-duckie}"

# Colors
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
CYAN='\033[0;36m'
PURPLE='\033[0;35m'
NC='\033[0m'

# Display banner
show_banner() {
    echo -e "${BLUE}"
    echo "╔══════════════════════════════════════════════════════════════════════════╗"
    echo "║                    🤖 SMART PROGRESSIVE DEPLOYMENT 🤖                    ║"
    echo "║                                                                          ║"
    echo "║          Safe, Piece-by-Piece Deployment for Duckiebot Systems          ║"
    echo "║                                                                          ║"
    echo "╚══════════════════════════════════════════════════════════════════════════╝"
    echo -e "${NC}"
    echo
    echo -e "${CYAN}🎯 Target Robot:${NC} $ROBOT_HOST"
    echo -e "${CYAN}📁 Project Root:${NC} $PROJECT_ROOT"
    echo -e "${CYAN}⏰ Session Time:${NC} $(date)"
    echo
}

# Show help
show_help() {
    echo -e "${YELLOW}📖 USAGE:${NC}"
    echo "  ./deploy_smart.sh [OPTIONS] [COMMAND]"
    echo
    echo -e "${YELLOW}🛠️  COMMANDS:${NC}"
    echo "  deploy              - Run full smart deployment (default)"
    echo "  monitor             - Start real-time monitoring only"
    echo "  test <component>    - Test specific component"
    echo "  health              - Check robot health"
    echo "  status              - Show deployment status"
    echo "  logs                - Show recent logs"
    echo "  help                - Show this help"
    echo
    echo -e "${YELLOW}⚙️  OPTIONS:${NC}"
    echo "  --robot HOSTNAME    - Robot hostname (default: $ROBOT_HOST)"
    echo "  --user USERNAME     - SSH username (default: $ROBOT_USER)"
    echo "  --interactive       - Interactive mode (default)"
    echo "  --auto              - Non-interactive mode"
    echo "  --start-from COMP   - Start from specific component"
    echo "  --dry-run           - Show what would be deployed"
    echo
    echo -e "${YELLOW}🔧 EXAMPLES:${NC}"
    echo "  ./deploy_smart.sh                                    # Full interactive deployment"
    echo "  ./deploy_smart.sh --auto                             # Automated deployment"
    echo "  ./deploy_smart.sh --start-from enhanced_navigation   # Start from specific component"
    echo "  ./deploy_smart.sh monitor                            # Real-time monitoring only"
    echo "  ./deploy_smart.sh test enhanced_vision_utils         # Test specific component"
    echo "  ./deploy_smart.sh health                             # Check robot health"
    echo
}

# Parse arguments
COMMAND="deploy"
INTERACTIVE=true
START_FROM=""
DRY_RUN=false

while [[ $# -gt 0 ]]; do
    case $1 in
        --robot)
            ROBOT_HOST="$2"
            shift 2
            ;;
        --user)
            ROBOT_USER="$2"
            shift 2
            ;;
        --interactive)
            INTERACTIVE=true
            shift
            ;;
        --auto)
            INTERACTIVE=false
            shift
            ;;
        --start-from)
            START_FROM="$2"
            shift 2
            ;;
        --dry-run)
            DRY_RUN=true
            shift
            ;;
        deploy|monitor|test|health|status|logs|help)
            COMMAND="$1"
            shift
            ;;
        -h|--help)
            COMMAND="help"
            shift
            ;;
        *)
            if [[ "$COMMAND" == "test" && -z "$START_FROM" ]]; then
                START_FROM="$1"
            fi
            shift
            ;;
    esac
done

# Validate requirements
check_requirements() {
    local missing_requirements=()
    
    # Check Python 3
    if ! command -v python3 &> /dev/null; then
        missing_requirements+=("python3")
    fi
    
    # Check SSH
    if ! command -v ssh &> /dev/null; then
        missing_requirements+=("ssh")
    fi
    
    # Check project structure
    if [[ ! -f "$SCRIPT_DIR/smart_orchestrator.py" ]]; then
        missing_requirements+=("smart_orchestrator.py")
    fi
    
    if [[ ! -f "$SCRIPT_DIR/ultra_monitoring.py" ]]; then
        missing_requirements+=("ultra_monitoring.py")
    fi
    
    if [[ ! -f "$SCRIPT_DIR/component_tester.py" ]]; then
        missing_requirements+=("component_tester.py")
    fi
    
    if [[ ${#missing_requirements[@]} -gt 0 ]]; then
        echo -e "${RED}❌ Missing requirements:${NC}"
        for req in "${missing_requirements[@]}"; do
            echo -e "   ${RED}•${NC} $req"
        done
        echo
        echo -e "${YELLOW}💡 Please install missing requirements or check project structure${NC}"
        exit 1
    fi
    
    # Test robot connectivity
    echo -e "${BLUE}🔍 Testing robot connectivity...${NC}"
    if ssh -o ConnectTimeout=5 -o StrictHostKeyChecking=no "$ROBOT_USER@$ROBOT_HOST" "echo 'Connection test successful'" &>/dev/null; then
        echo -e "${GREEN}✅ Robot connectivity OK${NC}"
    else
        echo -e "${RED}❌ Cannot connect to robot $ROBOT_HOST${NC}"
        echo -e "${YELLOW}💡 Please check:${NC}"
        echo -e "   ${YELLOW}•${NC} Robot is powered on and connected to network"
        echo -e "   ${YELLOW}•${NC} SSH keys are configured"
        echo -e "   ${YELLOW}•${NC} Robot hostname is correct"
        exit 1
    fi
}

# Execute command
execute_command() {
    case "$COMMAND" in
        "deploy")
            show_banner
            echo -e "${GREEN}🚀 Starting Smart Progressive Deployment${NC}"
            echo
            
            if [[ "$DRY_RUN" == "true" ]]; then
                echo -e "${YELLOW}🔍 DRY RUN MODE - Showing deployment plan${NC}"
                echo
                # Show what would be deployed
                python3 "$SCRIPT_DIR/smart_orchestrator.py" \
                    --robot "$ROBOT_HOST" \
                    --user "$ROBOT_USER" \
                    --project-root "$PROJECT_ROOT" \
                    --dry-run
            else
                check_requirements
                
                # Build command
                cmd_args=(
                    "--robot" "$ROBOT_HOST"
                    "--user" "$ROBOT_USER"
                    "--project-root" "$PROJECT_ROOT"
                )
                
                if [[ "$INTERACTIVE" == "false" ]]; then
                    cmd_args+=("--non-interactive")
                fi
                
                if [[ -n "$START_FROM" ]]; then
                    cmd_args+=("--start-from" "$START_FROM")
                fi
                
                # Execute deployment
                python3 "$SCRIPT_DIR/smart_orchestrator.py" "${cmd_args[@]}"
            fi
            ;;
            
        "monitor")
            echo -e "${PURPLE}📊 Starting Real-time Monitoring${NC}"
            echo -e "${CYAN}Robot:${NC} $ROBOT_HOST"
            echo -e "${CYAN}Press Ctrl+C to stop${NC}"
            echo
            
            python3 "$SCRIPT_DIR/ultra_monitoring.py" \
                --robot "$ROBOT_HOST" \
                --user "$ROBOT_USER"
            ;;
            
        "test")
            if [[ -z "$START_FROM" ]]; then
                echo -e "${RED}❌ Please specify component to test${NC}"
                echo -e "${YELLOW}Example: ./deploy_smart.sh test enhanced_vision_utils${NC}"
                exit 1
            fi
            
            echo -e "${BLUE}🧪 Testing Component: $START_FROM${NC}"
            echo -e "${CYAN}Robot:${NC} $ROBOT_HOST"
            echo
            
            python3 "$SCRIPT_DIR/component_tester.py" \
                --robot "$ROBOT_HOST" \
                --user "$ROBOT_USER" \
                --component "$START_FROM" \
                --project-root "$PROJECT_ROOT"
            ;;
            
        "health")
            echo -e "${GREEN}🏥 Robot Health Check${NC}"
            echo -e "${CYAN}Robot:${NC} $ROBOT_HOST"
            echo
            
            python3 "$SCRIPT_DIR/ultra_monitoring.py" \
                --robot "$ROBOT_HOST" \
                --user "$ROBOT_USER" \
                --health-check
            ;;
            
        "status")
            echo -e "${BLUE}📊 Deployment Status${NC}"
            echo
            
            # Check for recent deployment logs
            LOG_DIR="$PROJECT_ROOT/deployment_logs"
            if [[ -d "$LOG_DIR" ]]; then
                echo -e "${CYAN}Recent deployment sessions:${NC}"
                ls -la "$LOG_DIR" | grep "orchestrator_" | tail -5
                echo
                
                # Show latest report if available
                LATEST_REPORT=$(ls -t "$LOG_DIR"/smart_deployment_report_*.json 2>/dev/null | head -1)
                if [[ -n "$LATEST_REPORT" ]]; then
                    echo -e "${CYAN}Latest deployment report:${NC}"
                    python3 -c "
import json
import sys
from datetime import datetime

try:
    with open('$LATEST_REPORT', 'r') as f:
        data = json.load(f)
    
    print(f'Session ID: {data[\"session_id\"]}')
    print(f'Robot: {data[\"robot_host\"]}')
    print(f'Total Steps: {data[\"total_steps\"]}')
    print(f'Successful: {data[\"successful_steps\"]}')
    print(f'Failed: {data[\"failed_steps\"]}')
    print(f'Duration: {data[\"total_duration\"]:.1f}s')
    print(f'Success Rate: {data[\"successful_steps\"]/data[\"total_steps\"]*100:.1f}%' if data[\"total_steps\"] > 0 else 'N/A')
except Exception as e:
    print(f'Error reading report: {e}')
"
                else
                    echo -e "${YELLOW}No deployment reports found${NC}"
                fi
            else
                echo -e "${YELLOW}No deployment logs directory found${NC}"
            fi
            ;;
            
        "logs")
            echo -e "${BLUE}📋 Recent Deployment Logs${NC}"
            echo
            
            LOG_DIR="$PROJECT_ROOT/deployment_logs"
            if [[ -d "$LOG_DIR" ]]; then
                echo -e "${CYAN}Available log files:${NC}"
                ls -la "$LOG_DIR" | tail -10
                echo
                
                # Show latest orchestrator log
                LATEST_LOG=$(ls -t "$LOG_DIR"/orchestrator_*.log 2>/dev/null | head -1)
                if [[ -n "$LATEST_LOG" ]]; then
                    echo -e "${CYAN}Latest orchestrator log (last 20 lines):${NC}"
                    tail -20 "$LATEST_LOG"
                fi
            else
                echo -e "${YELLOW}No logs directory found${NC}"
            fi
            ;;
            
        "help")
            show_help
            ;;
            
        *)
            echo -e "${RED}❌ Unknown command: $COMMAND${NC}"
            echo
            show_help
            exit 1
            ;;
    esac
}

# Main execution
main() {
    # Handle interruption gracefully
    trap 'echo -e "\n${YELLOW}⚠️  Operation interrupted by user${NC}"; exit 130' INT TERM
    
    # Execute the command
    execute_command
    
    echo
    echo -e "${GREEN}✅ Operation completed${NC}"
    echo -e "${CYAN}📁 Logs available in: $PROJECT_ROOT/deployment_logs${NC}"
}

# Run main function
main "$@"
