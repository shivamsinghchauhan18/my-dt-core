#!/bin/bash
"""
System Validation Runner Script

Wrapper script to run comprehensive system validation with proper environment setup.
This script ensures all dependencies are available and runs the validation with
comprehensive logging and monitoring.
"""

set -e  # Exit on any error

# Script configuration
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"
VALIDATION_SCRIPT="$PROJECT_ROOT/src/packages/easy_regression/src/run_comprehensive_validation.py"

# Default configuration
DEFAULT_OUTPUT_DIR="$PROJECT_ROOT/validation_results"
DEFAULT_LOG_LEVEL="INFO"

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# Logging functions
log_info() {
    echo -e "${BLUE}[INFO]${NC} $1"
}

log_success() {
    echo -e "${GREEN}[SUCCESS]${NC} $1"
}

log_warning() {
    echo -e "${YELLOW}[WARNING]${NC} $1"
}

log_error() {
    echo -e "${RED}[ERROR]${NC} $1"
}

# Help function
show_help() {
    cat << EOF
System Validation Runner for Advanced Autonomous Duckietown System

USAGE:
    $0 [OPTIONS]

OPTIONS:
    -h, --help              Show this help message
    -o, --output-dir DIR    Output directory for validation results (default: validation_results)
    -l, --log-level LEVEL   Logging level: DEBUG, INFO, WARNING, ERROR (default: INFO)
    -f, --functional-only   Run only functional requirements validation
    -p, --performance-only  Run only performance requirements validation
    -s, --safety-only       Run only safety requirements validation
    -i, --integration-only  Run only integration requirements validation
    -c, --continue-on-failure  Continue validation even if individual tests fail
    -t, --timeout SECONDS  Validation timeout in seconds (default: 3600)
    --log-file FILE         Log to file in addition to console
    --export-format FORMAT  Export format: json, markdown, both (default: both)

EXAMPLES:
    # Run full validation with default settings
    $0

    # Run validation with custom output directory and debug logging
    $0 --output-dir /tmp/validation --log-level DEBUG

    # Run only functional requirements validation
    $0 --functional-only

    # Run validation with timeout and continue on failure
    $0 --timeout 1800 --continue-on-failure

    # Run validation and save logs to file
    $0 --log-file validation.log

EOF
}

# Check prerequisites
check_prerequisites() {
    log_info "Checking prerequisites..."
    
    # Check if we're in the correct directory
    if [[ ! -d "$PROJECT_ROOT/packages" ]]; then
        log_error "Not in dt-core root directory. Please run from dt-core root."
        exit 1
    fi
    
    # Check Python availability
    if ! command -v python3 &> /dev/null; then
        log_error "Python 3 is required but not installed."
        exit 1
    fi
    
    # Check validation script exists
    if [[ ! -f "$VALIDATION_SCRIPT" ]]; then
        log_error "Validation script not found: $VALIDATION_SCRIPT"
        exit 1
    fi
    
    # Check Python dependencies
    log_info "Checking Python dependencies..."
    python3 -c "import numpy, yaml, psutil" 2>/dev/null || {
        log_warning "Some Python dependencies may be missing. Validation may fail."
        log_warning "Please ensure numpy, pyyaml, and psutil are installed."
    }
    
    log_success "Prerequisites check completed"
}

# Setup environment
setup_environment() {
    log_info "Setting up validation environment..."
    
    # Change to project root
    cd "$PROJECT_ROOT"
    
    # Create output directory if it doesn't exist
    mkdir -p "$OUTPUT_DIR"
    
    # Set Python path
    export PYTHONPATH="$PROJECT_ROOT/src/packages/easy_regression/src:$PYTHONPATH"
    
    log_success "Environment setup completed"
}

# Run validation
run_validation() {
    log_info "Starting comprehensive system validation..."
    log_info "Output directory: $OUTPUT_DIR"
    log_info "Log level: $LOG_LEVEL"
    
    # Build command
    local cmd="python3 $VALIDATION_SCRIPT --output-dir $OUTPUT_DIR --log-level $LOG_LEVEL"
    
    # Add optional arguments
    [[ -n "$LOG_FILE" ]] && cmd="$cmd --log-file $LOG_FILE"
    [[ -n "$TIMEOUT" ]] && cmd="$cmd --timeout $TIMEOUT"
    [[ -n "$EXPORT_FORMAT" ]] && cmd="$cmd --export-format $EXPORT_FORMAT"
    
    # Add validation type flags
    [[ "$FUNCTIONAL_ONLY" == "true" ]] && cmd="$cmd --functional-only"
    [[ "$PERFORMANCE_ONLY" == "true" ]] && cmd="$cmd --performance-only"
    [[ "$SAFETY_ONLY" == "true" ]] && cmd="$cmd --safety-only"
    [[ "$INTEGRATION_ONLY" == "true" ]] && cmd="$cmd --integration-only"
    [[ "$CONTINUE_ON_FAILURE" == "true" ]] && cmd="$cmd --continue-on-failure"
    
    log_info "Executing: $cmd"
    
    # Run validation
    if eval "$cmd"; then
        log_success "Validation completed successfully!"
        return 0
    else
        local exit_code=$?
        log_error "Validation failed with exit code: $exit_code"
        return $exit_code
    fi
}

# Cleanup function
cleanup() {
    log_info "Cleaning up..."
    # Add any cleanup tasks here
}

# Main function
main() {
    # Set default values
    OUTPUT_DIR="$DEFAULT_OUTPUT_DIR"
    LOG_LEVEL="$DEFAULT_LOG_LEVEL"
    LOG_FILE=""
    TIMEOUT=""
    EXPORT_FORMAT=""
    FUNCTIONAL_ONLY="false"
    PERFORMANCE_ONLY="false"
    SAFETY_ONLY="false"
    INTEGRATION_ONLY="false"
    CONTINUE_ON_FAILURE="false"
    
    # Parse command line arguments
    while [[ $# -gt 0 ]]; do
        case $1 in
            -h|--help)
                show_help
                exit 0
                ;;
            -o|--output-dir)
                OUTPUT_DIR="$2"
                shift 2
                ;;
            -l|--log-level)
                LOG_LEVEL="$2"
                shift 2
                ;;
            --log-file)
                LOG_FILE="$2"
                shift 2
                ;;
            -t|--timeout)
                TIMEOUT="$2"
                shift 2
                ;;
            --export-format)
                EXPORT_FORMAT="$2"
                shift 2
                ;;
            -f|--functional-only)
                FUNCTIONAL_ONLY="true"
                shift
                ;;
            -p|--performance-only)
                PERFORMANCE_ONLY="true"
                shift
                ;;
            -s|--safety-only)
                SAFETY_ONLY="true"
                shift
                ;;
            -i|--integration-only)
                INTEGRATION_ONLY="true"
                shift
                ;;
            -c|--continue-on-failure)
                CONTINUE_ON_FAILURE="true"
                shift
                ;;
            *)
                log_error "Unknown option: $1"
                show_help
                exit 1
                ;;
        esac
    done
    
    # Validate log level
    case $LOG_LEVEL in
        DEBUG|INFO|WARNING|ERROR)
            ;;
        *)
            log_error "Invalid log level: $LOG_LEVEL"
            log_error "Valid levels: DEBUG, INFO, WARNING, ERROR"
            exit 1
            ;;
    esac
    
    # Set up trap for cleanup
    trap cleanup EXIT
    
    # Run validation workflow
    log_info "="*60
    log_info "ADVANCED AUTONOMOUS DUCKIETOWN SYSTEM VALIDATION"
    log_info "="*60
    
    check_prerequisites
    setup_environment
    
    # Run validation and capture exit code
    if run_validation; then
        log_success "="*60
        log_success "VALIDATION COMPLETED SUCCESSFULLY!"
        log_success "System is ready for deployment."
        log_success "="*60
        exit 0
    else
        exit_code=$?
        log_error "="*60
        log_error "VALIDATION FAILED!"
        log_error "Please check the logs and address any issues."
        log_error "="*60
        exit $exit_code
    fi
}

# Run main function with all arguments
main "$@"