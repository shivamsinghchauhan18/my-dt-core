#!/bin/bash

# Enhanced Build Script for Advanced Autonomous Duckietown System
# Provides comprehensive logging, monitoring, and multi-stage build support

set -e

# Configuration
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(dirname "$SCRIPT_DIR")"
LOG_DIR="$PROJECT_ROOT/build_logs"
BUILD_LOG="$LOG_DIR/build_$(date +%Y%m%d_%H%M%S).log"

# Build configuration
REPO_NAME="dt-core"
DISTRO="daffy"
ARCH="amd64"
DEFAULT_TARGET="production"

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# Logging functions
log_info() {
    local timestamp=$(date '+%Y-%m-%d %H:%M:%S')
    echo -e "${BLUE}[$timestamp] [INFO]${NC} $1" | tee -a "$BUILD_LOG"
}

log_warn() {
    local timestamp=$(date '+%Y-%m-%d %H:%M:%S')
    echo -e "${YELLOW}[$timestamp] [WARN]${NC} $1" | tee -a "$BUILD_LOG"
}

log_error() {
    local timestamp=$(date '+%Y-%m-%d %H:%M:%S')
    echo -e "${RED}[$timestamp] [ERROR]${NC} $1" | tee -a "$BUILD_LOG"
}

log_success() {
    local timestamp=$(date '+%Y-%m-%d %H:%M:%S')
    echo -e "${GREEN}[$timestamp] [SUCCESS]${NC} $1" | tee -a "$BUILD_LOG"
}

# Setup logging
setup_logging() {
    mkdir -p "$LOG_DIR"
    log_info "Enhanced build script started"
    log_info "Project root: $PROJECT_ROOT"
    log_info "Log directory: $LOG_DIR"
    log_info "Build log: $BUILD_LOG"
}

# System checks
check_prerequisites() {
    log_info "Checking build prerequisites..."
    
    # Check Docker
    if ! command -v docker &> /dev/null; then
        log_error "Docker is not installed or not in PATH"
        exit 1
    fi
    
    # Check Docker daemon
    if ! docker info &> /dev/null; then
        log_error "Docker daemon is not running"
        exit 1
    fi
    
    # Check Docker buildx (for multi-architecture support)
    if ! docker buildx version &> /dev/null; then
        log_warn "Docker buildx not available - multi-architecture builds disabled"
    fi
    
    log_success "Prerequisites check completed"
}

# Dependency validation
validate_dependencies() {
    log_info "Validating dependencies..."
    
    local dep_files=("dependencies-py3.txt" "dependencies-py3.dt.txt" "dependencies-py3.mock.txt")
    
    for dep_file in "${dep_files[@]}"; do
        if [[ -f "$PROJECT_ROOT/$dep_file" ]]; then
            log_info "Found dependency file: $dep_file"
            # Log dependency count
            local count=$(grep -v '^#' "$PROJECT_ROOT/$dep_file" | grep -v '^$' | wc -l)
            log_info "  - $count dependencies in $dep_file"
        else
            log_warn "Dependency file not found: $dep_file"
        fi
    done
    
    # Validate PyTorch and YOLO dependencies
    if grep -q "torch" "$PROJECT_ROOT/dependencies-py3.txt"; then
        log_success "PyTorch dependencies found"
    else
        log_warn "PyTorch dependencies not found in dependencies-py3.txt"
    fi
    
    if grep -q "ultralytics" "$PROJECT_ROOT/dependencies-py3.txt"; then
        log_success "YOLO dependencies found"
    else
        log_warn "YOLO dependencies not found in dependencies-py3.txt"
    fi
}

# Build monitoring
monitor_build() {
    local build_target=$1
    local image_tag=$2
    local start_time=$(date +%s)
    
    log_info "Starting build monitoring for target: $build_target"
    
    # Monitor Docker build progress
    docker build \
        --target "$build_target" \
        --build-arg BUILD_TARGET="$build_target" \
        --build-arg ARCH="$ARCH" \
        --build-arg DISTRO="$DISTRO" \
        --progress=plain \
        -t "$image_tag" \
        "$PROJECT_ROOT" 2>&1 | while IFS= read -r line; do
        echo "$(date '+%Y-%m-%d %H:%M:%S') [BUILD] $line" | tee -a "$BUILD_LOG"
    done
    
    local end_time=$(date +%s)
    local duration=$((end_time - start_time))
    
    if docker images | grep -q "$image_tag"; then
        log_success "Build completed for $build_target in ${duration}s"
        
        # Get image size
        local image_size=$(docker images --format "table {{.Size}}" "$image_tag" | tail -n 1)
        log_info "Image size: $image_size"
        
        # Log image layers
        log_info "Image layers:"
        docker history "$image_tag" --format "table {{.CreatedBy}}\t{{.Size}}" | head -10 | while IFS= read -r line; do
            log_info "  $line"
        done
        
        return 0
    else
        log_error "Build failed for $build_target after ${duration}s"
        return 1
    fi
}

# Build optimization analysis
analyze_build() {
    local image_tag=$1
    
    log_info "Analyzing build optimization for: $image_tag"
    
    # Image size analysis
    local image_size_bytes=$(docker inspect "$image_tag" --format='{{.Size}}')
    local image_size_mb=$((image_size_bytes / 1024 / 1024))
    log_info "Image size: ${image_size_mb}MB"
    
    # Layer analysis
    local layer_count=$(docker history "$image_tag" --quiet | wc -l)
    log_info "Number of layers: $layer_count"
    
    # Dependency analysis
    log_info "Analyzing installed packages..."
    docker run --rm "$image_tag" pip3 list 2>/dev/null | head -20 | while IFS= read -r line; do
        log_info "  $line"
    done
    
    # Performance recommendations
    if [[ $image_size_mb -gt 2000 ]]; then
        log_warn "Image size is large (${image_size_mb}MB) - consider optimization"
    fi
    
    if [[ $layer_count -gt 50 ]]; then
        log_warn "High layer count ($layer_count) - consider layer consolidation"
    fi
}

# Build stage execution
build_stage() {
    local target=$1
    local tag_suffix=$2
    
    local image_tag="duckietown/${REPO_NAME}:${DISTRO}-${tag_suffix}-${ARCH}"
    
    log_info "Building stage: $target"
    log_info "Image tag: $image_tag"
    
    if monitor_build "$target" "$image_tag"; then
        analyze_build "$image_tag"
        log_success "Stage $target completed successfully"
        return 0
    else
        log_error "Stage $target failed"
        return 1
    fi
}

# Main build function
build_enhanced() {
    local targets=("$@")
    
    if [[ ${#targets[@]} -eq 0 ]]; then
        targets=("$DEFAULT_TARGET")
    fi
    
    log_info "Building targets: ${targets[*]}"
    
    local failed_builds=()
    
    for target in "${targets[@]}"; do
        case $target in
            "development"|"dev")
                if build_stage "development" "devel"; then
                    log_success "Development build completed"
                else
                    failed_builds+=("development")
                fi
                ;;
            "testing"|"test")
                if build_stage "testing" "test"; then
                    log_success "Testing build completed"
                else
                    failed_builds+=("testing")
                fi
                ;;
            "production"|"prod")
                if build_stage "production" ""; then
                    log_success "Production build completed"
                else
                    failed_builds+=("production")
                fi
                ;;
            "all")
                build_enhanced "development" "testing" "production"
                return $?
                ;;
            *)
                log_error "Unknown build target: $target"
                failed_builds+=("$target")
                ;;
        esac
    done
    
    # Summary
    if [[ ${#failed_builds[@]} -eq 0 ]]; then
        log_success "All builds completed successfully"
        return 0
    else
        log_error "Failed builds: ${failed_builds[*]}"
        return 1
    fi
}

# Performance monitoring
monitor_performance() {
    log_info "Monitoring system performance during build..."
    
    # CPU and memory monitoring
    local monitor_pid=$$
    (
        while kill -0 $monitor_pid 2>/dev/null; do
            local cpu_usage=$(top -l 1 -n 0 | grep "CPU usage" | awk '{print $3}' | sed 's/%//' 2>/dev/null || echo "N/A")
            local memory_usage=$(vm_stat | grep "Pages active" | awk '{print $3}' | sed 's/\.//' 2>/dev/null || echo "N/A")
            log_info "System performance - CPU: ${cpu_usage}%, Memory: ${memory_usage} pages"
            sleep 30
        done
    ) &
    local perf_monitor_pid=$!
    
    # Cleanup function
    cleanup_monitoring() {
        kill $perf_monitor_pid 2>/dev/null || true
        log_info "Performance monitoring stopped"
    }
    
    trap cleanup_monitoring EXIT
}

# Usage information
show_usage() {
    echo "Enhanced Build Script for Advanced Autonomous Duckietown System"
    echo
    echo "Usage: $0 [OPTIONS] [TARGETS...]"
    echo
    echo "Targets:"
    echo "  development, dev    Build development image with debugging tools"
    echo "  testing, test       Build testing image with test frameworks"
    echo "  production, prod    Build production image (optimized)"
    echo "  all                 Build all targets"
    echo
    echo "Options:"
    echo "  -h, --help         Show this help message"
    echo "  -v, --verbose      Enable verbose logging"
    echo "  -m, --monitor      Enable performance monitoring"
    echo "  --arch ARCH        Target architecture (default: $ARCH)"
    echo "  --distro DISTRO    Target distribution (default: $DISTRO)"
    echo
    echo "Examples:"
    echo "  $0 development                    # Build development image"
    echo "  $0 all                           # Build all images"
    echo "  $0 --monitor production          # Build production with monitoring"
    echo "  $0 --arch arm64 development      # Build for ARM64 architecture"
}

# Main execution
main() {
    local verbose=false
    local enable_monitoring=false
    local targets=()
    
    # Parse arguments
    while [[ $# -gt 0 ]]; do
        case $1 in
            -h|--help)
                show_usage
                exit 0
                ;;
            -v|--verbose)
                verbose=true
                shift
                ;;
            -m|--monitor)
                enable_monitoring=true
                shift
                ;;
            --arch)
                ARCH="$2"
                shift 2
                ;;
            --distro)
                DISTRO="$2"
                shift 2
                ;;
            -*)
                log_error "Unknown option: $1"
                show_usage
                exit 1
                ;;
            *)
                targets+=("$1")
                shift
                ;;
        esac
    done
    
    # Setup
    setup_logging
    check_prerequisites
    validate_dependencies
    
    # Enable performance monitoring if requested
    if [[ $enable_monitoring == true ]]; then
        monitor_performance
    fi
    
    # Execute build
    if build_enhanced "${targets[@]}"; then
        log_success "Enhanced build script completed successfully"
        exit 0
    else
        log_error "Enhanced build script failed"
        exit 1
    fi
}

# Execute main function
main "$@"