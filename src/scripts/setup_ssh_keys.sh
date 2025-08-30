#!/bin/bash

# SSH Key Setup Script for Duckietown Robot
# Sets up passwordless SSH authentication

ROBOT_NAME="pinkduckie"
ROBOT_HOST="${ROBOT_NAME}.local"
ROBOT_USER="duckie"
ROBOT_PASSWORD="quackquack"

# Colors
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
RED='\033[0;31m'
NC='\033[0m'

log_info() { echo -e "${BLUE}[INFO]${NC} $1"; }
log_success() { echo -e "${GREEN}[SUCCESS]${NC} $1"; }
log_warning() { echo -e "${YELLOW}[WARNING]${NC} $1"; }
log_error() { echo -e "${RED}[ERROR]${NC} $1"; }

echo "=========================================="
echo "SSH Key Setup for Duckietown Robot"
echo "Robot: $ROBOT_NAME"
echo "=========================================="
echo

# Check if SSH key exists
if [[ ! -f ~/.ssh/id_rsa ]]; then
    log_info "No SSH key found. Generating new SSH key..."
    ssh-keygen -t rsa -b 4096 -f ~/.ssh/id_rsa -N "" -C "$(whoami)@$(hostname)"
    log_success "SSH key generated"
else
    log_info "SSH key already exists"
fi

# Check if sshpass is available for password authentication
if ! command -v sshpass &> /dev/null; then
    log_warning "sshpass not found. Installing via Homebrew..."
    if command -v brew &> /dev/null; then
        brew install hudochenkov/sshpass/sshpass
    else
        log_error "Homebrew not found. Please install sshpass manually:"
        log_error "  brew install hudochenkov/sshpass/sshpass"
        exit 1
    fi
fi

# Test connectivity first
log_info "Testing connectivity to robot..."
if ! ping -c 1 "$ROBOT_HOST" &>/dev/null; then
    log_error "Cannot reach robot at $ROBOT_HOST"
    log_error "Please ensure:"
    log_error "  1. Robot is powered on"
    log_error "  2. Robot is connected to the same network"
    log_error "  3. Robot hostname is correct"
    exit 1
fi

log_success "Robot is reachable at $ROBOT_HOST"

# Copy SSH key to robot
log_info "Copying SSH key to robot (will prompt for password: quackquack)..."
if sshpass -p "$ROBOT_PASSWORD" ssh-copy-id -o StrictHostKeyChecking=no "$ROBOT_USER@$ROBOT_HOST"; then
    log_success "SSH key copied successfully"
else
    log_error "Failed to copy SSH key"
    log_error "Please ensure the robot password is correct: $ROBOT_PASSWORD"
    exit 1
fi

# Test passwordless SSH
log_info "Testing passwordless SSH connection..."
if ssh -o ConnectTimeout=5 -o BatchMode=yes "$ROBOT_USER@$ROBOT_HOST" "echo 'SSH key authentication successful'"; then
    log_success "Passwordless SSH authentication working!"
else
    log_error "SSH key authentication failed"
    exit 1
fi

echo
log_success "SSH key setup completed successfully!"
log_info "You can now connect to your robot without a password:"
log_info "  ssh $ROBOT_USER@$ROBOT_HOST"
echo