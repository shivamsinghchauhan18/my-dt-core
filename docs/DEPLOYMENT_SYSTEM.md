# Enhanced Deployment System

This document describes the enhanced deployment system for the Advanced Autonomous Duckietown System, featuring SSH-based file transfer, robot-specific configuration management, comprehensive health checks, and automated deployment verification.

## Overview

The enhanced deployment system provides:

- **SSH-based file transfer** with rsync and scp for efficient code deployment
- **Robot-specific configuration management** with YAML-based configuration files
- **Comprehensive health checks** and deployment verification
- **Automated deployment workflows** with error handling and recovery
- **Performance monitoring** and deployment analytics
- **Integration with existing system monitoring** and logging infrastructure

## Quick Start

### Basic Deployment Commands

```bash
# Deploy everything to a robot
./scripts/enhanced_deploy.sh duckiebot01

# Deploy only source code
./scripts/enhanced_deploy.sh duckiebot01 source

# Deploy only Docker image
./scripts/enhanced_deploy.sh duckiebot01 image

# Deploy only configuration
./scripts/enhanced_deploy.sh duckiebot01 config

# Dry run to see what would be deployed
./scripts/enhanced_deploy.sh --dry-run duckiebot01
```

### Robot Configuration Setup

1. **Create robot configuration**:
   ```bash
   cp robot_configs/example.yaml robot_configs/duckiebot01.yaml
   ```

2. **Edit robot configuration**:
   ```yaml
   robot:
     host: "duckiebot01.local"
     user: "duckie"
     port: 22
     architecture: "arm64"
     distro: "daffy"
     workspace: "/code/catkin_ws/src/dt-core"
   ```

3. **Deploy to robot**:
   ```bash
   ./scripts/enhanced_deploy.sh duckiebot01
   ```

## Robot Configuration Management

### Configuration File Structure

Robot configurations are stored in `robot_configs/ROBOT_NAME.yaml`:

```yaml
robot:
  # Network configuration
  host: "duckiebot01.local"  # Robot hostname or IP address
  user: "duckie"             # SSH username
  port: 22                   # SSH port
  
  # System configuration
  architecture: "arm64"      # Robot architecture (arm64, amd64)
  distro: "daffy"           # Duckietown distribution
  workspace: "/code/catkin_ws/src/dt-core"  # Workspace path on robot
  
  # Calibration configuration
  calibration:
    directory: "/data/config/calibrations"
    camera_intrinsic: "camera_intrinsic.yaml"
    camera_extrinsic: "camera_extrinsic.yaml"
    kinematics: "kinematics.yaml"
  
  # Network configuration
  network:
    wifi_ssid: "Duckietown"
    wifi_password: ""  # Leave empty if using key-based auth
  
  # Hardware configuration
  hardware:
    camera_topic: "/camera/image_raw"
    motor_topic: "/wheels_driver_node/wheels_cmd"
    led_topic: "/led_emitter_node/led_pattern"
    
  # Advanced configuration
  advanced:
    ros_master_uri: "http://localhost:11311"
    ros_hostname: "localhost"
    log_level: "INFO"
    
  # Deployment preferences
  deployment:
    auto_start: true
    health_check_interval: 30
    restart_policy: "unless-stopped"
```

### Configuration Validation

The deployment system automatically validates robot configurations:

- **Network connectivity**: Tests SSH connection to robot
- **Configuration completeness**: Validates required fields
- **System compatibility**: Checks architecture and distribution compatibility
- **Workspace accessibility**: Verifies workspace directory permissions

## SSH-based File Transfer

### Prerequisites

1. **SSH Key Setup**:
   ```bash
   # Generate SSH key if not exists
   ssh-keygen -t rsa -b 4096 -C "your_email@example.com"
   
   # Copy public key to robot
   ssh-copy-id duckie@duckiebot01.local
   ```

2. **Network Connectivity**:
   - Robot must be connected to network
   - SSH service must be running on robot
   - Firewall must allow SSH connections

### File Transfer Operations

The deployment system uses multiple transfer methods:

#### Source Code Synchronization
```bash
# Rsync for efficient directory synchronization
rsync -avz --progress --delete \
    -e "ssh -p 22" \
    ./packages/ \
    duckie@duckiebot01.local:/code/catkin_ws/src/dt-core/packages/
```

#### Configuration File Transfer
```bash
# SCP for individual file transfer
scp -P 22 configurations.yaml \
    duckie@duckiebot01.local:/code/catkin_ws/src/dt-core/
```

#### Docker Image Deployment
```bash
# Save and transfer Docker images
docker save duckietown/dt-core:daffy-arm64 > dt-core.tar
scp -P 22 dt-core.tar duckie@duckiebot01.local:/tmp/
ssh -p 22 duckie@duckiebot01.local "docker load < /tmp/dt-core.tar"
```

### Transfer Optimization

- **Incremental transfers**: Only changed files are transferred
- **Compression**: Data is compressed during transfer
- **Progress monitoring**: Real-time transfer progress display
- **Error recovery**: Automatic retry on transfer failures
- **Bandwidth optimization**: Adaptive transfer rates

## Health Checks and Verification

### System Health Checks

The deployment system performs comprehensive health checks:

#### Workspace Verification
```bash
# Check workspace structure
test -d /code/catkin_ws/src/dt-core
test -d /code/catkin_ws/src/dt-core/packages
test -f /code/catkin_ws/src/dt-core/configurations.yaml
```

#### Docker Environment
```bash
# Check Docker availability
docker --version
docker images | grep duckietown/dt-core
systemctl is-active docker
```

#### ROS Environment
```bash
# Check ROS installation
source /opt/ros/noetic/setup.bash
roscore --version
rostopic list
```

#### System Resources
```bash
# Monitor system resources
free -h                    # Memory usage
df -h                      # Disk usage
uptime                     # System load
cat /proc/cpuinfo         # CPU information
```

### Deployment Verification

Post-deployment verification includes:

1. **File Integrity**: Verify all files transferred correctly
2. **Configuration Validation**: Check configuration file syntax
3. **Dependency Resolution**: Verify all dependencies are available
4. **Service Status**: Check that required services are running
5. **Functional Testing**: Basic functionality tests

### Health Check Results

```bash
[2024-01-15 14:30:15] [DEPLOY] Performing deployment health checks...
[2024-01-15 14:30:16] [DEPLOY]   Workspace directory: 25 items
[2024-01-15 14:30:16] [DEPLOY]   Packages directory: 23 items
[2024-01-15 14:30:16] [DEPLOY]   Scripts directory: 5 items
[2024-01-15 14:30:17] [DEPLOY]   Docker image available on robot
[2024-01-15 14:30:18] [DEPLOY]   ROS Noetic available
[2024-01-15 14:30:19] [DEPLOY]   CPU Load: 0.45, 0.52, 0.48
[2024-01-15 14:30:19] [DEPLOY]   Memory Usage: 45.2%
[2024-01-15 14:30:19] [DEPLOY]   Disk Usage: 60%
[2024-01-15 14:30:20] [SUCCESS] Health checks completed
```

## Enhanced Deployment Script

### Script Features

The `scripts/enhanced_deploy.sh` script provides:

- **Comprehensive logging** with timestamped output
- **Robot configuration management** with YAML parsing
- **SSH connectivity testing** with timeout handling
- **File transfer monitoring** with progress tracking
- **Health check automation** with detailed reporting
- **Deployment verification** with error detection
- **Performance monitoring** with timing metrics

### Usage Examples

```bash
# Basic deployment
./scripts/enhanced_deploy.sh duckiebot01

# Verbose deployment with monitoring
./scripts/enhanced_deploy.sh --verbose duckiebot01

# Deployment with custom timeout
./scripts/enhanced_deploy.sh --timeout 600 duckiebot01

# Deployment with retry configuration
./scripts/enhanced_deploy.sh --retry 5 duckiebot01

# Dry run to preview deployment
./scripts/enhanced_deploy.sh --dry-run duckiebot01
```

### Deployment Options

| Option | Description |
|--------|-------------|
| `source` | Deploy source code only |
| `image` | Deploy Docker image only |
| `config` | Deploy configuration only |
| `all` | Deploy everything (default) |

### Script Options

| Option | Description |
|--------|-------------|
| `--help` | Show help message |
| `--verbose` | Enable verbose logging |
| `--dry-run` | Preview deployment without executing |
| `--timeout SECONDS` | Set deployment timeout |
| `--retry COUNT` | Set retry count for failed operations |

## Integration with System Monitoring

### Logging Integration

The deployment system integrates with existing logging infrastructure:

```bash
# Deployment logs are saved to
deployment_logs/deploy_YYYYMMDD_HHMMSS.log

# Log format includes timestamps and severity levels
[2024-01-15 14:30:15] [DEPLOY] Starting deployment to robot: duckiebot01
[2024-01-15 14:30:16] [SUCCESS] SSH connection successful
[2024-01-15 14:30:17] [PROGRESS] Syncing packages directory...
[2024-01-15 14:30:25] [SUCCESS] Source code sync completed
```

### Performance Monitoring

Deployment performance is monitored and logged:

```bash
# Performance metrics
[2024-01-15 14:30:30] [DEPLOY] Deployment Performance Metrics:
[2024-01-15 14:30:30] [DEPLOY]   SSH Connection Time: 2.3s
[2024-01-15 14:30:30] [DEPLOY]   File Transfer Time: 45.7s
[2024-01-15 14:30:30] [DEPLOY]   Docker Deployment Time: 120.4s
[2024-01-15 14:30:30] [DEPLOY]   Configuration Deployment Time: 3.2s
[2024-01-15 14:30:30] [DEPLOY]   Health Check Time: 8.9s
[2024-01-15 14:30:30] [DEPLOY]   Total Deployment Time: 180.5s
```

### System Integration

The deployment system integrates with:

- **Enhanced configurations.yaml**: Robot-specific settings
- **Build system**: Automatic image deployment
- **Monitoring system**: Health check integration
- **Logging system**: Centralized log management

## Advanced Configuration Management

### Global Configuration

The enhanced `configurations.yaml` includes deployment settings:

```yaml
configurations:
  deployment:
    default_user: "duckie"
    default_port: 22
    timeout: 300
    retry_count: 3
    health_check_timeout: 60
    
  autonomous_system:
    lane_following:
      lateral_deviation_threshold: 0.05
      heading_error_threshold: 0.1
      adaptive_threshold_enabled: true
      
    object_detection:
      confidence_threshold: 0.6
      nms_threshold: 0.4
      max_detections: 10
      
    safety:
      emergency_stop_enabled: true
      collision_avoidance_enabled: true
      hardware_monitoring_enabled: true
```

### Robot-Specific Overrides

Robot configurations can override global settings:

```yaml
# robot_configs/duckiebot01.yaml
robot:
  host: "duckiebot01.local"
  
  # Override global autonomous system settings
  autonomous_overrides:
    lane_following:
      lateral_deviation_threshold: 0.03  # Tighter tolerance
    object_detection:
      confidence_threshold: 0.7          # Higher confidence
```

## Error Handling and Recovery

### Error Scenarios

The deployment system handles various error scenarios:

1. **SSH Connection Failures**:
   - Network connectivity issues
   - Authentication failures
   - Timeout errors

2. **File Transfer Failures**:
   - Insufficient disk space
   - Permission errors
   - Network interruptions

3. **Configuration Errors**:
   - Invalid YAML syntax
   - Missing required fields
   - Incompatible settings

4. **System Errors**:
   - Docker service not running
   - ROS environment issues
   - Resource constraints

### Recovery Strategies

```bash
# Automatic retry with exponential backoff
[2024-01-15 14:30:25] [WARN] File transfer failed, retrying in 5 seconds...
[2024-01-15 14:30:30] [WARN] File transfer failed, retrying in 10 seconds...
[2024-01-15 14:30:40] [SUCCESS] File transfer completed on retry 2

# Partial deployment recovery
[2024-01-15 14:30:45] [ERROR] Docker image deployment failed
[2024-01-15 14:30:45] [DEPLOY] Continuing with source code deployment...
[2024-01-15 14:30:50] [SUCCESS] Source code deployment completed
[2024-01-15 14:30:50] [WARN] Manual Docker image deployment required
```

## Troubleshooting

### Common Issues

**SSH Connection Failures**:
```bash
# Test SSH connectivity
ssh -v duckie@duckiebot01.local

# Check SSH key authentication
ssh-add -l

# Verify robot network connectivity
ping duckiebot01.local
```

**File Transfer Issues**:
```bash
# Check disk space on robot
ssh duckie@duckiebot01.local "df -h"

# Verify permissions
ssh duckie@duckiebot01.local "ls -la /code/catkin_ws/src/"

# Test manual file transfer
scp test.txt duckie@duckiebot01.local:/tmp/
```

**Configuration Problems**:
```bash
# Validate YAML syntax
python3 -c "import yaml; yaml.safe_load(open('robot_configs/duckiebot01.yaml'))"

# Check configuration completeness
./scripts/enhanced_deploy.sh --dry-run duckiebot01
```

### Deployment Logs

Deployment logs provide detailed troubleshooting information:

```bash
# View latest deployment log
tail -f deployment_logs/deploy_$(date +%Y%m%d)*.log

# Search for errors
grep ERROR deployment_logs/deploy_*.log

# Analyze deployment performance
grep "Performance Metrics" deployment_logs/deploy_*.log
```

### Debug Mode

Enable verbose logging for detailed troubleshooting:

```bash
# Verbose deployment
./scripts/enhanced_deploy.sh --verbose duckiebot01

# Debug output includes:
[2024-01-15 14:30:15] [DEBUG] Loading robot configuration from: robot_configs/duckiebot01.yaml
[2024-01-15 14:30:16] [DEBUG] Parsed configuration: host=duckiebot01.local, user=duckie
[2024-01-15 14:30:17] [DEBUG] Testing SSH connection with timeout=10s
[2024-01-15 14:30:19] [DEBUG] SSH connection successful, latency=2.3s
```

## Best Practices

### Deployment Workflow

1. **Pre-deployment**:
   - Verify robot configuration
   - Test SSH connectivity
   - Check local build status
   - Review deployment plan

2. **Deployment**:
   - Use dry-run for validation
   - Monitor deployment progress
   - Verify health checks
   - Confirm deployment success

3. **Post-deployment**:
   - Test robot functionality
   - Monitor system performance
   - Review deployment logs
   - Update documentation

### Security Considerations

1. **SSH Key Management**:
   - Use strong SSH keys (RSA 4096-bit or Ed25519)
   - Regularly rotate SSH keys
   - Limit SSH key access
   - Monitor SSH access logs

2. **Network Security**:
   - Use secure networks for deployment
   - Implement firewall rules
   - Monitor network traffic
   - Use VPN when necessary

3. **Configuration Security**:
   - Protect configuration files
   - Avoid storing passwords in configs
   - Use environment variables for secrets
   - Implement access controls

### Performance Optimization

1. **Transfer Optimization**:
   - Use incremental transfers
   - Enable compression
   - Optimize network settings
   - Monitor bandwidth usage

2. **Resource Management**:
   - Monitor robot resources
   - Optimize deployment timing
   - Use efficient file formats
   - Implement cleanup procedures

3. **Automation**:
   - Automate routine deployments
   - Implement CI/CD integration
   - Use deployment scheduling
   - Monitor deployment metrics

This enhanced deployment system provides a robust foundation for deploying the Advanced Autonomous Duckietown System to robots while maintaining security, reliability, and performance across the deployment lifecycle.