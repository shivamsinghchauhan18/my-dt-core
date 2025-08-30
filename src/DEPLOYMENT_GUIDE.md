# Advanced Autonomous Duckietown System - Deployment Guide

## Overview

This guide provides step-by-step instructions for deploying the Advanced Autonomous Duckietown System to your DB21J robot (pinkduckie) from a macOS M2 development environment.

## Prerequisites

### Development Environment (macOS M2)
- macOS with Apple Silicon (M1/M2)
- Docker Desktop for Mac (with buildx support)
- SSH client and rsync
- Network connectivity to your Duckietown robot

### Robot Requirements (DB21J - pinkduckie)
- Duckietown DB21J robot with Raspberry Pi 4
- Ubuntu 20.04 with ROS Noetic
- Docker installed and running
- SSH access configured
- Robot connected to same network as development machine

## System Architecture

The enhanced system includes:

1. **Adaptive Lane Detection** - Dynamic threshold adjustment based on lighting
2. **Temporal Consistency Filtering** - Stable detection across frames
3. **Polynomial Curve Fitting** - Advanced trajectory prediction
4. **Enhanced AprilTag Detection** - Multi-resolution with distance estimation
5. **YOLOv5 Object Detection** - Real-time obstacle detection
6. **Predictive Lane Control** - Model Predictive Control (MPC)
7. **Safety Monitoring** - Multi-layer safety system
8. **Intelligent Coordination** - Behavior arbitration and conflict resolution

## Quick Start Deployment

### Step 1: Pre-deployment Check

Run the pre-deployment validation to ensure your system is ready:

```bash
./src/scripts/pre_deployment_check.sh
```

This will verify:
- Development environment setup
- Project structure and dependencies
- Network connectivity to robot
- Robot system status

### Step 2: Deploy to Robot

Execute the automated deployment script:

```bash
./src/scripts/deploy_pinkduckie.sh
```

This script will:
1. Build ARM64 Docker image
2. Validate system components
3. Deploy code and configuration to robot
4. Start the enhanced autonomous system
5. Monitor initial performance

### Step 3: Verify Deployment

After deployment, verify the system is running:

```bash
# SSH into the robot
ssh duckie@pinkduckie.local

# Check ROS topics
rostopic list | grep pinkduckie

# Check system status
docker ps | grep dt-core

# Monitor system performance
htop
```

## Manual Deployment Steps

If you prefer manual deployment or need to troubleshoot:

### 1. Build System

```bash
# Build for ARM64 architecture
make buildx-setup
make buildx-prod

# Or build locally for testing
make build-prod ARCH=arm64
```

### 2. Validate System

```bash
# Run comprehensive validation
./src/scripts/run_system_validation.sh --functional-only

# Check build system
make build-validate
```

### 3. Deploy to Robot

```bash
# Use enhanced deployment script
./src/scripts/enhanced_deploy.sh pinkduckie all

# Or deploy components individually
./src/scripts/enhanced_deploy.sh pinkduckie source
./src/scripts/enhanced_deploy.sh pinkduckie image
./src/scripts/enhanced_deploy.sh pinkduckie config
```

### 4. Start System on Robot

```bash
# SSH to robot
ssh duckie@pinkduckie.local

# Start enhanced autonomous system
roslaunch duckietown_demos enhanced_autonomous_pinkduckie.launch veh:=pinkduckie
```

## Configuration

### Robot-Specific Configuration

The system uses `src/robot_configs/pinkduckie.yaml` for robot-specific settings:

```yaml
robot:
  host: "pinkduckie.local"
  architecture: "arm64"
  advanced:
    lane_following:
      adaptive_threshold_enabled: true
      temporal_filtering_enabled: true
    object_detection:
      enabled: true
      confidence_threshold: 0.5
    safety:
      emergency_stop_enabled: true
      cpu_warning_threshold: 75.0
```

### Performance Tuning for DB21J

The system is optimized for DB21J hardware:

- **Target FPS**: 15 (reduced for ARM64 performance)
- **Memory Limit**: 1536 MB
- **CPU Optimization**: Enabled
- **Processing Latency**: <300ms

### Calibration

Ensure your robot is properly calibrated:

```bash
# Check calibration files
ls /data/config/calibrations/camera_intrinsic/

# Run calibration if needed
dts calibrate_intrinsics pinkduckie
dts calibrate_extrinsics pinkduckie
```

## Monitoring and Debugging

### System Monitoring

Monitor system performance in real-time:

```bash
# SSH to robot
ssh duckie@pinkduckie.local

# Monitor ROS topics
rostopic hz /pinkduckie/camera_node/image/compressed
rostopic echo /pinkduckie/fsm_node/mode

# Monitor system resources
htop
docker stats

# Check temperature
vcgencmd measure_temp
```

### Log Analysis

View system logs:

```bash
# Docker container logs
docker logs $(docker ps -q --filter ancestor=duckietown/dt-core:daffy-arm64)

# ROS logs
tail -f ~/.ros/log/latest/rosout.log

# System logs
journalctl -u docker -f
```

### Debug Topics

The system provides debug visualization:

```bash
# View debug images (requires X11 forwarding or VNC)
rosrun image_view image_view image:=/pinkduckie/line_detector_node/debug/segments/compressed

# Monitor detection performance
rostopic echo /pinkduckie/line_detector_node/segment_list
```

## Troubleshooting

### Common Issues

1. **SSH Connection Failed**
   ```bash
   # Check network connectivity
   ping pinkduckie.local
   
   # Verify SSH keys
   ssh-add -l
   
   # Test SSH connection
   ssh -v duckie@pinkduckie.local
   ```

2. **Docker Build Failed**
   ```bash
   # Check Docker daemon
   docker info
   
   # Clean build cache
   docker system prune -f
   
   # Rebuild with verbose output
   docker build --no-cache --progress=plain .
   ```

3. **ROS Topics Not Available**
   ```bash
   # Check ROS master
   rostopic list
   
   # Restart ROS nodes
   rosnode kill -a
   roslaunch duckietown_demos enhanced_autonomous_pinkduckie.launch veh:=pinkduckie
   ```

4. **High CPU Usage**
   ```bash
   # Monitor CPU usage
   top -p $(pgrep -f ros)
   
   # Reduce processing load
   rosparam set /pinkduckie/line_detector_node/target_fps 10
   ```

5. **Memory Issues**
   ```bash
   # Check memory usage
   free -h
   
   # Restart system if needed
   sudo reboot
   ```

### Performance Optimization

If the system is running slowly:

1. **Reduce Processing Load**:
   ```bash
   # Lower target FPS
   rosparam set /pinkduckie/*/target_fps 10
   
   # Disable non-essential features
   rosparam set /pinkduckie/*/object_detection_enabled false
   ```

2. **Optimize Image Processing**:
   ```bash
   # Reduce image resolution
   rosparam set /pinkduckie/camera_node/res_w 320
   rosparam set /pinkduckie/camera_node/res_h 240
   ```

3. **Monitor System Health**:
   ```bash
   # Check system performance
   ./src/scripts/monitor_system_performance.sh pinkduckie
   ```

## Safety Considerations

### Emergency Stop

The system includes multiple emergency stop mechanisms:

1. **Hardware Emergency Stop**: Physical button on robot
2. **Software Emergency Stop**: ROS service call
3. **Automatic Emergency Stop**: Triggered by safety monitoring

```bash
# Manual emergency stop
rosservice call /pinkduckie/fsm_node/set_state "state: 'EMERGENCY_STOP'"

# Check safety status
rostopic echo /pinkduckie/safety_monitor/status
```

### Safe Operation Guidelines

1. **Always supervise** the robot during autonomous operation
2. **Test in controlled environment** before open operation
3. **Monitor system performance** regularly
4. **Keep emergency stop accessible** at all times
5. **Check battery level** before operation
6. **Verify calibration** periodically

## Advanced Features

### Lane Changing (Disabled by Default)

Lane changing is disabled for safety. To enable:

```bash
rosparam set /pinkduckie/navigation_node/lane_change_enabled true
```

### Object Detection Tuning

Adjust object detection sensitivity:

```bash
# Increase sensitivity (more detections)
rosparam set /pinkduckie/vehicle_detection_node/confidence_threshold 0.3

# Decrease sensitivity (fewer false positives)
rosparam set /pinkduckie/vehicle_detection_node/confidence_threshold 0.7
```

### Adaptive Threshold Tuning

Fine-tune adaptive lane detection:

```bash
# Faster adaptation
rosparam set /pinkduckie/line_detector_node/adaptation_rate 0.2

# More stable detection
rosparam set /pinkduckie/line_detector_node/consistency_threshold 0.8
```

## Maintenance

### Regular Maintenance Tasks

1. **Update System**:
   ```bash
   # Pull latest changes
   git pull origin main
   
   # Rebuild and redeploy
   ./src/scripts/deploy_pinkduckie.sh
   ```

2. **Clean Docker Images**:
   ```bash
   # On robot
   ssh duckie@pinkduckie.local "docker system prune -f"
   ```

3. **Check Calibration**:
   ```bash
   # Verify calibration accuracy
   dts calibrate_extrinsics pinkduckie --verify
   ```

4. **Monitor Logs**:
   ```bash
   # Check for errors
   grep -i error ~/.ros/log/latest/rosout.log
   ```

## Support and Resources

### Documentation
- [Duckietown Documentation](https://docs.duckietown.org/)
- [ROS Noetic Documentation](http://wiki.ros.org/noetic)
- [Docker Documentation](https://docs.docker.com/)

### Community Support
- [Duckietown Slack](https://duckietown.slack.com/)
- [GitHub Issues](https://github.com/duckietown/dt-core/issues)
- [Stack Overflow](https://stackoverflow.com/questions/tagged/duckietown)

### Troubleshooting Resources
- System logs: `~/.ros/log/latest/`
- Docker logs: `docker logs <container_id>`
- Performance monitoring: `htop`, `iotop`, `nethogs`

## Conclusion

This deployment guide provides comprehensive instructions for deploying and operating the Advanced Autonomous Duckietown System. The system is designed for safe, reliable autonomous operation with extensive monitoring and safety features.

For additional support or questions, please refer to the troubleshooting section or contact the development team through the official Duckietown channels.