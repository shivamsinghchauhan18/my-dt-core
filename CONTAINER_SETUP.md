# Enhanced Autonomous System - Container Setup Guide

This guide provides step-by-step instructions for setting up and running the enhanced autonomous system inside the Duckietown container.

## Prerequisites

1. Duckietown container running with your repository cloned
2. Container with network access for downloading YOLO model
3. Sufficient memory (>2GB recommended)

## Setup Steps

### 1. Container Access
```bash
# Enter the dt-duckiebot-interface container
docker exec -it dt-duckiebot-interface bash
```

### 2. Navigate to Workspace
```bash
# Go to the enhance_ws workspace
cd /code/enhance_ws
```

### 3. Source ROS Environment
```bash
# Source ROS base environment
source /opt/ros/noetic/setup.bash

# If workspace is already built, source it
source devel/setup.bash 2>/dev/null || echo "Workspace not built yet"
```

### 4. Build the Workspace
```bash
# Build the enhanced packages
catkin_make

# Source the newly built workspace
source devel/setup.bash
```

### 5. Download YOLO Model
```bash
# Run the model download script
cd /code/enhance_ws/src/my-dt-core
./scripts/download_yolo_model.sh
```

### 6. Set Environment Variables
```bash
# Set required environment variables
export VEHICLE_NAME="blueduckie"  # or your robot name
export ROS_MASTER_URI="http://localhost:11311"
export ROS_IP="127.0.0.1"
export PYTHONPATH="/code/enhance_ws/src/my-dt-core/packages:$PYTHONPATH"
```

### 7. Validate System
```bash
# Run pre-launch validation
./scripts/validate_system.sh
```

### 8. Launch Enhanced Autonomous System
```bash
# Launch the enhanced autonomous system
./launchers/enhanced_autonomous_system.sh
```

## Troubleshooting

### Common Issues and Solutions

#### 1. YOLO Model Not Found
```bash
# If model download fails, try manual download
cd /code/enhance_ws/src/my-dt-core/packages/vehicle_detection
wget https://github.com/ultralytics/yolov5/releases/download/v7.0/yolov5s.pt
```

#### 2. Permission Denied
```bash
# Fix permissions for executable files
chmod +x /code/enhance_ws/src/my-dt-core/launchers/enhanced_autonomous_system.sh
chmod +x /code/enhance_ws/src/my-dt-core/packages/vehicle_detection/src/enhanced_vehicle_detection_node.py
chmod +x /code/enhance_ws/src/my-dt-core/packages/duckietown_demos/scripts/enhanced_system_startup.py
```

#### 3. Python Import Errors
```bash
# Ensure Python path is set correctly
export PYTHONPATH="/code/enhance_ws/src/my-dt-core/packages:$PYTHONPATH"

# Test imports
python3 -c "
import sys
sys.path.insert(0, '/code/enhance_ws/src/my-dt-core/packages')
try:
    from vehicle_detection.src.optimized_yolo_detector import OptimizedYOLODetector
    print('✅ YOLO detector import OK')
except Exception as e:
    print(f'❌ Import failed: {e}')
"
```

#### 4. Workspace Build Errors
```bash
# Clean and rebuild workspace
cd /code/enhance_ws
catkin clean -y
catkin_make
source devel/setup.bash
```

#### 5. ROS Topic Communication Issues
```bash
# Check ROS environment
echo "ROS_MASTER_URI: $ROS_MASTER_URI"
echo "ROS_IP: $ROS_IP"

# Check if roscore is running
rostopic list

# Check for vehicle topics
rostopic list | grep $VEHICLE_NAME
```

## Verification Commands

### Check System Status
```bash
# Check running nodes
rosnode list

# Check topics
rostopic list

# Check vehicle detection output
rostopic echo /$VEHICLE_NAME/enhanced_vehicle_detection_node/detections

# Monitor system performance
htop
```

### Debug Mode
```bash
# Run launcher in verbose mode
VERBOSE=true ./launchers/enhanced_autonomous_system.sh

# Check log files
tail -f /tmp/enhanced_autonomous_system_logs/launch_*.log
```

## Expected Behavior

When successfully launched, you should see:

1. **ROS Core Start**: ROS master starts or connects to existing one
2. **Configuration Loading**: System loads configurations from YAML files  
3. **Startup Manager**: Enhanced system startup manager begins component initialization
4. **Phase Execution**: 5 startup phases execute sequentially:
   - Phase 1: Core Infrastructure (FSM, camera, wheels, LEDs)
   - Phase 2: Perception Layer (line detection, lane filter, AprilTag, YOLO)
   - Phase 3: Control Layer (lane controller, navigation, lane changing)
   - Phase 4: Safety & Coordination (safety monitoring, coordination)
   - Phase 5: Integration & Monitoring (system health, logging)
5. **System Ready**: All components operational, autonomous driving active

## Performance Expectations

- **Startup Time**: 60-120 seconds for full system initialization
- **Memory Usage**: 1.5-2.5GB typical, depending on YOLO model and image resolution
- **CPU Usage**: 50-80% during active detection and navigation
- **Frame Rate**: 5-15 FPS for camera processing and object detection

## Support

If you encounter issues:

1. Run the validation script: `./scripts/validate_system.sh`
2. Check log files in `/tmp/enhanced_autonomous_system_logs/`
3. Verify YOLO model exists and is accessible
4. Ensure all required Python packages are installed
5. Confirm ROS environment is properly configured
