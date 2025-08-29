# Duckietown Messages Package

This package contains ROS message definitions for the Duckietown platform, including both existing messages used throughout the codebase and new advanced messages for the autonomous system.

## Message Types

### Existing Messages (Legacy Compatibility)
- `LanePose.msg` - Basic lane pose with lateral offset and heading error
- `Segment.msg` - Line segment in image coordinates
- `SegmentList.msg` - Collection of line segments
- `FSMState.msg` - Finite State Machine state
- `BoolStamped.msg` - Boolean value with timestamp
- `WheelsCmdStamped.msg` - Wheel velocity commands
- `Twist2DStamped.msg` - 2D twist command with timestamp
- `VehicleCorners.msg` - Vehicle corner detection results
- `StopLineReading.msg` - Stop line detection information
- `CoordinationClearance.msg` - Coordination clearance status
- `CoordinationSignal.msg` - Inter-vehicle coordination signals
- `SignalsDetection.msg` - Traffic signal detection results
- `AprilTagsWithInfos.msg` - AprilTag detections with metadata
- `MaintenanceState.msg` - System maintenance status

### New Advanced Messages
- `ObjectDetection.msg` - Single object detection with risk assessment
- `ObjectDetectionArray.msg` - Array of object detections with summary
- `SafetyStatus.msg` - Comprehensive safety monitoring status
- `AdvancedLanePose.msg` - Enhanced lane pose with curve information
- `LaneCurve.msg` - Polynomial curve representation for lanes
- `LaneCurves.msg` - Collection of lane curves with quality metrics

### Service Types
- `ChangePattern.srv` - Service to change LED patterns

## Advanced Vision Utilities

The package also includes advanced vision utility classes in `packages/image_processing/src/advanced_vision_utils.py`:

### Data Structures
- `Point3D`, `Vector3D` - 3D geometric primitives
- `BoundingBox` - 2D bounding box with utility methods
- `VehicleState` - Complete vehicle state representation
- `ObjectDetection` - Object detection with risk assessment
- `SafetyStatus` - Comprehensive safety status

### Enumerations
- `RiskLevel` - Object risk levels (LOW, MEDIUM, HIGH, CRITICAL)
- `SafetyLevel` - System safety levels (SAFE, WARNING, CRITICAL, EMERGENCY)
- `HealthStatus` - Component health status (OK, WARNING, CRITICAL, FAILED)

### Algorithm Classes
- `AdaptiveThresholdDetector` - Adaptive thresholding for varying lighting
- `TemporalConsistencyFilter` - Multi-frame consistency checking
- `PolynomialCurveFitter` - Polynomial curve fitting for trajectories
- `ROIManager` - Dynamic region of interest management
- `PerformanceMonitor` - Algorithm performance tracking

## Message Utilities

The `msg_utils.py` module provides utility functions for converting between internal data structures and ROS messages:

- `create_header()` - Create standard ROS headers
- `object_detection_to_msg()` - Convert ObjectDetection to ROS message
- `object_detection_array_to_msg()` - Convert detection arrays to ROS messages
- `safety_status_to_msg()` - Convert SafetyStatus to ROS message
- `create_advanced_lane_pose()` - Create AdvancedLanePose messages
- `create_lane_curve()` - Create LaneCurve messages

## Dependencies

### Build Dependencies
- `catkin`
- `message_generation`
- `std_msgs`
- `geometry_msgs`
- `sensor_msgs`

### Runtime Dependencies
- `message_runtime`
- `std_msgs`
- `geometry_msgs`
- `sensor_msgs`

### Python Dependencies
- `numpy`
- `opencv-python` (cv2)
- `torch` (for deep learning components)
- `ultralytics` (for YOLO object detection)
- `scipy` (for advanced algorithms)

## Usage Example

```python
#!/usr/bin/env python3
import rospy
from duckietown_enhanced_msgs.msg import ObjectDetectionArray, SafetyStatus
from duckietown_msgs.msg_utils import create_header

# Create a safety status message
safety_msg = SafetyStatus()
safety_msg.header = create_header("base_link")
safety_msg.safety_level = SafetyStatus.SAFE
safety_msg.system_health_score = 95.0

# Publish the message
pub = rospy.Publisher('/safety_status', SafetyStatus, queue_size=1)
pub.publish(safety_msg)
```

## Testing

Run the basic tests to verify the data structures work correctly:

```bash
python3 packages/duckietown_msgs/test_basic.py
```

Note: Full ROS message tests require a ROS environment with the messages compiled.

## Integration

This package is designed to integrate seamlessly with the existing Duckietown codebase while providing enhanced capabilities for advanced autonomous navigation. All existing message types are preserved for backward compatibility.