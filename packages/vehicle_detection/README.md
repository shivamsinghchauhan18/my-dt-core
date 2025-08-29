# Enhanced Vehicle Detection Package

## Overview

The Enhanced Vehicle Detection package provides advanced object detection capabilities for Duckietown environments using YOLOv5 deep learning models. This package extends the basic vehicle detection with comprehensive object recognition, real-time performance monitoring, and intelligent risk assessment.

## Features

### 🎯 Advanced Object Detection
- **YOLOv5-based detection** with TensorRT optimization support
- **Custom Duckietown classes**: duckies, duckiebots, traffic cones, stop signs, intersection signs, pedestrians, vehicles, and barriers
- **Multi-resolution detection** for improved robustness
- **Confidence-based filtering** with configurable thresholds

### 📊 Performance Monitoring
- **Real-time performance metrics**: FPS, inference time, memory usage, GPU utilization
- **Comprehensive logging** with detailed debug information
- **Performance alerting** for resource constraints
- **Automatic load balancing** between detection modules

### 🛡️ Safety & Risk Assessment
- **Multi-factor risk calculation** based on distance, object type, and confidence
- **Distance estimation** using object size heuristics
- **Risk level classification**: Low, Medium, High, Critical
- **Emergency response integration** with safety monitoring systems

### 🔧 Developer-Friendly
- **Cross-platform support**: macOS development, Linux deployment
- **Comprehensive unit tests** with mocked dependencies
- **Detailed configuration options** via YAML files
- **Debug visualization** with bounding boxes and risk indicators

## Architecture

```
Enhanced Vehicle Detection Node
├── OptimizedYOLODetector
│   ├── Model Loading & Optimization
│   ├── TensorRT Integration
│   ├── Multi-scale Detection
│   └── Performance Monitoring
├── Risk Assessment Engine
│   ├── Distance Estimation
│   ├── Object Type Classification
│   └── Multi-factor Risk Scoring
└── ROS Integration
    ├── Image Subscription
    ├── Detection Publishing
    └── Debug Visualization
```

## Installation

### Prerequisites

Ensure the following dependencies are installed:

```bash
# Python dependencies (already in dependencies-py3.txt)
torch>=1.9.0
ultralytics>=8.0.0
scipy>=1.7.0
psutil>=5.8.0
```

### Build Instructions

```bash
# Build the package
catkin build vehicle_detection

# Source the workspace
source devel/setup.bash
```

## Usage

### Basic Launch

```bash
# Launch enhanced vehicle detection
roslaunch vehicle_detection enhanced_vehicle_detection_node.launch

# With custom parameters
roslaunch vehicle_detection enhanced_vehicle_detection_node.launch \
    model_path:="yolov5m.pt" \
    confidence_threshold:=0.7 \
    device:="cuda"
```

### Configuration

The node can be configured via YAML files in `config/enhanced_vehicle_detection_node/`:

```yaml
# Model Configuration
model_path: "yolov5s.pt"
confidence_threshold: 0.6
nms_threshold: 0.45
device: "auto"  # cpu, cuda, or auto
tensorrt_enabled: true

# Performance Settings
process_frequency: 10.0  # Hz
max_detections: 100

# Risk Assessment
risk_assessment:
  critical_distance: 0.5  # meters
  high_distance: 1.0
  medium_distance: 2.0
```

### ROS Topics

#### Subscribed Topics
- `~image` (`sensor_msgs/CompressedImage`): Input camera image

#### Published Topics
- `~detections` (`duckietown_enhanced_msgs/ObjectDetectionArray`): Detected objects with risk assessment
- `~debug/detection_image/compressed` (`sensor_msgs/CompressedImage`): Debug visualization
- `~performance_status` (`std_msgs/String`): Performance monitoring information

## Message Definitions

### ObjectDetection
```yaml
Header header
string class_name          # Object class (duckie, duckiebot, etc.)
float32 confidence         # Detection confidence (0.0-1.0)
geometry_msgs/Point32 top_left      # Bounding box top-left
geometry_msgs/Point32 bottom_right  # Bounding box bottom-right
float32 distance           # Estimated distance (meters)
geometry_msgs/Vector3 relative_velocity  # Relative velocity
uint8 risk_level          # Risk assessment (0=Low, 1=Medium, 2=High, 3=Critical)
float32 processing_time   # Detection processing time
bool is_tracked          # Object tracking status
```

### ObjectDetectionArray
```yaml
Header header
ObjectDetection[] detections    # Array of detected objects
int32 total_objects            # Total number of objects
float32 processing_time        # Total processing time
float32 inference_fps          # Current inference FPS
float32 average_confidence     # Average detection confidence
int32 high_risk_objects_count  # Number of high-risk objects
```

## Performance Specifications

### Requirements (from specification)
- **Latency**: < 200ms end-to-end
- **FPS**: > 20 FPS minimum
- **Memory**: < 2GB peak usage
- **Accuracy**: > 60% confidence threshold

### Typical Performance
- **CPU (Intel i7)**: ~15-25 FPS, 150-200ms latency
- **GPU (RTX 3060)**: ~30-50 FPS, 50-100ms latency
- **Raspberry Pi 4**: ~5-10 FPS, 200-400ms latency

## Testing

### Unit Tests

```bash
# Run validation script (no ROS required)
cd packages/vehicle_detection
python3 tests/validate_implementation.py

# Run full unit tests (requires ROS environment)
rostest vehicle_detection test_yolo_detection_accuracy.test
```

### Integration Testing

```bash
# Test with sample images
rostopic pub /duckiebot/camera_node/image/compressed sensor_msgs/CompressedImage [...]

# Monitor detection results
rostopic echo /duckiebot/enhanced_vehicle_detection_node/detections

# View debug visualization
rqt_image_view /duckiebot/enhanced_vehicle_detection_node/debug/detection_image/compressed
```

## Troubleshooting

### Common Issues

1. **CUDA Out of Memory**
   ```bash
   # Reduce batch size or use CPU
   roslaunch vehicle_detection enhanced_vehicle_detection_node.launch device:="cpu"
   ```

2. **Low FPS Performance**
   ```bash
   # Check GPU utilization
   nvidia-smi
   
   # Reduce image resolution or detection frequency
   rosparam set /enhanced_vehicle_detection_node/process_frequency 5.0
   ```

3. **Model Loading Errors**
   ```bash
   # Verify model path and permissions
   ls -la /path/to/yolov5s.pt
   
   # Download default model
   wget https://github.com/ultralytics/yolov5/releases/download/v6.0/yolov5s.pt
   ```

### Debug Logging

Enable detailed logging:

```bash
# Set debug level
rosparam set /enhanced_vehicle_detection_node/logging/level "DEBUG"

# Monitor logs
rostopic echo /enhanced_vehicle_detection_node/performance_status
```

## Development

### Adding New Object Classes

1. Update `DUCKIETOWN_CLASSES` in `optimized_yolo_detector.py`
2. Modify risk assessment logic in `_assess_risk_level()`
3. Update configuration file with new class parameters
4. Retrain YOLO model with new classes (if needed)

### Performance Optimization

1. **Model Optimization**
   - Use TensorRT for NVIDIA GPUs
   - Quantize model to FP16 or INT8
   - Use smaller model variants (YOLOv5s vs YOLOv5x)

2. **Processing Optimization**
   - Adjust `process_frequency` based on requirements
   - Implement region of interest (ROI) processing
   - Use multi-threading for parallel processing

### Custom Model Training

```python
# Train custom Duckietown model
from ultralytics import YOLO

# Load base model
model = YOLO('yolov5s.pt')

# Train on Duckietown dataset
model.train(
    data='duckietown_dataset.yaml',
    epochs=100,
    imgsz=640,
    batch_size=16
)
```

## Contributing

1. Fork the repository
2. Create a feature branch
3. Add comprehensive tests
4. Update documentation
5. Submit a pull request

### Code Style

- Follow PEP 8 for Python code
- Add comprehensive docstrings
- Include type hints where appropriate
- Add logging for debugging and monitoring

## License

This package is part of the Duckietown project and follows the same licensing terms.

## References

- [YOLOv5 Documentation](https://docs.ultralytics.com/yolov5/)
- [Duckietown Documentation](https://docs.duckietown.org/)
- [ROS Noetic Documentation](http://wiki.ros.org/noetic)
- [TensorRT Documentation](https://docs.nvidia.com/deeplearning/tensorrt/)

## Changelog

### v1.0.0 (Current)
- Initial implementation with YOLOv5 integration
- Custom Duckietown object classes
- Real-time performance monitoring
- Risk assessment and safety integration
- Comprehensive testing framework
- Cross-platform development support