# Single object detection result
Header header

# Object classification
string class_name
float32 confidence

# Bounding box in image coordinates
geometry_msgs/Point32 top_left
geometry_msgs/Point32 bottom_right

# 3D information
float32 distance          # Distance to object in meters
geometry_msgs/Vector3 relative_velocity  # Relative velocity vector

# Risk assessment
uint8 RISK_LOW=0
uint8 RISK_MEDIUM=1
uint8 RISK_HIGH=2
uint8 RISK_CRITICAL=3
uint8 risk_level

# Additional metadata
float32 processing_time   # Time taken to detect this object
bool is_tracked          # Whether this object is being tracked across frames