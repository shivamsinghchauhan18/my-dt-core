# Advanced lane pose with curve information
Header header

# Basic lane pose (compatible with existing LanePose)
float32 d              # Lateral offset from lane center (meters)
float32 phi            # Heading error (radians)

# Advanced curve information
float32 curvature      # Lane curvature (1/radius in meters)
float32 curve_radius   # Radius of curvature (meters, positive for left curves)
float32 curve_confidence  # Confidence in curve estimation (0-1)

# Predictive information
geometry_msgs/Point32[] predicted_path  # Predicted lane center points
float32 prediction_horizon  # Time horizon for prediction (seconds)

# Detection quality
float32 detection_confidence  # Overall detection confidence (0-1)
float32 left_lane_confidence  # Left lane detection confidence (0-1)
float32 right_lane_confidence # Right lane detection confidence (0-1)

# Temporal consistency
int32 frames_tracked     # Number of consecutive frames this pose has been tracked
float32 temporal_consistency  # Consistency with previous frames (0-1)

# Processing metadata
float32 processing_time  # Time taken to compute this pose (seconds)
string detection_method  # Method used for detection (e.g., "adaptive_threshold", "deep_learning")