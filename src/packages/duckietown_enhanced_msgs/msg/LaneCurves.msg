# Collection of lane curves
Header header

# Individual lane curves
LaneCurve left_lane
LaneCurve right_lane
LaneCurve center_line

# Overall lane information
float32 lane_width       # Estimated lane width (meters)
float32 lateral_offset   # Lateral offset from lane center (meters)
float32 heading_error    # Heading error relative to lane direction (radians)

# Quality metrics
float32 overall_confidence  # Overall detection confidence (0-1)
bool lanes_parallel      # Whether left and right lanes are parallel
float32 lane_consistency # Consistency between left and right lane curves (0-1)

# Processing information
float32 processing_time  # Total processing time (seconds)
string algorithm_used    # Algorithm used for curve detection
