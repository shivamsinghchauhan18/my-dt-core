# Single lane curve representation
Header header

# Polynomial coefficients for curve fitting (y = ax^3 + bx^2 + cx + d)
float32 a  # Cubic coefficient
float32 b  # Quadratic coefficient  
float32 c  # Linear coefficient
float32 d  # Constant coefficient

# Curve properties
float32 curvature        # Curvature at current position (1/radius)
float32 curve_radius     # Radius of curvature (meters)
float32 arc_length       # Length of detected curve segment (meters)

# Quality metrics
float32 fit_quality      # R-squared value of polynomial fit (0-1)
float32 confidence       # Detection confidence (0-1)
int32 num_points        # Number of points used for fitting

# Spatial bounds
float32 start_x         # Start x-coordinate of valid curve segment
float32 end_x           # End x-coordinate of valid curve segment
float32 max_deviation   # Maximum deviation from fitted curve

# Lane type
uint8 LANE_LEFT=0
uint8 LANE_RIGHT=1
uint8 LANE_CENTER=2
uint8 lane_type