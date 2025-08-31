# Stop line detection message
std_msgs/Header header
bool stop_line_detected
geometry_msgs/Point32 stop_line_point  # position of stop line in vehicle frame (m)
bool at_stop_line                       # true if within stopping distance and lateral bound
float32 distance_to_stop_line           # optional convenience distance
float32 confidence                      # optional confidence score
