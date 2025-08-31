# Line segment with image and ground coordinates
# Color constants
uint8 WHITE=0
uint8 YELLOW=1
uint8 RED=2

# Normalized pixel coordinates [0,1] in image frame
geometry_msgs/Point32[2] pixels_normalized

# Ground-projected points in vehicle frame (meters)
geometry_msgs/Point32[2] points

# Unit normal vector pointing towards the lane interior (vehicle frame)
geometry_msgs/Vector3 normal

# Segment color
uint8 color
