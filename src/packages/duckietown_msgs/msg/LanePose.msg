# Lane pose estimate
std_msgs/Header header
float32 d       # lateral displacement [m]
float32 phi     # heading with respect to lane axis [rad]
bool in_lane

# Status constants
uint8 NORMAL=0
uint8 WARNING=1
uint8 ERROR=2
uint8 status