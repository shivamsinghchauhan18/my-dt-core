# AprilTag single detection message
std_msgs/Header header
geometry_msgs/Transform transform
int32 tag_id
string tag_family
int32 hamming
float32 decision_margin
float32[9] homography
float32[2] center
float32[8] corners  # x1,y1,x2,y2,x3,y3,x4,y4
float32 pose_error