cmake_minimum_required(VERSION 3.0.2)
project(enhanced_apriltag)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  sensor_msgs
  geometry_msgs
  cv_bridge
  duckietown_enhanced_msgs
  # message_generation  # Commented out since no messages defined
)

# add_message_files(
#   FILES
#   AprilTagDetection.msg
#   AprilTagDetectionArray.msg
# )

# generate_messages(
#   DEPENDENCIES
#   std_msgs
#   geometry_msgs
# )

catkin_package(
  CATKIN_DEPENDS rospy std_msgs sensor_msgs geometry_msgs cv_bridge duckietown_enhanced_msgs # message_runtime
)


catkin_install_python(
  PROGRAMS
    src/enhanced_apriltag_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)