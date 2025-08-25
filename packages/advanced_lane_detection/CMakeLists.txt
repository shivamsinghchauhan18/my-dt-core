cmake_minimum_required(VERSION 3.0.2)
project(advanced_lane_detection)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  sensor_msgs
  geometry_msgs
  cv_bridge
  message_generation
)

add_message_files(
  FILES
  LaneCurve.msg
  LaneCurves.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
)

catkin_package(
  CATKIN_DEPENDS rospy std_msgs sensor_msgs geometry_msgs cv_bridge message_runtime
)