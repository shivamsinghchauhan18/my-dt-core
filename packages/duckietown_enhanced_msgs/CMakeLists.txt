cmake_minimum_required(VERSION 3.0.2)
project(duckietown_enhanced_msgs)

find_package(catkin REQUIRED COMPONENTS
  message_generation
  std_msgs
  geometry_msgs
  sensor_msgs
)

add_message_files(
  DIRECTORY msg
  FILES
    ObjectDetection.msg
    ObjectDetectionArray.msg
    SafetyStatus.msg
  PerformanceMetrics.msg
  SystemStatus.msg
  RiskAssessmentStatus.msg
  CoordinationLog.msg
    AdvancedLanePose.msg
    LaneCurve.msg
    LaneCurves.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
  sensor_msgs
)

catkin_package(
  CATKIN_DEPENDS message_runtime std_msgs geometry_msgs sensor_msgs
)
