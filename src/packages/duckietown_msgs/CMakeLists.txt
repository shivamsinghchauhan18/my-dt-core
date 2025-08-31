cmake_minimum_required(VERSION 3.0.2)
project(duckietown_msgs)

find_package(catkin REQUIRED COMPONENTS
  message_generation
  std_msgs
  geometry_msgs
  sensor_msgs
)

## Generate messages in the 'msg' folder
add_message_files(
  DIRECTORY msg
  FILES
  AprilTagDetection.msg
  AprilTagDetectionArray.msg
    BoolStamped.msg
    FSMState.msg
  LanePose.msg
    Segment.msg
    SegmentList.msg
    Twist2DStamped.msg
  WheelsCmdStamped.msg
    StopLineReading.msg
    LEDPattern.msg
    AntiInstagramThresholds.msg
)

## Generate services in the 'srv' folder
add_service_files(
  FILES
    ChangePattern.srv
  SetCustomLEDPattern.srv
  SetBool.srv
  SetFSMState.srv
)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
  sensor_msgs
)

catkin_package(
  CATKIN_DEPENDS message_runtime std_msgs geometry_msgs sensor_msgs
)
