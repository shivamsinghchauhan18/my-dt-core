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
    BoolStamped.msg
    FSMState.msg
    Segment.msg
    SegmentList.msg
    Twist2DStamped.msg
    StopLineReading.msg
    LEDPattern.msg
    AntiInstagramThresholds.msg
)

## Generate services in the 'srv' folder
add_service_files(
  FILES
    ChangePattern.srv
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
