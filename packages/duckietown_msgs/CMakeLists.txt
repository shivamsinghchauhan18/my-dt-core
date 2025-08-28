cmake_minimum_required(VERSION 3.0.2)
project(duckietown_msgs)

find_package(catkin REQUIRED COMPONENTS
  message_generation
  std_msgs
  geometry_msgs
  sensor_msgs
)

# Only include enhanced custom messages to avoid conflicts
file(GLOB ENHANCED_MSG_FILES "msg/enhanced/*.msg")

if(ENHANCED_MSG_FILES)
  # Extract just the filename from the full path for add_message_files
  set(ENHANCED_MSG_NAMES "")
  foreach(msg_file ${ENHANCED_MSG_FILES})
    get_filename_component(msg_name ${msg_file} NAME)
    list(APPEND ENHANCED_MSG_NAMES ${msg_name})
  endforeach()
  
  add_message_files(
    DIRECTORY msg/enhanced
    FILES ${ENHANCED_MSG_NAMES}
  )
  
  message(STATUS "Adding enhanced messages: ${ENHANCED_MSG_NAMES}")
else()
  message(WARNING "No enhanced messages found in msg/enhanced/")
endif()

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
  sensor_msgs
)

catkin_package(
  CATKIN_DEPENDS message_runtime std_msgs geometry_msgs sensor_msgs
)
