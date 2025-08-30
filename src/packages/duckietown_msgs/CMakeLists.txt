cmake_minimum_required(VERSION 3.0.2)
project(duckietown_msgs)

find_package(catkin REQUIRED COMPONENTS
  message_generation
  std_msgs
  geometry_msgs
  sensor_msgs
)

# Only include enhanced custom messages to avoid conflicts
# In the overlay, do not rebuild or redefine messages to avoid conflicts with the
# official dt-core duckietown_msgs. Enhanced custom messages live in
# 'duckietown_enhanced_msgs'. This package only provides utilities.

catkin_package()
