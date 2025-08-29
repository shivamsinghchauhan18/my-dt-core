cmake_minimum_required(VERSION 2.8.3)
project(vehicle_detection)

find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  duckietown_enhanced_msgs
  geometry_msgs
  sensor_msgs
  image_processing
  roscpp
  rospy
)

catkin_python_setup()

catkin_package(
  CATKIN_DEPENDS duckietown_enhanced_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)
