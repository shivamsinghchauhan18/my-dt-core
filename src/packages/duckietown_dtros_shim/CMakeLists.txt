cmake_minimum_required(VERSION 3.0.2)
project(duckietown_dtros_shim)

find_package(catkin REQUIRED)

catkin_python_setup()

catkin_package()

install(DIRECTORY src/
  DESTINATION ${CATKIN_PACKAGE_PYTHON_DESTINATION}
)
