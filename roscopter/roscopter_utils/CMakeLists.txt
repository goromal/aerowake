cmake_minimum_required(VERSION 2.8.3)
project(roscopter_utils)

find_package(catkin REQUIRED COMPONENTS
    rospy
    roscpp
)
catkin_package(
  INCLUDE_DIRS include lib/geometry/include
)

include_directories(lib/geometry/include include)

install(DIRECTORY lib/geometry/include/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}/..
  FILES_MATCHING PATTERN "*.h"
)
install(DIRECTORY include/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}/..
  FILES_MATCHING PATTERN "*.h"
)
