cmake_minimum_required(VERSION 2.8.3)
project(rosflight_sil)

SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++17")

# Default to release build for speed
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE "Release")
endif(NOT CMAKE_BUILD_TYPE)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  geometry_msgs
  nav_msgs
  rosflight_firmware
  rosflight_msgs
  utils
  message_generation
)

add_message_files(
  DIRECTORY msg
  FILES
  ROSflightSimState.msg
)

generate_messages(
  DEPENDENCIES
  geometry_msgs
  std_msgs
)

find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp rospy utils geometry_msgs nav_msgs
    rosflight_firmware rosflight_msgs
  DEPENDS EIGEN3
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(rosflight_sil_node
  src/rosflight_sil.cpp
  src/sil_board.cpp
  src/rosflight_sil_node.cpp
)
add_dependencies(rosflight_sil_node
  ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(rosflight_sil_node
  rosflight_firmware
  ${catkin_LIBRARIES}
)
install(TARGETS rosflight_sil_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
