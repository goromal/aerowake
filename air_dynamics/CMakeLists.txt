cmake_minimum_required(VERSION 2.8.3)
project(air_dynamics)

set(CMAKE_CXX_STANDARD 17)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  geometry_msgs
  visualization_msgs
  rosflight_sil
  utils
)

find_package(Eigen3 REQUIRED)

catkin_package(
  INCLUDE_DIRS include ${EIGEN3_INCLUDE_DIRS}
  CATKIN_DEPENDS roscpp rospy geometry_msgs visualization_msgs
    rosflight_sil utils
  DEPENDS EIGEN3
)

include_directories(
  include
  ${EIGEN3_INCLUDE_DIR}
  ${catkin_INCLUDE_DIRS}
)

add_executable(air_dynamics_node
  src/air_dynamics_node.cpp
  src/air_dynamics.cpp
)
add_dependencies(air_dynamics_node
  ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(air_dynamics_node
  ${catkin_LIBRARIES}
)
install(TARGETS air_dynamics_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
