cmake_minimum_required(VERSION 2.8.3)
project(boat_dynamics)

set(CMAKE_CXX_STANDARD 17)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  geometry_msgs
  visualization_msgs
  tf2
  tf2_ros
  utils
)

find_package(Eigen3 REQUIRED)

catkin_package(
  INCLUDE_DIRS include ${EIGEN3_INCLUDE_DIRS}
  CATKIN_DEPENDS roscpp rospy geometry_msgs visualization_msgs
    tf2 tf2_ros utils
  DEPENDS EIGEN3
)

include_directories(
  include
  ${EIGEN3_INCLUDE_DIR}
  ${catkin_INCLUDE_DIRS}
)

add_executable(boat_dynamics_node
  src/boat_dynamics_node.cpp
  src/boat_dynamics.cpp
)
add_dependencies(boat_dynamics_node
  ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(boat_dynamics_node
  ${catkin_LIBRARIES}
)
install(TARGETS boat_dynamics_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

add_executable(boat_to_boatNWU_broadcaster
  src/boat_to_boatNWU_broadcaster.cpp
)
add_dependencies(boat_to_boatNWU_broadcaster
  ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(boat_to_boatNWU_broadcaster
  ${catkin_LIBRARIES}
)
install(TARGETS boat_to_boatNWU_broadcaster
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

add_executable(simulated_mocap_boat
  src/simulated_mocap.cpp
)
add_dependencies(simulated_mocap_boat
  ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(simulated_mocap_boat
  ${catkin_LIBRARIES}
)
install(TARGETS simulated_mocap_boat
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
