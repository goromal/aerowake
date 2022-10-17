cmake_minimum_required(VERSION 2.8.3)
project(unscented_kalman_filter)
add_definitions(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  mav_msgs
  nav_msgs
  roscpp
  sensor_msgs
  cmake_modules
  dynamic_reconfigure
  utils
  visualization_msgs
)

find_package(Eigen REQUIRED)

# options must be called before catkin_package, dependencies after
generate_dynamic_reconfigure_options(
  cfg/UKF.cfg
)

catkin_package(
  INCLUDE_DIRS include ${Eigen_INCLUDE_DIRS}
  LIBRARIES unscented_kalman_filter 
  CATKIN_DEPENDS geometry_msgs mav_msgs nav_msgs roscpp sensor_msgs visualization_msgs
  DEPENDS Eigen
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Eigen_INCLUDE_DIRS}
)

add_library(unscented_kalman_filter
  src/unscented_kalman_filter.cpp
  src/force_offset_fsm.cpp
)


target_link_libraries(unscented_kalman_filter ${catkin_LIBRARIES})
add_dependencies(unscented_kalman_filter ${catkin_EXPORTED_TARGETS})


add_executable(unscented_kalman_filter_node src/nodes/unscented_kalman_filter_node.cpp)
add_dependencies(unscented_kalman_filter_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(unscented_kalman_filter_node
  unscented_kalman_filter ${catkin_LIBRARIES})

  install(TARGETS unscented_kalman_filter
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

install(TARGETS unscented_kalman_filter_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(
  DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
)

add_dependencies(unscented_kalman_filter_node ${PROJECT_NAME}_gencfg)

