cmake_minimum_required (VERSION 2.8.11)
project(logging-utils-lib)

set(CMAKE_CXX_STANDARD 17)

find_package(Boost REQUIRED COMPONENTS system thread)
find_package(Eigen3 REQUIRED)
find_package(yaml-cpp REQUIRED)

include_directories(
  include
  ${YAML_CPP_INCLUDE_DIR}
  ${EIGEN3_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
)

option(BUILD_TESTS "Build Tests" OFF) # ON
if (BUILD_TESTS)
    find_package(GTest REQUIRED)
    add_executable(logging-utils-lib-test src/test.cpp)
    target_include_directories(logging-utils-lib-test PUBLIC include ${GTEST_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS} ${YAML_CPP_INCLUDE_DIR})
    target_link_libraries(logging-utils-lib-test ${GTEST_LIBRARIES} gtest gtest_main pthread)
endif()

add_library(logging-utils-lib INTERFACE)
target_include_directories(logging-utils-lib
  INTERFACE
  include
  ${YAML_CPP_INCLUDE_DIR}
  ${EIGEN3_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
)
