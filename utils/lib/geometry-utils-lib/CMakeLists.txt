cmake_minimum_required (VERSION 2.8.11)
project(geometry-utils-lib)

set(CMAKE_CXX_STANDARD 17)

find_package(Eigen3 REQUIRED)

include_directories(
  include
  ${EIGEN3_INCLUDE_DIRS}
)

option(BUILD_TESTS "Build Tests" OFF) # ON
if (BUILD_TESTS)
    find_package(GTest REQUIRED)
    add_executable(geometry-utils-lib-test src/test.cpp)
    target_include_directories(geometry-utils-lib-test PUBLIC include ${GTEST_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIRS})
    target_link_libraries(geometry-utils-lib-test ${GTEST_LIBRARIES} gtest gtest_main pthread)
endif()

add_library(geometry-utils-lib INTERFACE)
target_include_directories(geometry-utils-lib
  INTERFACE
  include
  ${EIGEN3_INCLUDE_DIRS}
)
