cmake_minimum_required(VERSION 2.6)
project(apriltags)

configure_file(${CMAKE_SOURCE_DIR}/cmake/FindEigen.cmake
               ${CMAKE_BINARY_DIR}/FindEigen.cmake @ONLY)

set(CMAKE_MODULE_PATH
    ${CMAKE_BINARY_DIR}
    ${CMAKE_MODULE_PATH}
   )

find_package(Eigen REQUIRED)
include_directories(${EIGEN_INCLUDE_DIR})

## The "imgcodecs" package appears in OpenCV 3.0
# Not sure how to do a version check here and request "highgui" instead...?
find_package(OpenCV REQUIRED core imgcodecs imgproc calib3d )
include_directories(${OpenCV_INCLUDE_DIRS})

include_directories(include)

# libapriltags
file(GLOB SOURCE_FILES "lib/*.cc")
add_library(apriltags ${SOURCE_FILES})

target_link_libraries(apriltags ${OpenCV_LIBS}) #-pg) #-fopenmp)

## Slightly obtuse syntax.  Travis uses CMake 2.8.7
add_definitions( -std=c++11 -pedantic -Wall -Werror )

file(GLOB HEADER_FILES ${CMAKE_SOURCE_DIR}/include/Apriltags/*)
install(PROGRAMS ${HEADER_FILES} DESTINATION include/AprilTags COMPONENT applications)
install(TARGETS apriltags
  RUNTIME DESTINATION bin
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  )

## Configurable options
option( BUILD_DETECTOR_WITH_DEBUG "Build TagDetector with debug outputs" ON )
if( BUILD_DETECTOR_WITH_DEBUG )
  add_definitions( -DBUILD_DEBUG_TAG_DETECTOR )
endif()

option( USE_OPENMP "Build with OpenMP" ON )
if( USE_OPENMP )
  add_definitions( -DUSE_OPENMP )
  add_definitions( "-fopenmp" )
endif()

## Performance and Unit testing
# Location for test images used by unit and performance tests
set( TEST_DATA_DIR ${CMAKE_SOURCE_DIR}/test/data )
add_definitions( -DTEST_DATA_DIR="${TEST_DATA_DIR}" )

option( BUILD_PERF_TESTS "Build performance tests" ON )
if( BUILD_PERF_TESTS )
  add_subdirectory( test/perf )
endif()

option( BUILD_UNIT_TESTS "Build unit tests" ON )
if( BUILD_UNIT_TESTS )
  add_subdirectory( test/unit )
endif()
