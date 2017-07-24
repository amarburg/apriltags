cmake_minimum_required(VERSION 2.6)
project(apriltags)

option(USE_CONAN  "Use the Conan packaging system" OFF)
if(USE_CONAN)
  message("Using Conan")
  include(${CMAKE_BINARY_DIR}/conanbuildinfo.cmake)
  conan_basic_setup()
endif()

configure_file(${CMAKE_SOURCE_DIR}/cmake/FindEigen.cmake
               ${CMAKE_BINARY_DIR}/FindEigen.cmake @ONLY)

list(INSERT CMAKE_MODULE_PATH 0
    ${CMAKE_SOURCE_DIR}/cmake )

## Get Eigen using the system
find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})

find_package(OpenCV REQUIRED core imgproc calib3d )
if( OpenCV_VERSION_MAJOR VERSION_LESS "3.0.0" )
  find_package(OpenCV REQUIRED highgui )
else()
  find_package(OpenCV REQUIRED imgcodecs )
endif()

include_directories(${OpenCV_INCLUDE_DIRS})
include_directories(include lib)

add_definitions( -std=c++11 -Wall )

set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -ggdb")

#
# libapriltags
#
file(GLOB HEADER_FILES ${CMAKE_SOURCE_DIR}/include/Apriltags/*)
file(GLOB SOURCE_FILES "lib/*/*.cc" "lib/*.cc" )
option(SHARED_LIBS "Build shared libraries" ON)
if(SHARED_LIBS)
  add_library(apriltags SHARED ${SOURCE_FILES})
else()
  add_library(apriltags STATIC ${SOURCE_FILES})
endif()

add_definitions( -Wall -std=c++11 )
#add_definitions( -std=c++11 -pedantic -Wall -Werror )

# TODO:  This breaks with subdirectories in include/Apriltags
install(TARGETS apriltags
  RUNTIME DESTINATION bin
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
)

## Configurable options
option( USE_OPENMP "Build with OpenMP" ON )
if( USE_OPENMP )
  add_definitions( -DUSE_OPENMP )
  add_definitions( "-fopenmp" )
endif()

#
## Performance and Unit testing
## To enable testing through conan, do
##   conan install --scope build_tests=true

# Location for test images used by unit and performance tests
#
set( TEST_DATA_DIR ${CMAKE_SOURCE_DIR}/test/data )
add_definitions( -DTEST_DATA_DIR="${TEST_DATA_DIR}" )

option( BUILD_PERF_TESTS "Build performance tests" ON )
option( BUILD_UNIT_TESTS "Build unit tests" ON )

if( BUILD_PERF_TESTS OR BUILD_UNIT_TESTS AND NOT USE_CONAN)
  include( ${CMAKE_SOURCE_DIR}/cmake/BuildGTestLibraries.cmake )

  if( BUILD_PERF_TESTS )
    add_subdirectory( test/perf )
  endif()

  if( BUILD_UNIT_TESTS )
    add_subdirectory( test/unit )
  endif()

endif()
