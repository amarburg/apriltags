cmake_minimum_required(VERSION 2.6)
project(apriltags)

# configure_file(${CMAKE_SOURCE_DIR}/cmake/FindEigen.cmake
#                ${CMAKE_BINARY_DIR}/FindEigen.cmake @ONLY)

list(INSERT CMAKE_MODULE_PATH 0
    ${CMAKE_SOURCE_DIR}/cmake )

find_package(Eigen REQUIRED)
include_directories(${EIGEN_INCLUDE_DIR})

## The "imgcodecs" package appears in OpenCV 3.0
# Not sure how to do a version check here and request "highgui" instead...?
find_package(OpenCV REQUIRED core imgproc calib3d )
if( OpenCV_VERSION_MAJOR VERSION_LESS "3.0.0" )
  find_package(OpenCV REQUIRED highgui )
else()
  find_package(OpenCV REQUIRED imgcodecs )
endif()

include_directories(${OpenCV_INCLUDE_DIRS})
include_directories(include lib)



## Slightly obtuse syntax.
add_definitions( -std=c++11 -Wall )
#-pg) #-fopenmp)
#add_definitions( -std=c++11 -pedantic -Wall )

set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -ggdb")
#  -Werror (Eigen version 3.0.5 has unresolved warnings on travis, so cannot use -Werror)
# see: http://eigen.tuxfamily.org/bz/show_bug.cgi?id=666

# use default for now
#set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE}")

# Use default for now
#set(CMAKE_CXX_FLAGS_RELWITHDEBINFO "${CMAKE_CXX_FLAGS_RELWITHDEBINFO} -O3 -ggdb")



#
# libapriltags
#
file(GLOB HEADER_FILES ${CMAKE_SOURCE_DIR}/include/Apriltags/*)
file(GLOB SOURCE_FILES "lib/*/*.cc" "lib/*.cc" )
add_library(apriltags ${SOURCE_FILES})

target_link_libraries(apriltags ${OpenCV_LIBS})

# TODO:  This breaks with subdirectories in include/Apriltags
# install(PROGRAMS ${HEADER_FILES} DESTINATION include/AprilTags COMPONENT applications)
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
#
# Location for test images used by unit and performance tests
#
set( TEST_DATA_DIR ${CMAKE_SOURCE_DIR}/test/data )
add_definitions( -DTEST_DATA_DIR="${TEST_DATA_DIR}" )

option( BUILD_PERF_TESTS "Build performance tests" ON )
option( BUILD_UNIT_TESTS "Build unit tests" ON )

if( BUILD_PERF_TESTS OR BUILD_UNIT_TESTS)
  include( ${CMAKE_SOURCE_DIR}/cmake/BuildGTestLibraries.cmake )

  if( BUILD_PERF_TESTS )
    add_subdirectory( test/perf )
  endif()

  if( BUILD_UNIT_TESTS )
    add_subdirectory( test/unit )
  endif()

endif()
