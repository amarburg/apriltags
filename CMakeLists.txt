if (NOT FIPS_IMPORT)
  cmake_minimum_required(VERSION 3.5.0)

  get_filename_component(FIPS_ROOT_DIR "../fips" ABSOLUTE)
  include("${FIPS_ROOT_DIR}/cmake/fips.cmake")

  fips_setup(PROJECT apriltags)
endif()

list(INSERT CMAKE_MODULE_PATH 0
    ${CMAKE_SOURCE_DIR}/cmake )

find_package(Eigen REQUIRED)
include_directories(${EIGEN_INCLUDE_DIR})

## The "imgcodecs" package is new in OpenCV 3.0
find_package(OpenCV REQUIRED core imgproc calib3d )
if( OpenCV_VERSION_MAJOR VERSION_LESS "3.0.0" )
  find_package(OpenCV REQUIRED highgui )
else()
  find_package(OpenCV REQUIRED imgcodecs )
endif()

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

fips_include_directories(${OpenCV_INCLUDE_DIRS}
                    include lib )

#
# libapriltags
#
#file(GLOB HEADER_FILES ${CMAKE_SOURCE_DIR}/include/Apriltags/*)

## Configurable options
option( USE_OPENMP "Use OpenMP" ON )
if( USE_OPENMP )

  if(APPLE)
    ## These are the required "hints" for libomp installed from Homebrew
    set(OpenMP_CXX_FLAGS "-Xpreprocessor -fopenmp -I/usr/local/opt/libomp/include")
    set(OpenMP_CXX_LIB_NAMES "omp")
    set(OpenMP_omp_LIBRARY /usr/local/opt/libomp/lib/libomp.dylib)
  endif()

  find_package(OpenMP)
  if(OpenMP_CXX_FOUND)
    add_definitions( -DUSE_OPENMP )
    message( STATUS "Found OpenMP, enabling" )
  endif()
endif()


fips_begin_module( apriltags )
  file(GLOB SOURCE_FILES "lib/*/*.cc" "lib/*.cc" )

  fips_files( ${SOURCE_FILES})
  fips_libs( ${OpenCV_LIBS} )

  if(OpenMP_CXX_FOUND)
    fips_libs(OpenMP::OpenMP_CXX)
  endif()

fips_end_module()


if( NOT FIPS_IMPORT )
  #
  ## Performance and Unit testing
  #
  # Location for test images used by unit and performance tests
  #
  set( TEST_DATA_DIR ${CMAKE_SOURCE_DIR}/test/data )
  add_definitions( -DTEST_DATA_DIR="${TEST_DATA_DIR}" )

  option( BUILD_PERF_TESTS "Build performance tests" ON )

  if( BUILD_PERF_TESTS )
    add_subdirectory( test/perf )
  endif()

  if( FIPS_UNITTESTS )
    add_subdirectory( test/unit )
  endif()

  fips_finish()
endif()
