<<<<<<< HEAD
include_directories( ${CONAN_INCLUDE_DIRS}
=======
include_directories( ${GTEST_INCLUDE_DIRS}
                     ${CONAN_INCLUDE_DIRS}
>>>>>>> ae0dfeff2af877559d7f3fa1b83a3c593f853a5d
                     ${CMAKE_SOURCE_DIR}/test/data )

add_executable( tag_detector_perf
               tag_detector_perf.cc )

target_link_libraries( tag_detector_perf
         ${CONAN_LIBS}
         ${OpenCV_LIBS}
         ${CONAN_LIBS}
         apriltags )

# Add a custom target "perf" which runs any performance tests
add_custom_target( perf_test ${CMAKE_CURRENT_BINARY_DIR}/tag_detector_perf )
