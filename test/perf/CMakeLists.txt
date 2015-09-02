enable_testing()
find_package( GTest REQUIRED )

include_directories( ${GTEST_INCLUDE_DIRS}
                     ${CMAKE_SOURCE_DIR}/test/data )

add_executable( tag_detector_perf
               tag_detector_perf.cc )

target_link_libraries( tag_detector_perf
         ${GTEST_BOTH_LIBRARIES}
         ${OpenCV_LIBS}
         apriltags )

# Add a custom target "perf" which runs any performance tests
add_custom_target( perf ${CMAKE_CURRENT_BINARY_DIR}/tag_detector_perf )
