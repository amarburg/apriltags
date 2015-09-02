enable_testing()
find_package( GTest REQUIRED )

include_directories( ${GTEST_INCLUDE_DIRS}
                     ${CMAKE_SOURCE_DIR}/test/data )

add_executable( apriltags_unit_test
              tag_detector_test.cc
              empty_test.cc )

target_link_libraries( apriltags_unit_test
          ${GTEST_BOTH_LIBRARIES}
          ${OpenCV_LIBS}
          apriltags )

add_test( NAME AprilTagsUnitTest
          COMMAND apriltags_unit_test )
