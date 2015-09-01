enable_testing()
find_package( GTest REQUIRED )

include_directories( ${GTEST_INCLUDE_DIRS} )

add_executable( apriltags_unit_test
              empty_test.cc )

target_link_libraries( apriltags_unit_test ${GTEST_BOTH_LIBRARIES})

add_test( NAME AprilTagsUnitTest
          COMMAND apriltags_unit_test )
