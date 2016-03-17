find_package( GTest REQUIRED )

include_directories( ${GTEST_INCLUDE_DIRS}
                     ${CMAKE_SOURCE_DIR}/test/data )

file(GLOB UNIT_TEST_FILES ${CMAKE_CURRENT_SOURCE_DIR}/*.cc)

add_executable( apriltags_unit_test
               ${UNIT_TEST_FILES} )

target_link_libraries( apriltags_unit_test
          ${GTEST_BOTH_LIBRARIES}
          ${OpenCV_LIBS}
          apriltags )

# Don't use ctest, instead just exectute the Google test binaries
add_custom_target( test ${CMAKE_CURRENT_BINARY_DIR}/apriltags_unit_test
                    DEPENDS apriltags_unit_test )
