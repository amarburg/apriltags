include_directories( ${CONAN_INCLUDE_DIRS}
                     ${CMAKE_SOURCE_DIR}/test/data )

add_executable( apriltags_unit_test
              tag_detector_test.cc )

target_link_libraries( apriltags_unit_test
          ${CONAN_LIBS}
          ${OpenCV_LIBS}
          apriltags )

# Don't use ctest, instead just execture the Google test binaries
add_custom_target( test ${CMAKE_CURRENT_BINARY_DIR}/apriltags_unit_test )
