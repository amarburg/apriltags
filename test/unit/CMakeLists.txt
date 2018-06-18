
set( UNIT_TEST_FILES
    corner_detector_test.cc
    corners_test.cc
    tag_array_test.cc )
    # subtag_detector_test.cc
    # tag_detector_test.cc )

gtest_begin(apriltags)

  fips_files( ${UNIT_TEST_FILES} )

  fips_deps(apriltags)

  fips_include_directories( ${CMAKE_SOURCE_DIR}/test/data )

gtest_end()
