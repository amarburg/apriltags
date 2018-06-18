
gtest_begin(apriltags_perf)

  fips_files( tag_detector_perf.cc )

  fips_deps(apriltags)

  fips_include_directories( ${CMAKE_SOURCE_DIR}/test/data )

gtest_end()
