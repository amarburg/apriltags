enable_testing()
find_package( GTest REQUIRED )

include_directories( ${GTEST_INCLUDE_DIRS} )

# add_executable( apriltags_perf
#               empty_test.cc )
