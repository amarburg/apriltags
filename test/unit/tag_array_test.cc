// An empty test as a placeholder

#include "gtest/gtest.h"

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
using namespace cv;

#include "AprilTags/TagArray.h"
#include "AprilTags/CornerArray.h"
using namespace AprilTags;

#include "test_data.h"

const int WhichTag = 143;


// Tests the code which makes
TEST( TagArrayTest, Add ) {
  TagArray array( whichCode );

  EXPECT_EQ( 1, array.add( cv::Point2f( 0, 0 ), WhichTag ) );
  EXPECT_EQ( 2, array.add( cv::Point2f( 2, 2 ), WhichTag, M_PI/4.0 ) );
  EXPECT_EQ( 3, array.add( cv::Point2f( 4,4 ), WhichTag, M_PI/2.0 ) );

  Mat img;
  array.draw( img, cv::Size( 200, 200 ));

  imwrite( "/tmp/tag_array_test.png", img);
}

TEST( TagArrayTest, ConvertToCorners ) {
  TagArray array( whichCode );

  EXPECT_EQ( 1, array.add( cv::Point2f( 0, 0 ), WhichTag ) );
  EXPECT_EQ( 2, array.add( cv::Point2f( 2, 2 ), WhichTag, M_PI/4.0 ) );
  EXPECT_EQ( 3, array.add( cv::Point2f( 4,4 ), WhichTag, M_PI/2.0 ) );

  CornerArray corners( array.corners() );

  Mat img;
  corners.draw( img, cv::Size( 200,200 ));
  imwrite( "/tmp/corner_array_test.png", img );
}
