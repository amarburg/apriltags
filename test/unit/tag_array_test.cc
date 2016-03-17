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
  array.draw( img, cv::Size( 100, 100 ));

  imwrite( "/tmp/tag_array_test.png", img);
}

TEST( TagArrayTest, ConvertToCorners ) {
  TagArray array( whichCode );

  EXPECT_EQ( 1, array.add( cv::Point2f( 0, 0 ), WhichTag ) );
  EXPECT_EQ( 2, array.add( cv::Point2f( 4,4 ), WhichTag, M_PI/2.0 ) );

  CornerArray corners( array.corners() );

  Mat img;
  corners.draw( img, cv::Size( 100,100 ));
  imwrite( "/tmp/corner_array_test.png", img );
}


//
//   const int edge = whichCode.dim;
//
//   Mat tag( Corners::makeTagMat( whichCode, WhichTag ));
//
//   EXPECT_EQ( 6, edge );
//   EXPECT_EQ( edge+4, tag.rows );
//   EXPECT_EQ( edge+4, tag.cols );
//
//   imwrite("tag.jpg", Corners::drawTagMat( tag, Size(50,50)));
//
//   int whichGroundTruth = Tag36H11GroundTruths.find( WhichTag );
//   ASSERT_GE( whichGroundTruth, 0 );
//   const TagGroundTruth &gt( Tag36H11GroundTruths[whichGroundTruth] );
//
//   EXPECT_EQ( gt.numElement(), tag.size().area() );
//
//   for( Point p(0,0); p.y < tag.rows; ++p.y )
//     for( p.x = 0; p.x < tag.cols; ++p.x )
//       EXPECT_EQ( gt.element( p.y*tag.cols + p.x ), tag.at<unsigned char>(p) );
//
// }
//
// TEST( CornersTest, MakeCornerMat ) {
//
//   const int edge = whichCode.dim;
//   EXPECT_EQ( 6, edge );
//
//   Mat corners( Corners::makeCornerMat( whichCode, WhichTag ));
//
//   EXPECT_EQ( edge+3, corners.rows );
//   EXPECT_EQ( edge+3, corners.cols );
//
//   imwrite("corners.jpg", Corners::drawCornerMat( corners, Size(50, 50 ) ));
//
//   int whichGroundTruth = Tag36H11GroundTruths.find( WhichTag );
//   ASSERT_GE( whichGroundTruth, 0 );
//   const TagGroundTruth &gt( Tag36H11GroundTruths[whichGroundTruth] );
//
//   EXPECT_EQ( gt.numCorners(), corners.size().area() );
//
//   for( Point p(0,0); p.y < corners.rows; ++p.y )
//     for( p.x = 0; p.x < corners.cols; ++p.x )
//       EXPECT_EQ( gt.corner( p.y*corners.cols + p.x ), corners.at<unsigned char>(p) );
//
// }
