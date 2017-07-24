// An empty test as a placeholder

#include "gtest/gtest.h"

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
using namespace cv;

#include "AprilTags/Corners.h"
using namespace AprilTags;

#include "test_data.h"

const int WhichTag = 143;


// Tests the code which makes
TEST( CornersTest, MakeTagMat ) {

  const int edge = whichCode.dim;

  Mat tag( Corners::makeTagMat( whichCode, WhichTag ));

  EXPECT_EQ( 6, edge );
  EXPECT_EQ( edge+4, tag.rows );
  EXPECT_EQ( edge+4, tag.cols );

  imwrite("tag.jpg", Corners::drawTagMat( tag, Size(50,50)));

  int whichGroundTruth = Tag36H11GroundTruths.find( WhichTag );
  ASSERT_GE( whichGroundTruth, 0 );
  const TagGroundTruth &gt( Tag36H11GroundTruths[whichGroundTruth] );

  EXPECT_EQ( gt.numElement(), tag.size().area() );

  for( Point p(0,0); p.y < tag.rows; ++p.y )
    for( p.x = 0; p.x < tag.cols; ++p.x )
      EXPECT_EQ( gt.element( p.y*tag.cols + p.x ), tag.at<unsigned char>(p) );

}

TEST( CornersTest, MakeCornerMat ) {

  const int edge = whichCode.dim;
  EXPECT_EQ( 6, edge );

  Mat corners( Corners::makeCornerMat( whichCode, WhichTag ));

  EXPECT_EQ( edge+3, corners.rows );
  EXPECT_EQ( edge+3, corners.cols );

  imwrite("corners.jpg", Corners::drawCornerMat( corners, Size(50, 50 ) ));

  int whichGroundTruth = Tag36H11GroundTruths.find( WhichTag );
  ASSERT_GE( whichGroundTruth, 0 );
  const TagGroundTruth &gt( Tag36H11GroundTruths[whichGroundTruth] );

  EXPECT_EQ( gt.numCorners(), corners.size().area() );

  for( Point p(0,0); p.y < corners.rows; ++p.y )
    for( p.x = 0; p.x < corners.cols; ++p.x )
      EXPECT_EQ( gt.corner( p.y*corners.cols + p.x ), corners.at<unsigned char>(p) );

}

const unsigned char t[] = { 0x13, 0x16, 0x1C, 0x19 };
const unsigned int tcount = sizeof(t)/(4*sizeof(t[0]));

TEST( CornersTest, RotateTest )
{

  for( unsigned int i = 0; i < tcount; ++i ) {
    int offset = i*4;
    EXPECT_EQ( Corners::cornerLUT( t[offset] ), t[offset] );
    for( unsigned int j = 0; j < 4; ++j ) {
      unsigned char r = Corners::rotate( t[offset], j );
      EXPECT_EQ( r, t[offset+j] );
      EXPECT_EQ( Corners::cornerLUT( r ), r );
    }
  }
}

TEST( CornersTest, SpinTest )
{
  for( unsigned int i = 0; i < tcount; ++i ) {
    int offset = i*4;
    for( unsigned int j = 0; j < 4; ++j ) {
      EXPECT_EQ( Corners::spinMatch( t[offset], t[offset+j]), j );
    }
  }

  EXPECT_EQ( Corners::spinMatch( t[0], 0x28 ), -1 );
  EXPECT_EQ( Corners::spinMatch( t[0], 0x4E ), -1 );
  EXPECT_EQ( Corners::spinMatch( t[0], 0x85 ), -1 );


}



TEST( CornersTest, Terminator )
{
  exit(0);
}
