// An empty test as a placeholder

#include "gtest/gtest.h"

#include <opencv2/core.hpp>
using namespace cv;

#include "AprilTags/Corners.h"
#include "AprilTags/TagFamily.h"
#include "AprilTags/Tag36h11.h"
using namespace AprilTags;


const int WhichTag = 34;
const TagCodes &code( AprilTags::tagCodes36h11 );

// Tests the code which makes
TEST( CornersTest, MakeTagMat ) {

  const int edge = code.dim;

  Mat tag( Corners::makeTagMat( code, WhichTag ));

  EXPECT_EQ( 6, edge );
  EXPECT_EQ( edge+4, tag.rows );
  EXPECT_EQ( edge+4, tag.cols );

  Mat huge;
  const double scale = 10.0;
  cv::resize( 255*tag, huge, Size(), scale, scale, INTER_NEAREST );

  imwrite("/tmp/tag.jpg", huge);
}

TEST( CornersTest, MakeCornerMat ) {

  const int edge = code.dim;
  EXPECT_EQ( 6, edge );

  Mat corners( Corners::makeCornerMat( code, WhichTag ));

  EXPECT_EQ( edge+3, corners.rows );
  EXPECT_EQ( edge+3, corners.cols );

  Mat out( Corners::drawCornerMat( corners ));
  const double scale = 10.0;
  Mat huge;
  cv::resize( 255*out, huge, Size(), scale, scale, INTER_NEAREST );

  imwrite("/tmp/corners.jpg", huge);


}
