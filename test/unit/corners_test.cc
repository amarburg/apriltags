// An empty test as a placeholder

#include "gtest/gtest.h"

#include <opencv2/core.hpp>
using namespace cv;

#include "AprilTags/Corners.h"
#include "AprilTags/TagFamily.h"
#include "AprilTags/Tag36h11.h"
using namespace AprilTags;

TEST( CornersTest, MakeTagMat ) {
  const int WhichTag = 34;
  const TagCodes &code( AprilTags::tagCodes36h11 );

  const int edge = (int)std::sqrt((float)code.bits);

  Mat tag( Corners::makeTagMat( code.codes[ WhichTag ], edge ));

  EXPECT_EQ( 6, edge );
  EXPECT_EQ( edge+4, tag.rows );
  EXPECT_EQ( edge+4, tag.cols );

  Mat huge;
  const double scale = 10.0;
  cv::resize( 255*tag, huge, Size(), scale, scale, INTER_NEAREST );

  imwrite("/tmp/tag.jpg", huge);
}

TEST( CornersTest, MakeCornerMat ) {
  const int WhichTag = 34;
  const TagCodes &code( AprilTags::tagCodes36h11 );

  const int edge = (int)std::sqrt((float)code.bits);
  EXPECT_EQ( 6, edge );

  Mat corners( Corners::makeCornerMat( code.codes[ WhichTag ], edge ));

  EXPECT_EQ( edge+3, corners.rows );
  EXPECT_EQ( edge+3, corners.cols );

  Mat out( Corners::drawCornerMat( corners ));
  const double scale = 10.0;
  Mat huge;
  cv::resize( 255*out, huge, Size(), scale, scale, INTER_NEAREST );

  imwrite("/tmp/corners.jpg", huge);


}