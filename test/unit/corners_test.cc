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
  EXPECT_EQ( 10, tag.rows );
  EXPECT_EQ( 10, tag.cols );

  Mat huge;
  const double scale = 10.0;
  cv::resize( 255*tag, huge, Size(), scale, scale );

  imwrite("/tmp/tag.jpg", huge);
}
