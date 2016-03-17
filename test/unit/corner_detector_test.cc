// An empty test as a placeholder

#include "gtest/gtest.h"

#include <opencv2/highgui/highgui.hpp>
using namespace cv;

#include "AprilTags/CornerDetector.h"
using namespace AprilTags;

#include "test_data.h"

static Mat load36H11GreyscaleImage( void )
{
  Mat inputImage( imread(  TEST_36H11_CLOSE_JPG, CV_LOAD_IMAGE_GRAYSCALE ));
  EXPECT_FALSE( inputImage.empty() );
  return inputImage;
}

// Inefficient to detect both here and in the tag detector tests
TEST( CornerDetectorTest, DefaultConfiguration ) {
  const TagCodes whichCode = tagCodes36h11;

  // Load the TagArray for this board.
  TagArray array( AprilTags::TestData::GroundTruthArray() );

  EXPECT_EQ( array.size(), AprilTags::TestData::t36h11ArraySize );

  Mat img;
  array.draw( img, cv::Size(50,50) );
  imwrite("/tmp/corner_test_ground_truth.jpg", img );
  CornerArray corners( array.corners() );

  Mat input( load36H11GreyscaleImage() );

  CornerDetector detector;

  CornerDetectionArray detections( detector.detect( input, corners ) );

  EXPECT_EQ( 0, detections.size() );
}
