// An empty test as a placeholder

#include "gtest/gtest.h"

#include <opencv2/highgui.hpp>
using namespace cv;

#include "AprilTags/TagDetector.h"
#include "AprilTags/Tag36h11.h"
using namespace AprilTags;

#include "test_data.h"

static Mat load36H11GreyscaleImage( void )
{
  Mat inputImage( imread(  TEST_36H11_GREYSCALE_JPG, CV_LOAD_IMAGE_GRAYSCALE ));
  EXPECT_FALSE( inputImage.empty() );
  return inputImage;
}

static void validate_36h11_tags( const std::vector<TagDetection> &tags )
{
  EXPECT_EQ( 80, tags.size() );
}

// This isn't a unit test per se, but tests that the tag TagDetector
// Is in fact still working against test data.
TEST( TagDetectorTest, DefaultConfiguration ) {
  TagDetector detector( tagCodes36h11 );

  Mat inputImage( load36H11GreyscaleImage() );

  std::vector<TagDetection> tags = detector.extractTags( inputImage );

  validate_36h11_tags( tags );
}

#ifdef BUILD_DEBUG_TAG_DETECTOR

TEST( DebugTagDetectorTest, DefaultConfiguration ) {
  DebugTagDetector detector( tagCodes36h11 );

  Mat inputImage( load36H11GreyscaleImage() );

  std::vector<TagDetection> tags = detector.extractTags( inputImage );

  EXPECT_EQ( detector.originalImage.size(), inputImage.size() );
  EXPECT_EQ( detector.gaussianLowPassImage.size(), inputImage.size() );
  EXPECT_EQ( detector.lineSegmentsImage.size(), inputImage.size() );
  EXPECT_EQ( detector.quadImage.size(), inputImage.size() );

  // imwrite("/tmp/original.jpg", detector.originalImage );
  // imwrite("/tmp/low_pass_image.jpg", detector.gaussianLowPassImage );
  // imwrite("/tmp/line_segments_image.jpg", detector.lineSegmentsImage );
  // imwrite("/tmp/quad_image.jpg", detector.quadImage );

  validate_36h11_tags( tags );
}

#endif
