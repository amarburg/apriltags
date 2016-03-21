// An empty test as a placeholder

#include "gtest/gtest.h"

#include <opencv2/highgui/highgui.hpp>
using namespace cv;

#include "AprilTags/CornerDetector.h"
using namespace AprilTags;

#include "../test_common.h"
#include "test_data.h"

static Mat load36H11GreyscaleImage( void )
{
  Mat inputImage( imread(  TEST_36H11_CLOSE_JPG, CV_LOAD_IMAGE_GRAYSCALE ));
  EXPECT_FALSE( inputImage.empty() );
  return inputImage;
}

static Mat load36H11ObliqueGreyscaleImage( void )
{
  Mat inputImage( imread(  TEST_36H11_CLOSE_OBLIQUE_JPG, CV_LOAD_IMAGE_GRAYSCALE ));
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

  Mat inputImage( load36H11GreyscaleImage() );

  CornerDetector detector;
  detector.saveDebugImages( true );

  CornerDetectionArray detections( detector.detect( inputImage, corners ) );

  EXPECT_EQ( 0, detections.size() );

  EXPECT_EQ( detector.debugImage(DetectorBase::OriginalImage).size(), inputImage.size() );
  EXPECT_EQ( detector.debugImage(DetectorBase::BlurredImage).size(), inputImage.size() );
  EXPECT_EQ( detector.debugImage(DetectorBase::MagnitudeImage).size(), inputImage.size() );
  EXPECT_EQ( detector.debugImage(DetectorBase::LineSegmentsImage).size(), inputImage.size() );
  EXPECT_EQ( detector.debugImage(CornerDetector::IntersectionImage).size(), inputImage.size() );
  EXPECT_EQ( detector.debugImage(CornerDetector::TriangleImage).size(), inputImage.size() );


  // EXPECT_EQ( detector.debugImage(TagDetector::QuadImage).size(), inputImage.size() );

  write32FC1("/tmp/corner_original.jpg", detector.debugImage(DetectorBase::OriginalImage) );
  write32FC1("/tmp/corner_low_pass_image.jpg", detector.debugImage(DetectorBase::BlurredImage) );
  write32FC1("/tmp/corner_magnitude.jpg", detector.debugImage(DetectorBase::MagnitudeImage) );
  write32FC3("/tmp/corner_line_segment_image.jpg", detector.debugImage(DetectorBase::LineSegmentsImage) );
  write32FC3("/tmp/corner_intersections.jpg", detector.debugImage(CornerDetector::IntersectionImage) );
  write32FC3("/tmp/corner_triangles.jpg", detector.debugImage(CornerDetector::TriangleImage) );
  write32FC3("/tmp/corner_mesh.jpg", detector.debugImage(CornerDetector::MeshImage) );


  // write32FC1("/tmp/quad_image.jpg", detector.debugImage(TagDetector::QuadImage) );
}


TEST( CornerDetectorTest, ObliqueImage ) {
  const TagCodes whichCode = tagCodes36h11;

  // Load the TagArray for this board.
  TagArray array( AprilTags::TestData::GroundTruthArray() );

  EXPECT_EQ( array.size(), AprilTags::TestData::t36h11ArraySize );
  CornerArray corners( array.corners() );

  Mat inputImage( load36H11ObliqueGreyscaleImage() );

  CornerDetector detector;
  detector.saveDebugImages( true );

  CornerDetectionArray detections( detector.detect( inputImage, corners ) );

  EXPECT_EQ( 0, detections.size() );

  // EXPECT_EQ( detector.debugImage(TagDetector::QuadImage).size(), inputImage.size() );

  write32FC1("/tmp/corner_oblique_original.jpg", detector.debugImage(DetectorBase::OriginalImage) );
  write32FC1("/tmp/corner_oblique_low_pass_image.jpg", detector.debugImage(DetectorBase::BlurredImage) );
  write32FC1("/tmp/corner_oblique_magnitude.jpg", detector.debugImage(DetectorBase::MagnitudeImage) );
  write32FC3("/tmp/corner_oblique_line_segment_image.jpg", detector.debugImage(DetectorBase::LineSegmentsImage) );
  write32FC3("/tmp/corner_oblique_intersections.jpg", detector.debugImage(CornerDetector::IntersectionImage) );
  write32FC3("/tmp/corner_oblique_triangles.jpg", detector.debugImage(CornerDetector::TriangleImage) );
  write32FC3("/tmp/corner_oblique_mesh.jpg", detector.debugImage(CornerDetector::MeshImage) );

  // write32FC1("/tmp/quad_image.jpg", detector.debugImage(TagDetector::QuadImage) );

//exit(0);
}
