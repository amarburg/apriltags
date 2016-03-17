// An empty test as a placeholder

#include "gtest/gtest.h"

#include <opencv2/highgui/highgui.hpp>
using namespace cv;

#include "AprilTags/TagDetector.h"
#include "AprilTags/SubtagDetector.h"
#include "AprilTags/Tag36h11.h"
using namespace AprilTags;

#include "test_data.h"

static Mat load36H11GreyscaleImage( void )
{
  Mat inputImage( imread(  TEST_36H11_GREYSCALE_JPG, CV_LOAD_IMAGE_GRAYSCALE ));
  EXPECT_FALSE( inputImage.empty() );
  return inputImage;
}

static Mat load36H11ObliqueGreyscaleImage( void )
{
  Mat inputImage( imread(  TEST_36H11_OBLIQUE_GREYSCALE_JPG, CV_LOAD_IMAGE_GRAYSCALE ));
  EXPECT_FALSE( inputImage.empty() );
  return inputImage;
}


// Inefficient to detect both here and in the tag detector tests
TEST( SubtagDetectorTest, DefaultConfiguration ) {
const TagCodes whichCode = tagCodes36h11;
  TagDetector detector( whichCode );
  Mat inputImage( load36H11GreyscaleImage() );
  std::vector<TagDetection> tags = detector.extractTags( inputImage );

  SubtagDetector subtagDetector( whichCode );
  subtagDetector.saveDebugImages( true );

  int whichId = 143;
  // Find tag 143 in the vector of detections
  int whichDetection = -1;
  for( unsigned int i = 0; i < tags.size(); ++i ) {
    if( tags[i].id == whichId ) {
      whichDetection = i;
      break;
    }
  }
  ASSERT_GE( whichDetection, 0 );

  vector<CornerDetection> corners( subtagDetector.detectTagSubstructure( inputImage, tags[whichDetection] ) );

  // Produce debug outputs
  imwrite( "predictedCorners.jpg", subtagDetector.debugImage( SubtagDetector::PredictedCorners ) );
  imwrite( "refinedCorners.jpg", subtagDetector.debugImage( SubtagDetector::RefinedCorners ) );

  const cv::Size sz( 50, 50 );
  imwrite("expectedTag.jpg", Corners::drawTagMat( whichCode, tags[whichDetection].id, sz ) );
  imwrite("expectedCorners.jpg", Corners::drawCornerMat( whichCode, tags[whichDetection].id, sz ) );

}


TEST( SubtagDetectorTest, ObliqueImage ) {
  TagDetector detector( tagCodes36h11 );
  Mat inputImage( load36H11ObliqueGreyscaleImage() );
  std::vector<TagDetection> tags = detector.extractTags( inputImage );

  SubtagDetector subtagDetector( whichCode );
  subtagDetector.saveDebugImages( true );

    int whichId = 90;

    int whichDetection = -1;
    for( unsigned int i = 0; i < tags.size(); ++i ) {
      if( tags[i].id == whichId ) {
        whichDetection = i;
        break;
      }
    }
    ASSERT_GE( whichDetection, 0 );

    vector<CornerDetection> corners( subtagDetector.detectTagSubstructure( inputImage, tags[whichDetection] ) );

    // Produce debug outputs
    imwrite( "oblique_predictedCorners.jpg", subtagDetector.debugImage( SubtagDetector::PredictedCorners) );
    imwrite( "oblique_refinedCorners.jpg", subtagDetector.debugImage( SubtagDetector::RefinedCorners) );

    const cv::Size sz( 50, 50 );
    imwrite("oblique_expectedTag.jpg", Corners::drawTagMat( whichCode, tags[whichDetection].id, sz ));
    imwrite("oblique_expectedCorners.jpg", Corners::drawCornerMat( whichCode, tags[whichDetection].id, sz ) );


}
