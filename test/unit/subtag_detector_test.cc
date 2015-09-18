// An empty test as a placeholder

#include "gtest/gtest.h"

#include <opencv2/highgui.hpp>
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

TEST( SubtagDetectorTest, DefaultConfiguration ) {
const TagCodes whichCode = tagCodes36h11;
  TagDetector detector( whichCode );
  Mat inputImage( load36H11GreyscaleImage() );
  std::vector<TagDetection> tags = detector.extractTags( inputImage );

#ifdef BUILD_DEBUG_TAG_DETECTOR
  DebugSubtagDetector subtagDetector( whichCode );
#else
  SubtagDetector subtagDetector( whichCode );
#endif

  unsigned int whichId = 143;
  // Find tag 143 in the vector of detections
  int whichDetection = -1;
  for( unsigned int i = 0; i < tags.size(); ++i ) {
    if( tags[i].id == whichId ) {
      whichDetection = i;
      break;
    }
  }

  EXPECT_GE( whichDetection, 0 );


  SubtagDetection subtag( subtagDetector.detectTagSubstructure( inputImage, tags[whichDetection] ) );

#ifdef BUILD_DEBUG_TAG_DETECTOR
  // Produce debug outputs
  imwrite( "predictedCorners.jpg", subtagDetector.predictedCorners );
  imwrite( "refinedCorners.jpg", subtagDetector.refinedCorners );

  const cv::Size sz( 50, 50 );
  imwrite("expectedTag.jpg", Corners::drawTagMat( whichCode, tags[whichDetection].id,
                                                  sz ));
  imwrite("expectedCorners.jpg", Corners::drawCornerMat( whichCode, tags[whichDetection].id, sz ) );

#endif

}
