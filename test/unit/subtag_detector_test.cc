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

  unsigned int whichDetection = 5;

#ifdef BUILD_DEBUG_TAG_DETECTOR
  DebugSubtagDetector subtagDetector( whichCode );
#else
  SubtagDetector subtagDetector( whichCode );
#endif

  SubtagDetection subtag( subtagDetector.detectTagSubstructure( inputImage, tags[whichDetection] ) );


#ifdef BUILD_DEBUG_TAG_DETECTOR
  imwrite( "predictedCorners.jpg", subtagDetector.predictedCorners );
  imwrite( "refinedCorners.jpg", subtagDetector.refinedCorners );

    Mat tag( Corners::makeTagMat( whichCode, tags[whichDetection].id ));
    Mat huge;
    const double scale = 10.0;
    cv::resize( 255*tag, huge, Size(), scale, scale, INTER_NEAREST );
    imwrite("expectedTag.jpg", huge);

    Mat corners( Corners::makeCornerMat( whichCode, tags[whichDetection].id ));
    Mat out( Corners::drawCornerMat( corners ));
    cv::resize( 255*out, huge, Size(), scale, scale, INTER_NEAREST );

    imwrite("expectedCorners.jpg", huge);

#endif

}
