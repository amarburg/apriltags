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

struct BenchmarkData {
public:
  BenchmarkData( void )
    : duration( 0 ), numberOfTags( 0 ) {;}
  unsigned int duration, numberOfTags;
};

// This isn't a unit test per se, but tests that the tag TagDetector
// Is in fact still working against test data.
TEST( TagDetectorPerf, DefaultConfiguration ) {

  const unsigned int Repetitions = 5;
  vector<BenchmarkData> data( Repetitions );

  Mat inputImage( load36H11GreyscaleImage() );

  for( unsigned int rep = 0; rep < Repetitions; ++rep ) {
    TagDetector detector( tagCodes36h11 );

    int64 t = cv::getTickCount();
    std::vector<TagDetection> tags = detector.extractTags( inputImage );
    unsigned int ms = round( (double)(cv::getTickCount() - t) / getTickFrequency() * 1000.0 );

    data[rep].duration = ms;
    data[rep].numberOfTags = tags.size();

    validate_36h11_tags( tags );
  }

  float mean = 0, var = 0;
  float meanTags = 0;
  for( unsigned int rep = 0; rep < Repetitions; ++rep ) {
    mean += data[rep].duration;
    meanTags += data[rep].numberOfTags;
  }
  mean /= Repetitions;
  meanTags /= Repetitions;

  for( unsigned int rep = 0; rep < Repetitions; ++rep ) { var += powf(data[rep].duration - mean, 2); }
  var /= Repetitions;

  cout << "For " << Repetitions << " reps on a " << inputImage.size().width << " x " << inputImage.size().height << " image, with an average of " << meanTags << " tags per image." << endl;
  cout << "Mean: " << mean << " ms, stddev: " << sqrt(var) << endl;

}
