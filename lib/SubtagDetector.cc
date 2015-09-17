

#include "AprilTags/SubtagDetector.h"


namespace AprilTags {

using namespace cv;

SubtagDetector::SubtagDetector( const TagCodes &tagCodes )
: _code( tagCodes )
{;}

SubtagDetection
SubtagDetector::detectTagSubstructure(const Mat& image, const TagDetection &detection )
{

  // Identify the ROI in the image based on the detection.
  // Look for corners

  return SubtagDetection();
}


}
