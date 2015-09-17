

#ifndef __SUBTAG_DETECTOR__
#define __SUBTAG_DETECTOR__

#include <opencv2/core.hpp>

#include "AprilTags/SubtagDetection.h"
#include "AprilTags/TagDetection.h"

namespace AprilTags {


  // This detector uses the corners to find substructure within an already
  // detected tag ...
  // CornerDetector goes from nothing (raw image) to find tags by detecting
  // corners.
  class SubtagDetector {
  public:

    SubtagDetector( const TagCodes &tagCodes );

    SubtagDetection detectTagSubstructure(const cv::Mat& image, const TagDetection &detection );

  protected:

    const TagCodes &_code;


  };

}

#endif
