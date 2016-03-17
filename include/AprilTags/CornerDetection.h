#ifndef __APRILTAGS_SUBTAG_DETECTION_H__
#define __APRILTAGS_SUBTAG_DETECTION_H__

#include <opencv2/core/core.hpp>

#include "AprilTags/TagFamily.h"
#include "AprilTags/Corners.h"

namespace AprilTags {

  struct CornerDetection {
  public:
    CornerDetection( unsigned char corner_ )
      : corner( corner_ )
    {;}

    unsigned char corner;

    bool detectable( void ) const { return (corner & CORNER_DETECTABLE) != 0; }

    cv::Point2f inImage, inTag;

    static cv::Point2f InImageTx( const CornerDetection &det ) { return det.inImage; }
  };

}


#endif
