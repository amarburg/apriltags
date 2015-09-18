#ifndef __SUBTAG_DETECTION_H__
#define __SUBTAG_DETECTION_H__

#include <opencv2/core.hpp>

#include "AprilTags/TagFamily.h"
#include "AprilTags/Corners.h"

namespace AprilTags {

  struct CornerDetection {
  public:
    CornerDetection( unsigned char corner_ )
      : corner( corner_ ), detected( false )
    {;}


    unsigned char corner;
    bool detected;

    bool detectable( void ) const { return (corner & CORNER_DETECTABLE) != 0; }


    cv::Point2f inImage, inTag;

  };

  class SubtagDetection {
  public:
    SubtagDetection( const TagCodes &code_, const TagDetection &detection );

    Code_t code;
    int id;

    // Was this tag detected at all?
    // false == systematic failure, for example the tag was too small
    //          in the image
    bool good;

    std::vector< CornerDetection > corners;

  protected:

  private:
    SubtagDetection( void ) {;}

  };

}


#endif
