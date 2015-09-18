#ifndef __CORNERS_H__
#define __CORNERS_H__

#include <opencv2/core.hpp>

// FWIW, I think Mat is too big of a hammer for just
// storing the corners, but it gets the process
// going.

#include "TagFamily.h"

namespace AprilTags {

  enum CornerTypeMasks {
    CORNER_EDGE = 0x10,
    CORNER_IBC  = 0x20,    // Inner Black Corner (three black, one white)
    CORNER_IWC  = 0x40,    // Inner White Corner (three white, one black)
    CORNER_CHECK = 0x80    // Checkerboard corner
  };

  static const unsigned char CORNER_DETECTABLE = (CORNER_IBC | CORNER_IWC | CORNER_CHECK);

  inline bool detectable( unsigned char corner ) { return (corner & CORNER_DETECTABLE) != 0; }

  namespace Corners {

     cv::Mat makeTagMat( const TagCodes &family, int which, int blackBorder = 1, int whiteBoard = 1 );

     cv::Mat makeCornerMat( const Code_t code, int dim, int blackBorder = 1 );
     cv::Mat makeCornerMat( const TagCodes &family, int which, int blackBorder = 1 );

     cv::Mat drawCornerMat( const cv::Mat &corners );

  };


}

#endif
