#ifndef __CORNERS_H__
#define __CORNERS_H__

#include <opencv2/core.hpp>

// FWIW, I think Mat is too big of a hammer for just
// storing the corners, but it gets the process
// going.

#include "TagFamily.h"

namespace AprilTags {

  class Corners {
    public:
      Corners( const cv::Mat &corners );

    static cv::Mat makeTagMat( const TagCodes &family, int which, int blackBorder = 1, int whiteBoard = 1 );

    static cv::Mat makeCornerMat( const Code_t code, int dim, int blackBorder = 1 );
    static cv::Mat makeCornerMat( const TagCodes &family, int which, int blackBorder = 1 );

    static cv::Mat drawCornerMat( const cv::Mat &corners );

  private:
    Corners( void ) {;}

    cv::Mat _corners;

  };


}

#endif
