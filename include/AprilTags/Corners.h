#ifndef __CORNERS_H__
#define __CORNERS_H__

#include <opencv2/core.hpp>

// FWIW, I think Mat is too big of a hammer for just
// storing the corners, but it gets the process
// going.

namespace AprilTags {

  class Corners {
    public:
      Corners( const cv::Mat &corners );


    static cv::Mat makeTagMat( unsigned long long code, int dim, int blackBorder = 1, int whiteBoard = 1 );
    static cv::Mat makeCornerMat( unsigned long long code, int dim, int blackBorder = 1 );

  private:
    Corners( void ) {;}

    cv::Mat _corners;

  };


}

#endif
