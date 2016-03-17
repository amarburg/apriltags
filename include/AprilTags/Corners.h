#ifndef __CORNERS_H__
#define __CORNERS_H__

#include <opencv2/core/core.hpp>

// FWIW, I think Mat is too big of a hammer for just
// storing the corners, but it gets the process
// going.

#include "TagFamily.h"

namespace AprilTags {

  using cv::Size;

  enum CornerTypeMasks {
    CORNER_EDGE = 0x10,
    CORNER_IBC  = 0x20,    // Inner Black Corner (three black, one white)
    CORNER_IWC  = 0x40,    // Inner White Corner (three white, one black)
    CORNER_CHECK = 0x80    // Checkerboard corner
  };

  static const unsigned char CORNER_DETECTABLE = (CORNER_IBC | CORNER_IWC | CORNER_CHECK);

  inline bool detectable( unsigned char corner ) { return (corner & CORNER_DETECTABLE) != 0; }

  namespace Corners {

    static const int WHITE = 1, BLACK = 0;

     /* A TagMat is a 8UC1 OpenCV Mat which stores the binary pattern for a
      * given AprilTags code, including a fixed width black border (with a
      * width set by blackBorder in multiples of the size of one "bit" of
      * Apriltags data), and a fixed width white border beyond that (set
      * but whiteBorder).
      *
      * So, for example TagFamily36h11 creates tags with a 6 bit x 6 bit
      * payload.  If blackBoard = whiteBorder = 1, the resulting TagMat
      * will be 10 x 10.
      *
      * Values in the array are either WHITE or BLACK as above.
      */
     cv::Mat makeTagMat( const TagCodes &family, int which, int blackBorder = 1, int whiteBorder = 1 );

     /* A CornerMat is an 8UC1 OpenCV Mat which describes the
      * corners/interstices of a tag.  It is (N-1) x (N-1)
      * in size where N is the size of the TagMat (e.g. 10 for TagFamily36h11)
      *
      * Each element in the array is actually two bytefields.  The top four
      * bits describe the type of corner -- checkerboard, inside black
      * corner, inside white corner, or an edge (by CornerTypeMasks).
      * The lower four bits describe the four surrounding squares:
      *
      *   0 | 1
      *   --+--
      *   2 | 3
      *
      * As either BLACK or WHITE.  By definition, there are only
      * 2^4 = 16 possible combinations and the value of the lower four bits
      * determines the values of the upper four bits.  But it makes it
      * easier to mask out findable and unfindable corner types.
      */
     //cv::Mat makeCornerMat( const Code_t code, int dim, int blackBorder = 1, int whiteBorder = 1 );
     cv::Mat makeCornerMat( const TagCodes &family, int which, int blackBorder = 1, int whiteBorder = 1 );

     cv::Mat drawTagMat( const TagCodes &family, int which, const Size size = Size(0,0), int blackBorder = 1, int whiteBorder = 1 );
     cv::Mat drawTagMat( const cv::Mat &mat, const Size size = Size(0,0) );

     cv::Mat drawCornerMat( const TagCodes &family, int which, const Size size = Size(0,0) );
     cv::Mat drawCornerMat( const cv::Mat &corners, const Size size = Size(0,0) );

  };


}

#endif
