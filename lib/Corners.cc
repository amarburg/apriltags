
#include "AprilTags/Corners.h"

using namespace cv;

namespace AprilTags {

  Corners::Corners( const Mat &corners )
    : _corners( corners )
  {;}


//===================================================================

  Mat Corners::makeTagMat( unsigned long long code, int dim, int blackBorder, int whiteBorder )
  {

    int edge = dim + 2*(blackBorder + whiteBorder);

    const int WHITE = 1, BLACK = 0;

    // To avoid going nuts, draw the actual tag with border ...
    // with a 1-pixel white border around the outside
    Mat tag( edge, edge, CV_8UC1 );

    int bwBorder = blackBorder + whiteBorder;

    for( Point p(0,0); p.y < edge; ++p.y ) {
      for( p.x = 0; p.x < edge; ++p.x ) {

        if( (p.y < whiteBorder) || (p.y >=  edge - whiteBorder) ||
            (p.x < whiteBorder) || (p.x >=  edge - whiteBorder) ) {
          tag.at<unsigned char>(p) = WHITE;
        } else if( (p.y < bwBorder) || (p.y >= edge - bwBorder) ||
                   (p.x < bwBorder) || (p.x >= edge - bwBorder) ) {
          tag.at<unsigned char>(p) = BLACK;
        } else {
          int offset = p.x-bwBorder + dim*(p.y-bwBorder);
          unsigned long long mask = 1 << offset;

          if( code & mask )
            tag.at<unsigned char>(p) = WHITE;
          else
            tag.at<unsigned char>(p) = BLACK;

        }

      }
    }

  return tag;
  }


  cv::Mat Corners::makeCornerMat( unsigned long long code, int dim, int blackBorder )
  {
    Mat tag( makeTagMat(code, dim, blackBorder, 1) );
    int edge = tag.rows + 1;

    Mat corners( edge, edge, CV_8UC1 );

    return corners;
  }

}
