
#include "AprilTags/Corners.h"

using namespace cv;

namespace AprilTags {

  Corners::Corners( const Mat &corners )
    : _corners( corners )
  {;}


//===================================================================

  Mat Corners::makeTagMat( unsigned long long code, int dim, int blackBorder, int whiteBorder )
  {

    int bwBorder = blackBorder + whiteBorder;
    int edge = dim + 2*(bwBorder);

    const int WHITE = 1, BLACK = 0;

    // To avoid going nuts, draw the actual tag with border ...
    // with a 1-pixel white border around the outside
    Mat tag( edge, edge, CV_8UC1, Scalar(0) );

// printf("The code is : %016llx\n", code );

    for( Point p(0,0); p.y < edge; ++p.y ) {
      for( p.x = 0; p.x < edge; ++p.x ) {

        if( (p.y < whiteBorder) || (p.y >=  edge - whiteBorder) ||
            (p.x < whiteBorder) || (p.x >=  edge - whiteBorder) ) {
          tag.at<unsigned char>(p) = WHITE;
        } else if( (p.y < bwBorder) || (p.y >= edge - bwBorder) ||
                   (p.x < bwBorder) || (p.x >= edge - bwBorder) ) {
          tag.at<unsigned char>(p) = BLACK;
        } else {

          // In image coordinates, the MSB bit of the code is the upper left (Y = 0, X = 0)

          int offset = p.x-bwBorder + dim*( p.y - bwBorder );
          unsigned long long mask = 1ll << (dim*dim - offset - 1);

// printf("p.x = %d, p.y = %d, offset = %d, bit = %d, %c\n", p.x-bwBorder, p.y-bwBorder, offset, dim*dim-offset-1, (code&mask) ? '1' : '0' );
// printf("Mask = %016llx\n", mask);

          if( (code & mask) )
            tag.at<unsigned char>(p) = WHITE;
          else
            tag.at<unsigned char>(p) = BLACK;

        }

      }
    }

  return tag;
  }


// Lower 4 bytes represent the four corners:
//   0 | 1
//   --+--
//   2 | 3
//
// Upper four bytes are used as a mask.
//  0x0_   -- All white/black
//  0x1_   -- edge
//  0x2_   -- inner black corner (3 black, 1 white)
//  0x4_   -- inner white corner (3 white, 1 black)
//  0x8_   -- meeting corners (checkerboard)

// Done by hand..
static unsigned char CornerCodeLUT[] = {
    0x00, 0x20, 0x20, 0x10, 0x20, 0x10, 0x80, 0x40,
    0x20, 0x80, 0x10, 0x40, 0x10, 0x40, 0x40, 0x00
};

  cv::Mat Corners::makeCornerMat( unsigned long long code, int dim, int blackBorder )
  {
    Mat tag( makeTagMat(code, dim, blackBorder, 1) );
    int edge = tag.rows - 1;

    Mat corners( edge, edge, CV_8UC1 );

    for( Point p(0,0); p.y < edge; ++p.y ){
      for( p.x=0; p.x < edge; ++p.x ) {
        unsigned char bits = 0;
        // Might be easier way to do this.  LUT?
        if( tag.at<unsigned char>(p) )             bits |= 0x01;
        if( tag.at<unsigned char>(p.y, p.x+1) )    bits |= 0x02;
        if( tag.at<unsigned char>(p.y+1,p.x) )     bits |= 0x04;
        if( tag.at<unsigned char>(p.y+1, p.x+1) )  bits |= 0x08;

        corners.at<unsigned char>(p) = bits | (CornerCodeLUT[bits] & 0xF0);
      }
    }

    return corners;
  }

   cv::Mat Corners::drawCornerMat( const Mat &corners )
  {
    Mat out( corners.size().width * 2, corners.size().height * 2, CV_8UC1 );

    for( Point p(0,0); p.y < corners.rows; ++p.y ) {
      for( p.x = 0; p.x < corners.cols; ++p.x ) {

      out.at<unsigned char>( p.y*2,   p.x*2 )   = ( corners.at<unsigned char>(p) & 0x01 ) ? 1 : 0;
      out.at<unsigned char>( p.y*2,   p.x*2+1 ) = ( corners.at<unsigned char>(p) & 0x02 ) ? 1 : 0;
      out.at<unsigned char>( p.y*2+1, p.x*2 )   = ( corners.at<unsigned char>(p) & 0x04 ) ? 1 : 0;
      out.at<unsigned char>( p.y*2+1, p.x*2+1 ) = ( corners.at<unsigned char>(p) & 0x08 ) ? 1 : 0;
      }
    }

  return out;
  }

}