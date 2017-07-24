
#include "AprilTags/Corners.h"

#include <opencv2/imgproc/imgproc.hpp>
using namespace cv;

namespace AprilTags {
namespace Corners {


//===================================================================

Mat makeTagMat( const TagCodes &family, int which, int blackBorder, int whiteBorder )
{
  int dim = family.dim;

  Code_t code = family[which];

  int bwBorder = blackBorder + whiteBorder;
  int edge = dim + 2*(bwBorder);

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
//
//   2 | 3     .-> +X
//   --+--     |
//   1 | 0     v +Y
//
// Upper four bytes are used as a mask.
//  0x0_   -- All white/black
//  0x1_   -- edge
//  0x2_   -- inner black corner (3 black, 1 white)
//  0x4_   -- inner white corner (3 white, 1 black)
//  0x8_   -- meeting corners (checkerboard)

// Done by hand..
static unsigned char CornerCodeLUT[] = {
    0x00, 0x20, 0x20, 0x10, 0x20, 0x80, 0x10, 0x40,
    0x20, 0x10, 0x80, 0x40, 0x10, 0x40, 0x40, 0x00
};

unsigned char cornerLUT( unsigned char corner )
{
  unsigned char upper = CornerCodeLUT[ corner & 0x0F ];
  return (upper & 0xF0) | (corner & 0x0F);
}


cv::Mat makeCornerMat( const TagCodes &family, int which, int blackBorder, int whiteBorder )
{
  Mat tag( makeTagMat(family, which, blackBorder, whiteBorder) );

  int edge = tag.rows - 1;

  Mat corners( edge, edge, CV_8UC1 );

  for( Point p(0,0); p.y < edge; ++p.y ){
    for( p.x=0; p.x < edge; ++p.x ) {
      unsigned char bits = 0;
      // Might be easier way to do this.  LUT?
      if( tag.at<unsigned char>(p) )             bits |= 0x04;
      if( tag.at<unsigned char>(p.y,   p.x+1) )  bits |= 0x08;
      if( tag.at<unsigned char>(p.y+1, p.x) )    bits |= 0x02;
      if( tag.at<unsigned char>(p.y+1, p.x+1) )  bits |= 0x01;

      corners.at<unsigned char>(p) = cornerLUT( bits );
    }
  }

  return corners;
}

// Measured + in the image x-y frame, CCW in the "looking at the image" frame
unsigned char rotate( unsigned char a, int count )
{
  unsigned char out = a & 0x0F;
  if( count > 0 && count < 4 ) {
    out <<= count;
  } else if( count < 0 && count > -4 ) {
    out <<= 4+count;
  }
  out = (out & 0x0F) | (out & 0xF0)>>4;

  out &= 0x0F;
  out |= (a & 0xF0);

  return out;
}

int spinMatch( unsigned char a, unsigned char b )
{
  // By definition, if the type code matches, then a spinMatch is possible
  // (and for a checkerboard corner, there are two possible solutions...)
  if( (a & 0xF0) != (b & 0xF0) ) return -1;

  if( (a & 0x0F) == (b & 0x0F) ) return 0;
  for( unsigned int i = 1; i < 4; ++i )
    if( (rotate(a,i) & 0x0F) == (b & 0x0F) ) return i;

  return -1;
}

//=== Convenience drawing functions ====

cv::Mat drawTagMat( const TagCodes &family, int which, const Size size, int whiteBorder, int blackBorder )
{
  return drawTagMat( makeTagMat( family, which, whiteBorder, blackBorder ), size );
}


cv::Mat drawTagMat( const Mat &tag, const Size size )
{
  if( size.area() == 0 ) return tag;

  Mat huge;
  cv::resize( 255*tag, huge, size, 0, 0, INTER_NEAREST );
  return huge;
}


cv::Mat drawCornerMat( const TagCodes &family, int which, const Size size )
{
  return drawCornerMat( makeCornerMat( family, which ), size );
}


cv::Mat drawCornerMat( const Mat &corners, const Size size )
{
  Mat out( corners.size().width * 2, corners.size().height * 2, CV_8UC1 );

  for( Point p(0,0); p.y < corners.rows; ++p.y ) {
    for( p.x = 0; p.x < corners.cols; ++p.x ) {

      out.at<unsigned char>( p.y*2,   p.x*2 )   = ( corners.at<unsigned char>(p) & 0x04 ) ? 1 : 0;
      out.at<unsigned char>( p.y*2,   p.x*2+1 ) = ( corners.at<unsigned char>(p) & 0x08 ) ? 1 : 0;
      out.at<unsigned char>( p.y*2+1, p.x*2 )   = ( corners.at<unsigned char>(p) & 0x02 ) ? 1 : 0;
      out.at<unsigned char>( p.y*2+1, p.x*2+1 ) = ( corners.at<unsigned char>(p) & 0x01 ) ? 1 : 0;
    }
  }

  if( size.area() == 0 ) return out;

  Mat huge;
  cv::resize( 255*out, huge, size, 0, 0, INTER_NEAREST );
  return huge;
}

}
}
