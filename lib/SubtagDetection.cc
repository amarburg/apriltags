
#include "AprilTags/SubtagDetection.h"
#include "AprilTags/Corners.h"

namespace AprilTags {

using namespace cv;

SubtagDetection::SubtagDetection( const TagCodes &code_, const TagDetection &detection )
: code( code_[detection.id] ), id( detection.id ), good( false )
{
  Mat cornerMat( Corners::makeCornerMat( code_, detection.id ) );

  // The width of one square in their tag-centric coord frame
  float ncScalar  = 2.0/ (code_.dim+2);

  float ncOffset  = 1;  //ncScalar * (code_.dim/2.0 + 1);

   for( Point p(0,0); p.y < cornerMat.rows; ++p.y ) {
     for( p.x = 0;    p.x < cornerMat.cols; ++p.x ) {
        CornerDetection c( cornerMat.at<unsigned char>(p) );

        // Translate to their in-tag location.
        c.inTag.x = p.x*ncScalar - ncOffset;

        // Once again, the in-tag coordinate system is aligned
        // with the mathematical system s.t. -1,-1 is at the lower
        // left corner while the cornerMat is in image coordinates
        // (origin at upper left)
        c.inTag.y = -(p.y*ncScalar - ncOffset);

        c.inImage = detection.interpolatePt( c.inTag );

        corners.push_back( c );
    }
  }

}


}
