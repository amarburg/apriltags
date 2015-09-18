
#include "AprilTags/Utils.h"

namespace AprilTags {

using namespace cv;

Rect boundingBox( const TagDetection &detection )
{
  std::vector<Point2f> corners(4);
  for( unsigned int i = 0; i < 4; ++i ) {
    corners[i].x = detection.p[i].first;
    corners[i].y = detection.p[i].second;
  }

  return cv::boundingRect( corners );
}

 void expandBoundingBox( Rect &bb, float expansion )
{
  bb.x -= bb.width * 0.5 * expansion;
  bb.width *= ( 1.0 + expansion );
  bb.y -= bb.height * 0.5 * expansion;
  bb.height *= ( 1.0 + expansion );
}

}
