
#include "AprilTags/Utils.h"

namespace AprilTags {

using namespace cv;

Rect boundingBox( const TagDetection &detection )
{
  float minx(FLT_MAX), maxx(0), miny(FLT_MAX), maxy(0);
  std::vector<Point2f> corners(4);
  for( unsigned int i = 0; i < 4; ++i ) {

    minx = std::min( minx, detection.p[i].first );
    maxx = std::max( maxx, detection.p[i].first );

    miny = std::min( miny, detection.p[i].second );
    maxy = std::max( maxy, detection.p[i].second );
  }

  return Rect( minx, miny, (maxx-minx), (maxy-miny) );
}

void expandBoundingBox( Rect &bb, float expansion )
{
  bb.x -= bb.width * 0.5 * expansion;
  bb.width *= ( 1.0 + expansion );
  bb.y -= bb.height * 0.5 * expansion;
  bb.height *= ( 1.0 + expansion );
}

void clipBoundingBox( Rect &bb, const cv::Mat &image )
{
  bb.x = std::max( bb.x, 0 );
  bb.y = std::max( bb.y, 0 );

  bb.width = std::min( bb.width, image.cols - bb.x );
  bb.height = std::min( bb.height, image.rows - bb.y );
}

}
