
#include <iostream>
using namespace std;

#include "AprilTags/SubtagDetector.h"
#include "AprilTags/Utils.h"

namespace AprilTags {

using namespace cv;

SubtagDetector::SubtagDetector( const TagCodes &tagCodes )
: minPixPerEdge( 2.0 ),
  subPixSearchWindow( 5,5 ),
  _code( tagCodes )
{;}

SubtagDetection
SubtagDetector::detectTagSubstructure(const Mat& image, const TagDetection &detection )
{
  SubtagDetection d( _code, detection );
  // First a quick scale check.  Determine the bounding box for the tag.
  // Needs to be at least at least minPixPerEdge * edge pixels wide before
  // detection is attempted.
  Rect bb( boundingBox( detection ) );

  float minPix = minPixPerEdge * _code.dim;
  if( (bb.width < minPix) || (bb.height < minPix) ) return d;

  // Identify, extract and warp to planar/upright ROI in the image based on the detection.
  //cout << "Bounding box is " << bb.width << " x " << bb.height << endl;

  const float expansion = 0.1;
  expandBoundingBox( bb, expansion );

  // TODO: When do we clip to edge of image?
  drawPredictedCornerLocations( image, bb, d );

  // Attempt to refine corner location
  vector< Point2f > detectableCorners;
  for( unsigned int i = 0; i < d.corners.size(); ++i ) {
    if( d.corners[i].detectable() ) detectableCorners.push_back( d.corners[i].inImage );
  }

  cornerSubPix( image, detectableCorners, subPixSearchWindow,  Size(-1,-1), TermCriteria() );

  for( unsigned int i = 0, c = 0; i < d.corners.size(); ++i ) {
    if( c >= detectableCorners.size() ) break;   // This shouldn't happen
    if( d.corners[i].detectable() ) d.corners[i].inImage = detectableCorners[ c++ ];
  }

  drawRefinedCornerLocations( image, bb, d );

  return d;
}


#ifdef BUILD_DEBUG_TAG_DETECTOR

//========= DebugSubtagDetector ======================================


void DebugSubtagDetector::drawCornerLocations( const Mat &image, const cv::Rect &bb,
                                                        const SubtagDetection &detection,
                                                Mat &dest )
{
  Mat imageRoi( image, bb );
  if( image.depth() != 3 )
    cvtColor( imageRoi, dest, CV_GRAY2BGR );
  else
    imageRoi.copyTo( dest );

  for( unsigned int i = 0; i < detection.corners.size(); ++i ) {
    const CornerDetection &c( detection.corners[i] );

    Point2f pt( c.inImage );
    pt.x -= bb.x;
    pt.y -= bb.y;

    if( c.detectable() ) cv::circle( dest, pt, 4, Scalar(0,0,255), 1 );

    char out[3];
    snprintf( out, 3, "%02x", detection.corners[i].corner );
    putText( dest, out, pt, FONT_HERSHEY_SIMPLEX , 0.4, Scalar( 0,255,0 ) );

  }

}


#endif


}
