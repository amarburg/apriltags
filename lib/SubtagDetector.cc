
#include <algorithm>
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

  vector<CornerDetection>
  SubtagDetector::detectTagSubstructure(const Mat& image, const TagDetection &detection )
  {
    vector<CornerDetection> corners;

    // First the pass/fail checks.

    // Quick scale check.  Determine the bounding box for the tag.
    // Needs to be at least at least minPixPerEdge * edge pixels wide before
    // detection is attempted.
    Rect bb( boundingBox( detection ) );
    float minPix = minPixPerEdge * _code.dim;
    if( (bb.width < minPix) || (bb.height < minPix) ) return corners;

    // Identify, extract and warp to planar/upright ROI in the image based on the detection.
    //cout << "Bounding box is " << bb.width << " x " << bb.height << endl;

    const float expansion = 0.1;
    expandBoundingBox( bb, expansion );
    clipBoundingBox( bb, image );

    populateCorners( detection, corners );

    if( _saveDebugImages ) drawPredictedCornerLocations( image, bb, corners );

    vector< Point2f > refinedLocations;
    std::transform( corners.begin(), corners.end(),
                    back_inserter<vector< Point2f > >(refinedLocations),
                    CornerDetection::InImageTx );
    cornerSubPix( image, refinedLocations, subPixSearchWindow,  Size(-1,-1), TermCriteria() );

    assert( refinedLocations.size() == corners.size() );
    for( unsigned int i = 0; i < corners.size(); ++i ) {
        // Consider other heuristics here

        corners[i].inImage = refinedLocations[ i ];
    }

    if( _saveDebugImages ) drawRefinedCornerLocations( image, bb, corners );

    return corners;
  }

  void SubtagDetector::populateCorners( const TagDetection &detection, vector<CornerDetection> &corners )
  {
    Mat cornerMat( Corners::makeCornerMat( _code, detection.id ) );

    // The width of one square in the tag-centric coord frame
    float ncScalar  = 2.0/ (_code.dim+2);
    float ncOffset  = 1;  //ncScalar * (code_.dim/2.0 + 1);

     for( Point p(0,0); p.y < cornerMat.rows; ++p.y ) {
       for( p.x = 0;    p.x < cornerMat.cols; ++p.x ) {
          CornerDetection c( cornerMat.at<unsigned char>(p) );
          if( c.detectable() == false ) continue;

          // Translate to the in-tag location.
          c.inTag.x = p.x*ncScalar - ncOffset;

          // The tag coordinate system is aligned
          // with the mathematical system s.t. -1,-1 is at the lower
          // left corner while the cornerMat is in image coordinates
          // (origin at upper left)
          c.inTag.y = -(p.y*ncScalar - ncOffset);

          c.inImage = detection.interpolatePt( c.inTag );

          corners.push_back( c );
      }
    }
  }


  //========= Debug Functions ======================================

   bool SubtagDetector::validDebugImage( DebugImages_t which )
  {
    return (which >= 0) && (which < NUM_DEBUG_IMAGES);
  }

  bool SubtagDetector::saveDebugImages( bool val )
  {
    return _saveDebugImages = val;
  }

  Mat SubtagDetector::debugImage( DebugImages_t which )
  {
    if( validDebugImage( which ) )
    return _debugImages[which];

    return Mat();
  }

  void SubtagDetector::saveDebugImage( const Mat &img, DebugImages_t which, bool clone )
  {
    if( validDebugImage( which ) ) {
      if( clone )
        img.copyTo(_debugImages[which]);
      else
        _debugImages[which] = img;
    }
  }

  void SubtagDetector::drawPredictedCornerLocations( const Mat &image, const cv::Rect &bb, const vector<CornerDetection> &corners )
  {
    Mat img;
    drawCornerLocations( image, bb, corners, img );
    saveDebugImage( img, PredictedCorners, false );
  }

  void SubtagDetector::drawRefinedCornerLocations( const Mat &image, const cv::Rect &bb, const vector<CornerDetection> &corners )
  {
    Mat img;
    drawCornerLocations( image, bb, corners, img );
    saveDebugImage( img, RefinedCorners, false );
  }


  void SubtagDetector::drawCornerLocations( const Mat &image, const cv::Rect &bb, const vector<CornerDetection> &corners, Mat &dest )
  {
    Mat imageRoi( image, bb );
    if( image.depth() != 3 )
      cvtColor( imageRoi, dest, CV_GRAY2BGR );
    else
      imageRoi.copyTo( dest );

    for( unsigned int i = 0; i < corners.size(); ++i ) {
      Point2f pt( corners[i].inImage );
      pt.x -= bb.x;
      pt.y -= bb.y;

      if( corners[i].detectable() ) cv::circle( dest, pt, 4, Scalar(0,0,255), 1 );

      // Draw the corner code as well
      char out[3];
      snprintf( out, 3, "%02x", corners[i].corner );
      putText( dest, out, pt, FONT_HERSHEY_SIMPLEX , 0.4, Scalar( 0,255,0 ) );

    }

  }




}
