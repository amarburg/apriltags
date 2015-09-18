

#ifndef __SUBTAG_DETECTOR__
#define __SUBTAG_DETECTOR__

#include <vector>
#include <opencv2/core.hpp>

#include "AprilTags/SubtagDetection.h"
#include "AprilTags/TagDetection.h"

namespace AprilTags {

  using std::vector;
  using cv::Mat;

  // This detector uses the corners to find substructure within an already
  // detected tag ...
  // CornerDetector goes from nothing (raw image) to find tags by detecting
  // corners.
  class SubtagDetector {
  public:

    SubtagDetector( const TagCodes &tagCodes );

    SubtagDetection detectTagSubstructure(const Mat &image, const TagDetection &detection );

    float minPixPerEdge;
    cv::Size subPixSearchWindow;

  protected:

#ifdef BUILD_DEBUG_TAG_DETECTOR
  virtual void drawPredictedCornerLocations( const Mat &image, const cv::Rect &bb,
                                              const SubtagDetection &detection ) {;}
  virtual void drawRefinedCornerLocations( const Mat &image, const cv::Rect &bb,
                                              const SubtagDetection &detection ) {;}
#endif

    const TagCodes &_code;

  };

#ifdef BUILD_DEBUG_TAG_DETECTOR

  class DebugSubtagDetector : public SubtagDetector {
  public:

    DebugSubtagDetector( const TagCodes &tagCodes )
      : SubtagDetector( tagCodes )
    {;}

    virtual void drawPredictedCornerLocations( const Mat &image, const cv::Rect &bb,
                                                const SubtagDetection &detection )
      { drawCornerLocations( image, bb, detection, predictedCorners ); }

    virtual void drawRefinedCornerLocations( const Mat &image, const cv::Rect &bb,
                                                const SubtagDetection &detection )
      { drawCornerLocations( image, bb, detection, refinedCorners ); }

     void drawCornerLocations( const Mat &image, const cv::Rect &bb,
                                const SubtagDetection &detection,
                                Mat &dest );


    Mat predictedCorners, refinedCorners;

  };

}

#endif

#endif
