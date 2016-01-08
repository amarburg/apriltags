

#ifndef __APRILTAGS_SUBTAG_DETECTOR__
#define __APRILTAGS_SUBTAG_DETECTOR__

#include <vector>
#include <opencv2/core.hpp>

#include "AprilTags/CornerDetection.h"
#include "AprilTags/TagDetection.h"

namespace AprilTags {

  using std::vector;
  using cv::Mat;

  // This detector uses the corners to find substructure within an already
  // detected tag ...
  class SubtagDetector {
  public:

    SubtagDetector( const TagCodes &tagCodes );

    vector<CornerDetection> detectTagSubstructure(const Mat &image, const TagDetection &detection );

    float minPixPerEdge;
    cv::Size subPixSearchWindow;

    bool saveDebugImages( bool val );

    /* Intermediate images */
    enum DebugImages_t { PredictedCorners = 0,
                         RefinedCorners = 1,
                         NUM_DEBUG_IMAGES = 2
                        };

    Mat debugImage( DebugImages_t which );

  protected:

    void populateCorners( const TagDetection &detection, vector<CornerDetection> &corners );

    //== Functions for generating and accessing debug images ==
    bool validDebugImage( DebugImages_t which );
    void saveDebugImage( const Mat &img, DebugImages_t which, bool clone = true );

    void drawPredictedCornerLocations( const Mat &image, const cv::Rect &bb,
                                              const vector<CornerDetection> &corners );
    void drawRefinedCornerLocations( const Mat &image, const cv::Rect &bb,
                                              const vector<CornerDetection> &corners );
    void drawCornerLocations( const Mat &image, const cv::Rect &bb,
                               const vector<CornerDetection> &corner,
                               Mat &dest );

    const TagCodes &_code;

    bool _saveDebugImages;
    Mat _debugImages[ NUM_DEBUG_IMAGES ];

  };

}

#endif
