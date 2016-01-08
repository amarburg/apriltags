#ifndef TAGDETECTOR_H
#define TAGDETECTOR_H

#include <vector>

#include "opencv2/opencv.hpp"

#include "AprilTags/TagDetection.h"
#include "AprilTags/TagFamily.h"
#include "AprilTags/FloatImage.h"
#include "AprilTags/GLineSegment2D.h"
#include "AprilTags/Quad.h"
#include "AprilTags/XYWeight.h"
#include "AprilTags/Segment.h"

namespace AprilTags {

  using cv::Mat;

  // Default parameter values
  const int DefaultAdaptiveThresholdRadius = 9;
  const int DefaultAdaptiveThresholdOffset = 5;

  class TagDetector {
  public:

  	const TagFamily thisTagFamily;

  	//! Constructor
    // note: TagFamily is instantiated here from TagCodes
    TagDetector(const TagCodes& tagCodes)
      : thisTagFamily(tagCodes),
        m_UseHybrid( false ),
        m_MinSize( 0.02 ),
        m_MaxSize( 0.98 ),
        m_BlockSize( DefaultAdaptiveThresholdRadius ),
        m_Offset( DefaultAdaptiveThresholdOffset ),
        m_Sigma(0),
        m_SegmentationSigma(0.8),
        m_saveDebugImages( false ) {}

  	std::vector<TagDetection> extractTags(const Mat& image);

    /**
     * \brief If true will additionally try to extract quads using adaptive thresholding and OpenCV findContours.
     */
    void SetUseHybridMethod( bool useHybrid);

    /**
     * \brief Sets the minimum size of the tag, measured as a fraction between 0 and 1 of the maximum of the number of rows and columns.
     */
    void SetMinSize( float minSize);

    /**
     * \brief Sets the maximum size of the tag, measured as a fraction between 0 and 1 of the maximum of the number of rows and columns.
     */
    void SetMaxSize( float maxSize);

    /**
     * \brief Sets the window size for adaptive thresholding.
     */
    void SetBlockSize( int blockSize);

    /**
     * \brief Sets the amount below the mean intensity of the window to set the threshold at.
     */
    void SetOffset( int offset);

    //! Gaussian smoothing kernel applied to image (0 == no filter).
    /*! Used when sampling bits. Filtering is a good idea in cases
     * where A) a cheap camera is introducing artifical sharpening, B)
     * the bayer pattern is creating artifcats, C) the sensor is very
     * noisy and/or has hot/cold pixels. However, filtering makes it
     * harder to decode very small tags. Reasonable values are 0, or
     * [0.8, 1.5].
     */
    void SetSigma( float sigma);

    //! Gaussian smoothing kernel applied to image (0 == no filter).
    /*! Used when detecting the outline of the box. It is almost always
     * useful to have some filtering, since the loss of small details
     * won't hurt. Recommended value = 0.8. The case where sigma ==
     * segsigma has been optimized to avoid a redundant filter
     * operation.
     */
    void SetSegmentationSigma( float segmentationSigma);

    bool SaveDebugImages( bool val );

    /* Intermediate images */
    enum  DebugImages_t {
            OriginalImage = 0,
            OriginalBGRImage = 1,
            BlurredImage = 2,
            MagnitudeImage = 3,
            LineSegmentsImage = 4,
            QuadImage = 5,
            NUM_DEBUG_IMAGES = 6
            };

    Mat debugImage( DebugImages_t which );

  private:

    void ExtractSegment(
      const GLineSegment2D& gseg,
      const float& length,
      const std::vector<XYWeight>& points,
      const Mat &thetaImage,
      const Mat &magImage,
      Segment& seg
    );

    bool validDebugImage( DebugImages_t which );
    void saveDebugImage( const Mat &img, DebugImages_t which, bool clone = true );
    void saveOriginalImage( const Mat &img );
    void saveBlurredImage( const Mat &img );
    void saveMagnitudeImage( const Mat &img );
    void saveLineSegments( const vector<Segment> &segments );
    void saveQuadImage( const vector<Quad> &quads );
    void drawQuadBit( const cv::Point2f &pt, const cv::Scalar &color );

    // Parameters
    bool m_UseHybrid;
    float m_MinSize;
    float m_MaxSize;
    int m_BlockSize;
    int m_Offset;
    float m_Sigma;
    float m_SegmentationSigma;

    Mat _debugImages[ NUM_DEBUG_IMAGES ];
    bool m_saveDebugImages;

  };

} // namespace

#endif
