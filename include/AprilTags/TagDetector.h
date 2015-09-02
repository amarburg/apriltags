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
        m_BlockSize( DefaultAdaptiveThresholdRadius ),
        m_Offset( DefaultAdaptiveThresholdOffset ),
        m_Sigma(0),
        m_SegmentationSigma(0.8) {}

  	std::vector<TagDetection> extractTags(const cv::Mat& image);

    /**
     * \brief If true will additionally try to extract quads using adaptive thresholding and OpenCV findContours.
     */
    void SetUseHybridMethod(const bool& useHybrid);

    /**
     * \brief Sets the minimum size of the tag, measured as a fraction between 0 and 1 of the maximum of the number of rows and columns.
     */
    void SetMinSize(const float& minSize);

    /**
     * \brief Sets the maximum size of the tag, measured as a fraction between 0 and 1 of the maximum of the number of rows and columns.
     */
    void SetMaxSize(const float& maxSize);

    /**
     * \brief Sets the window size for adaptive thresholding.
     */
    void SetBlockSize(const int& blockSize);

    /**
     * \brief Sets the amount below the mean intensity of the window to set the threshold at.
     */
    void SetOffset(const int& offset);

    /**
     * \brief Sets the gaussian sigma to be applied to the image in general, so will affect the ability to read off tag patterns.
     */
    void SetSigma(const float& sigma);

    /**
     * \brief Sets the gaussian sigma used when detecting the tag outline, so will affect the ability to find a tag.
     */
    void SetSegmentationSigma(const float& segmentationSigma);


  protected:

    /*
     * If defined, will build DebugTagDetector with debug output
     * If undefined, these virtuals aren't needed.
     */
  #ifdef BUILD_DEBUG_TAG_DETECTOR
    virtual void drawOriginalImage( const FloatImage &img )              {;}
    virtual void drawGaussianLowPassImage( const FloatImage &img )       {;}
    virtual void drawLineSegments( const vector<Segment> &segments )     {;}
    virtual void drawQuadImage( const vector<Quad> &quads )              {;}
    virtual void drawQuadBit( const cv::Point2f &pt, const cv::Scalar &color ) {;}
  #endif


  private:

    void ExtractSegment(
      const GLineSegment2D& gseg,
      const float& length,
      const std::vector<XYWeight>& points,
      const FloatImage& thetaImage,
      const FloatImage& magImage,
      Segment& seg
    );

    bool m_UseHybrid;
    float m_MinSize;
    float m_MaxSize;
    int m_BlockSize;
    int m_Offset;
    float m_Sigma;
    float m_SegmentationSigma;

  };

  #ifdef BUILD_DEBUG_TAG_DETECTOR

  class DebugTagDetector : public TagDetector {
  public:
    DebugTagDetector(const TagCodes& tagCodes)
      : TagDetector( tagCodes )  {;}

    /* Debug outputs */
    cv::Mat originalImage, gaussianLowPassImage, lineSegmentsImage, quadImage;

  protected:
    virtual void drawOriginalImage( const FloatImage &fimSeg );
    virtual void drawGaussianLowPassImage( const FloatImage &fimSeg );
    virtual void drawLineSegments( const vector<Segment> &segments );
    virtual void drawQuadImage( const vector<Quad> &quads );
    virtual void drawQuadBit( const cv::Point2f &pt, const cv::Scalar &color );
  };

  #endif

} // namespace

#endif
