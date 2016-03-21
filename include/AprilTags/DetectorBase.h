
#pragma once

#include <vector>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include "Geometry/Edge.h"
#include "Geometry/Segment.h"

namespace AprilTags {

	class DetectorBase {
	public:

		/* Intermediate images */
    enum  DebugImages_t {
            OriginalImage = 0,
            OriginalBGRImage = 1,
            BlurredImage = 2,
            MagnitudeImage = 3,
            LineSegmentsImage = 4,
            BASE_DEBUG_IMAGES = 5
            };

		bool saveDebugImages( bool val ) { return _saveDebugImages = val; }
		cv::Mat debugImage( unsigned int which );

	protected:
		DetectorBase( unsigned int numDebugImages = BASE_DEBUG_IMAGES )
		: _saveDebugImages( false ),
		  _blurSigma(0),
			_segmentationSigma(0.8),
			_debugImages( numDebugImages )
		{;}

		// Each of these correspond to steps in the original algorithm
		void doImagePrep( const cv::Mat &inImage, cv::Mat &outImage );
		void calculateGradient( const cv::Mat &blurredImage, cv::Mat &magnitude, cv::Mat &angle);
		void fitSegments(const cv::Mat &magnitude, const cv::Mat &angle, std::vector<Segment> &segments);

		bool validDebugImage( unsigned int which );
		void saveDebugImage( const cv::Mat &img, unsigned int which, bool clone = true );
		void saveOriginalImage( const cv::Mat &img );
		void saveLineSegments( const std::vector<Segment> &segments );


		bool _saveDebugImages;

		float _blurSigma;
		float _segmentationSigma;

	private:

		std::vector< cv::Mat > _debugImages;

	};

}
