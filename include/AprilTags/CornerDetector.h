
#pragma once

#include <vector>
#include <opencv2/core/core.hpp>

#include "AprilTags/CornerArray.h"

namespace AprilTags {

	struct CornerDetection {
	public:
		CornerDetection()
		{;}

	};

	typedef std::vector< CornerDetection > CornerDetectionArray;

	class CornerDetector {
	public:

		CornerDetector( void );

		CornerDetectionArray detect( const cv::Mat &img, const CornerArray &array );

	};

}
