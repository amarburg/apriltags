
#pragma once

#include <vector>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include "AprilTags/DetectorBase.h"
#include "AprilTags/CornerArray.h"

namespace AprilTags {

	struct Intersection;
	struct Triplet;

	struct CornerDetection {
	public:
		CornerDetection()
		{;}

	};

	typedef std::vector< CornerDetection > CornerDetectionArray;


	class CornerDetector : public DetectorBase {
	public:

		enum  CornerDebugImages_t {
			IntersectionImage = BASE_DEBUG_IMAGES,
			TripletImage = BASE_DEBUG_IMAGES+1,
			CORNER_DEBUG_IMAGES = BASE_DEBUG_IMAGES+2
		};


		CornerDetector( void );

		CornerDetectionArray detect( const cv::Mat &inImage, const CornerArray &array );

	protected:

		void saveIntersectionImage( const vector< Intersection > &intersections );
		void saveTripletImage( const vector< Triplet > &triplets );


	};

}
