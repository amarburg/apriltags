
#pragma once

#include <map>
#include <vector>
#include <memory>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include "AprilTags/DetectorBase.h"
#include "AprilTags/CornerArray.h"

#include "CornerDetector/CornerDetectorTypes.h"

namespace AprilTags {

	struct Intersection;
	struct Triplet;

	class DelaunayGeometry;

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
			TriangleImage = BASE_DEBUG_IMAGES+1,
			MeshImage = BASE_DEBUG_IMAGES+2,
			CORNER_DEBUG_IMAGES = BASE_DEBUG_IMAGES+3
		};


		CornerDetector( void );

		CornerDetectionArray detect( const cv::Mat &inImage, const CornerArray &array );

		void attemptMatch( const CornerArray &array, const DelaunayGeometry &delaunany );

	protected:

		void drawIntersection( Mat &img, const shared_ptr<Intersection> &intersection, const cv::Scalar &color = cv::Scalar(0,0,255) );
		void saveIntersectionImage( const std::vector< std::shared_ptr<Intersection> > &intersections );
		void saveTriangleImage( const std::vector< std::shared_ptr<Triangle> > &triangles );
		void saveMeshImage( const std::map< shared_ptr< Intersection >, cv::Point2f > &mesh );

	};


}
