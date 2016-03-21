
#pragma once

#include <vector>

#include <opencv2/core/core.hpp>

#include "AprilTags/TagFamily.h"

#include "AprilTags/Corners.h"
#include "AprilTags/CornerArray.h"

namespace AprilTags {

	class CornerArray {
	public:

		struct ArrayElement {
		public:
			ArrayElement( const cv::Point2f &center_, unsigned int arrayId_, cv::Point2i idx_, unsigned char type, float rotation  )
				: center( center_ ),
				id( arrayId_ ),
				idx( idx_ ),
				cornerType( type ),
				rotation( rotation )
			{;}

			cv::Point2f center;
			unsigned int id;
			cv::Point2i idx;
			unsigned char cornerType;
			float rotation;
		};

		CornerArray( );

		unsigned int size( void ) const { return _elements.size(); }
		unsigned int add( const cv::Point2f &center_, unsigned int arrayId_, cv::Point2i idx_, unsigned char type, float rotation );
		unsigned int add( const cv::Point2f &center_, unsigned char type, float rotation );


		// bitSize determines the size of one tag, with black border in pixels.
		// Total image size is calculated from this unit of measure.
		cv::Mat draw( cv::Mat &mat, const cv::Size tagSize = cv::Size(1,1) );

	protected:

		cv::Rect_<float> boundingBox( void ) const;

		std::vector< ArrayElement > _elements;

	};

}
