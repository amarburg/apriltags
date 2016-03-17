
#pragma once

#include <vector>

#include <opencv2/core/core.hpp>

#include "AprilTags/TagFamily.h"

#include "AprilTags/Corners.h"

namespace AprilTags {

	struct ArrayElement {
	public:
		ArrayElement( const cv::Point2f &center_, Code_t code_, float rot_ = 0 )
			: center( center_ ),
				code( code_ ),
				rotation( rot_ )
		{;}

		cv::Point2f center;
		Code_t code;
		float rotation;
	};

	class TagArray {
	public:

		TagArray( const TagCodes &code );

		unsigned int size( void ) const { return _elements.size(); }
		unsigned int add( const cv::Point2f &center, Code_t code, float rot = 0 );

		// bitSize determines the size of one tag, with black border in pixels.
		// Total image size is calculated from this unit of measure.
		cv::Mat draw( cv::Mat &mat, const cv::Size bitSize = cv::Size(1,1) );

	protected:

		cv::Rect_<float> boundingBox( void ) const;

		const TagCodes &_code;
		std::vector< ArrayElement > _elements;

	};

}
