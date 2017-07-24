
#pragma once

#include <vector>

#include <opencv2/core/core.hpp>

#include "AprilTags/TagFamily.h"

#include "AprilTags/Corners.h"
#include "AprilTags/CornerArray.h"

namespace AprilTags {

	struct CornerArrayImage;

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

		const ArrayElement &operator[]( unsigned int i ) const { return _elements[i]; }

		std::vector< std::pair< int, int > > spinMatch( unsigned char corner ) const;
		std::vector< std::pair< int, int > > spinMatch( unsigned char corner, const cv::Point2f &origin, float radius ) const;

		// bitSize determines the size of one tag, with black border in pixels.
		// Total image size is calculated from this unit of measure.
		CornerArrayImage draw( cv::Mat &mat, const cv::Size tagSize = cv::Size(1,1) ) const;

	protected:

		cv::Rect_<float> boundingBox( void ) const;

		std::vector< ArrayElement > _elements;

	};

	struct CornerArrayImage {
		CornerArrayImage( void )
			: mat(), bb(), tagSize()
		{;}

		CornerArrayImage( const cv::Mat &m, const cv::Rect_<float> &b, const cv::Size &ts )
			: mat(m), bb(b), tagSize(ts)
		{;}

		cv::Point2f operator()(const cv::Point2f &pt )
			{ return cv::Point2f((pt.x-bb.x) * tagSize.width, (pt.y-bb.y) * tagSize.height ); }

		cv::Mat mat;
		cv::Rect_<float> bb;
		cv::Size tagSize;
	};

}
