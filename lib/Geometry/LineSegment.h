
#pragma once

#include <opencv2/core/core.hpp>

namespace AprilTags {

	// TODO.  Merge with GLineSegment2D
	// A LineSegment is directed, but unlike GLineSegment2D the
	// direction doesn't have a particuar meaning (e.g. gradient goes
	// left->right, etc.)
	struct LineSegment {

		LineSegment( const Segment &seg )
			: _a( seg.getX0(), seg.getY0() ),
	 			_b( seg.getX1(), seg.getY1() )
		{;}

		LineSegment( const cv::Point2f &a, const cv::Point2f &b )
			: _a(a), _b(b)
		{;}

		const cv::Point2f &a(void) const { return _a; }
		const cv::Point2f &b(void) const { return _b; }

		const float theta( void ) const { return atan2( _b.y-_a.y, _b.x - _a.x ); }

		LineSegment flip( void ) const { return LineSegment( _b, _a); }

	protected:
		cv::Point2f _a, _b;
	};

}
