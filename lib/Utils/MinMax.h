

#pragma once

#include <opencv2/core/core.hpp>

struct MinMax {
	MinMax( void )
		: min(1e23), max(-1e23)
	{;}

	void check( float x )
	{
		if( x < min ) min = x;
		if( x > max ) max = x;
	}

	float min, max;
};

struct PointMinMax {
	PointMinMax( void )
		: x(), y()
	{;}

	void check( const cv::Point2f &pt )
	{
		x.check( pt.x );
		y.check( pt.y );
	}

	MinMax x,y;
};
