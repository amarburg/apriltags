

#include <iostream>
using namespace std;

#include "AprilTags/CornerArray.h"

#include "Utils/range.hpp"
#include "Utils/MathUtil.h"

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

namespace AprilTags {

	using namespace cv;
	using util::lang::range;

CornerArray::CornerArray(  )
{;}

unsigned int CornerArray::add( const cv::Point2f &center, unsigned int arrayId, cv::Point2i idx, unsigned char type, float rotation )
{
	_elements.push_back( ArrayElement(center, arrayId, idx, type, rotation) );
	return _elements.size();
 }

unsigned int CornerArray::add( const cv::Point2f &center, unsigned char type, float rotation )
{
	_elements.push_back( ArrayElement(center, 0, Point2i(0,0), type, rotation) );
	return _elements.size();
}

std::vector< std::pair< int, int > > CornerArray::spinMatch( unsigned char corner ) const
{
	std::vector< std::pair< int, int > > output;

	for( auto i : range(0ul,_elements.size()) ) {
		const int rot = Corners::spinMatch( corner, _elements[i].cornerType );
		if( rot >= 0 )
			output.push_back( std::make_pair( i, rot ));
	}

	return output;
}

std::vector< std::pair< int, int > > CornerArray::spinMatch( unsigned char corner, const cv::Point2f &origin, float radius ) const
{
	const float r2 = radius*radius;
	std::vector< std::pair< int, int > > output;

	for( auto i : range(0ul,_elements.size()) ) {
		if( MathUtil::distance2Dsqr( _elements[i].center, origin ) > r2 ) continue;

		const int rot = Corners::spinMatch( corner, _elements[i].cornerType );
		if( rot >= 0 )
			output.push_back( std::make_pair( i, rot ));
	}

	return output;
}



CornerArrayImage CornerArray::draw( cv::Mat &mat, const Size tagSize ) const
{
	if( size() == 0 || tagSize.area() == 0 ) return CornerArrayImage();

	// Bounding box is in tag-normalized units
	Rect_<float> bb( boundingBox() );

	const float margin = 0.25;  // Margin also expressed in tag-widths
	bb.x -= margin;
	bb.y -= margin;
	bb.width += 2*margin;
	bb.height += 2*margin;

	cv::Size imgSize( tagSize.width * bb.width, tagSize.height * bb.height );

	const float radius = (tagSize.width + tagSize.height) * 0.5 * 0.05;

	const cv::Scalar backgroundColor( 255,255,255 );
	mat.create( imgSize, CV_8UC3 );
	mat = backgroundColor;												// Fill with white

	// Draw each element in turn
	for( std::vector< ArrayElement >::const_iterator itr = _elements.begin(); itr != _elements.end(); ++itr  ) {
		Point2f cornerCenter((itr->center.x-bb.x) * tagSize.width, (itr->center.y-bb.y) * tagSize.height );

		for( unsigned int i = 0; i < 4; ++i ) {
			cv::circle( mat, cornerCenter + radius * Point2f( cos( Corners::angles[i]+itr->rotation  ), sin( Corners::angles[i] + itr->rotation  ) ),
									radius/1.3,
									cv::Scalar( 0,0,0 ),
									itr->cornerType & (1 << i) ? radius/10 : -1 );
		}

		cv::Scalar color( 255, 0, 0); // Blue for more corners
		if( itr->idx.x == 0 && itr->idx.y == 0 )	color = cv::Scalar( 0,0,255); // Red for (0,0)

		// And overlay a master circle
		cv::circle( mat, cornerCenter,
								radius, color, radius/10 );

	}

	return CornerArrayImage( mat, bb, tagSize );
}


Rect_<float> CornerArray::boundingBox( void ) const
{
	Point2f mini( _elements.front().center ), maxi( _elements.front().center );

	for( std::vector< ArrayElement >::const_iterator c = _elements.begin(); c != _elements.end() ; ++c ) {
		if( c->center.x > maxi.x ) maxi.x = c->center.x;
		if( c->center.y > maxi.y ) maxi.y = c->center.y;
		if( c->center.x < mini.x ) mini.x = c->center.x;
		if( c->center.y < mini.y ) mini.y = c->center.y;
	}

	return cv::Rect_<float>( mini.x, mini.y, maxi.x-mini.x, maxi.y-mini.y );
}

}
