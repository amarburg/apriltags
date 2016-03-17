

#include <iostream>
using namespace std;

#include "AprilTags/CornerArray.h"


#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

namespace AprilTags {

	using namespace cv;

CornerArray::CornerArray(  )
{;}

unsigned int CornerArray::add( const cv::Point2f &center, unsigned int arrayId, cv::Point2i idx, unsigned char type )
{
	_elements.push_back( ArrayElement(center, arrayId, idx, type) );
	return _elements.size();
 }

Mat CornerArray::draw(  cv::Mat &mat, const Size tagSize )
{
	if( size() == 0 || tagSize.area() == 0 ) return mat;

	// Bounding box is in tag-normalized units
	Rect_<float> bb( boundingBox() );

	const float margin = 0.25;  // Margin also expressed in tag-widths
	bb.x -= margin;
	bb.y -= margin;
	bb.width += 2*margin;
	bb.height += 2*margin;

	cv::Size imgSize( tagSize.width * bb.width, tagSize.height * bb.height );

	const float radius = (tagSize.width + tagSize.height) * 0.5 * 0.05;

	const cv::Scalar backgroundColor( 255 );
	mat.create( imgSize, CV_8UC1 );
	mat = backgroundColor;												// Fill with white

	// Draw each element in turn
	for( std::vector< ArrayElement >::const_iterator itr = _elements.begin(); itr != _elements.end(); ++itr  ) {
		Point2f cornerCenter((itr->center.x-bb.x) * tagSize.width, (itr->center.y-bb.y) * tagSize.height );

		// Draw subdots (this is how the corners are ordered: UL, UR, LL, LR )
		const float rots[4] = {3*M_PI/4, M_PI/4, 5*M_PI/4, 7*M_PI/4 };
		for( unsigned int i = 0; i < 4; ++i ) {
			cv::circle( mat, cornerCenter + radius * Point2f( cos( rots[i] + itr->rotation ), sin( rots[i] + itr->rotation ) ),
									radius,
									cv::Scalar( 0,0,0 ),
									itr->type & (1 << i) ? -1 : radius/15 );
		}

		// And overlay a master circle
		cv::circle( mat, cornerCenter,
								radius, cv::Scalar( 0,255,0), radius/10 );

	}

	return mat;
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
