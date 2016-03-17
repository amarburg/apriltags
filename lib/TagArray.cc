

#include <iostream>
using namespace std;

#include "AprilTags/TagArray.h"


#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

namespace AprilTags {

	using namespace cv;

TagArray::TagArray( const TagCodes &code )
	: _code( code )
{;}

unsigned int TagArray::add( const cv::Point2f &center, Code_t code, float rot )
{
	_elements.push_back( ArrayElement(center, code, rot) );
	return _elements.size();
 }

Mat TagArray::draw(  cv::Mat &mat, const Size bitSize )
{
	if( size() == 0 || bitSize.area() == 0 ) return mat;

	Rect_<float> bb( boundingBox() );

	cv::Size imgSize( bitSize.width * bb.width, bitSize.height * bb.height );

	const cv::Scalar backgroundColor( 255 );
	mat.create( imgSize, CV_8UC1 );
	mat = backgroundColor;												// Fill with white

	// Draw each element in turn
	for( std::vector< ArrayElement >::const_iterator itr = _elements.begin(); itr != _elements.end(); ++itr  ) {
		Mat elem( Corners::drawTagMat( _code, itr->code, bitSize, 1, 0 ) );

		// Create an affine transform
		Mat trans = (Mat_<double>(2,3) << cos( itr->rotation ), sin( itr->rotation), (itr->center.x-0.5)*bitSize.width  - bb.x,
																      -sin( itr->rotation), cos( itr->rotation), (itr->center.y-0.5)*bitSize.height - bb.y );

		warpAffine( elem, mat, trans, imgSize, INTER_LINEAR, BORDER_TRANSPARENT, backgroundColor );

		// Composite warped into mat (is this a saturating add?)

		imshow( "elem",elem);
		// imshow( "warped", warped);
		// cv::add(mat, warped, mat);

		imshow( "mat", mat);
		waitKey(0);
	}


	return mat;
}


Rect_<float> TagArray::boundingBox( void ) const
{
	// Calculate the inefficient way
	std::vector< Point2f > corners;

	// TODO.  Should expose these to end user
	const int whiteBorder = 1, blackBorder = 1;
	float radius = (_code.dim + 2*whiteBorder + 2*blackBorder) * 0.5;

	for( std::vector< ArrayElement >::const_iterator itr = _elements.begin(); itr != _elements.end(); ++itr  ) {
		Point2f cor[4] = { Point2f( radius, radius ),
 												Point2f( radius, -radius ),
												Point2f( -radius, -radius ),
												Point2f( -radius, radius ) };

		float c = cos( itr->rotation ), s = sin( itr->rotation );

		for( unsigned int i = 0; i < 4; ++i ) {
		corners.push_back( cv::Point2f( itr->center.x + (  cor[i].x * c + cor[i].y * s),
  																	itr->center.y + ( -cor[i].x * s + cor[i].y * c) ));
		}

	}

	// And find max.
	Point2f mini( corners.back() ), maxi( corners.back() );
	corners.pop_back();

	for( std::vector< cv::Point2f >::const_iterator c = corners.begin(); c != corners.end() ; ++c ) {
		if( c->x > maxi.x ) maxi.x = c->x;
		if( c->y > maxi.y ) maxi.y = c->y;
		if( c->x < mini.x ) mini.x = c->x;
		if( c->y < mini.y ) mini.y = c->y;
	}

	return cv::Rect_<float>( mini.x, mini.y, maxi.x-mini.x, maxi.y-mini.y );
}

}
