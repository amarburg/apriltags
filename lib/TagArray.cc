

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

CornerArray TagArray::corners( void ) const
{
	CornerArray out;

	for( unsigned int i = 0; i < _elements.size(); ++i ) {
		const ArrayElement &elem( _elements[i] );
		Mat corners( Corners::makeCornerMat( _code, elem.code, 1, 1 ) );

		// Point2f halfLen( corners.cols * 0.5, corners.rows * 0.5 );
		for( Point2i p(0,0); p.y < corners.rows; ++p.y ) {
			for( p.x = 0; p.x < corners.cols; ++p.x ) {
				Point2f xyloc( (float)p.x/(corners.cols-1) - 0.5, (float)p.y/(corners.rows-1) - 0.5 );

				float c = cos( elem.rotation ), s = sin( elem.rotation );
				Point2f rotloc(  c*xyloc.x + s*xyloc.y + elem.center.x,
												-s*xyloc.x + c*xyloc.y + elem.center.y );

				// std::cout << rotloc << " " << p << std::endl;
				out.add( rotloc, i, p, corners.at<unsigned char>(p), elem.rotation );
			}
		}

	}

	return out;
}

Mat TagArray::draw(  cv::Mat &mat, const Size tagSize )
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

	const cv::Scalar backgroundColor( 255 );
	mat.create( imgSize, CV_8UC1 );
	mat = backgroundColor;												// Fill with white

	// Draw each element in turn
	for( std::vector< ArrayElement >::const_iterator itr = _elements.begin(); itr != _elements.end(); ++itr  ) {
		Mat elem( Corners::drawTagMat( _code, itr->code, tagSize, 1, 0 ) );

		// Create an affine transform
		Mat trans = (Mat_<double>(3,3) << cos( itr->rotation ), sin( itr->rotation), (itr->center.x-bb.x)*tagSize.width,
																      -sin( itr->rotation), cos( itr->rotation), (itr->center.y-bb.y)*tagSize.height,
																			0,0,1);
		Mat offset = (Mat_<double>(3,3) << 1,0, -0.5*tagSize.width,
																       0,1, -0.5*tagSize.height,
																			 0,0,1);

		warpPerspective( elem, mat, trans*offset, imgSize, INTER_LINEAR, BORDER_TRANSPARENT, backgroundColor );
	}

	return mat;
}


Rect_<float> TagArray::boundingBox( void ) const
{
	// Calculate the inefficient way
	std::vector< Point2f > corners;

	const Point2f cor[4] = { Point2f( 0.5, 0.5 ),
													Point2f( 0.5, -0.5 ),
													Point2f( -0.5, -0.5 ),
													Point2f( -0.5, 0.5 ) };


	for( std::vector< ArrayElement >::const_iterator itr = _elements.begin(); itr != _elements.end(); ++itr  ) {
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
