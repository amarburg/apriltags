

#include "CornerDetectorTypes.h"


namespace AprilTags {

//===================================================================

// How to set radius?
Intersection::Intersection( const cv::Point2f &c, float basis0, float basis1, const Mat &img )
	: center( c )
{
	basis[0] = ZeroTwoPi(basis0);// + 8*M_PI;
	basis[1] = ZeroTwoPi(basis1);// + 8*M_PI;

	// // Ensure ordering
	// if( basis[1] < basis[0] ) std::swap( basis[1], basis[0]);
	//
	// while( basis[0] > 2*M_PI ) {
	// 	basis[0] -= 2*M_PI;
	// 	basis[1] -= 2*M_PI;
	// }

	// while( basis[1] - basis[0] > M_PI ) basis[1] -= M_PI;

	// // Rotate until the basis[0]-basis[1] is acute (or right)
	// while( basis[1] - basis[0] > M_PI/2 ) {
	// 	basis[0] += M_PI;
	// 	swap(basis[1], basis[0]);
	// }

	// Estimate transform
	estimateTransform();

	sample( img );
}

float Intersection::meanAngle( void ) const
{
	return atan2( sin(basis[0])+sin(basis[1]), cos(basis[0])+cos(basis[1]));
}

float Intersection::includedAngle( void ) const
{
	// TODO:  More robust algorithm
	float dt = basis[1] - basis[0];
	while( dt > M_PI ) dt = 2*M_PI - dt;
	return dt;
}

cv::Point2f Intersection::transform( const cv::Point2f &in ) const
{
	cv::Vec3f o( _transform * cv::Vec3f( in.x, in.y, 1 ));
	return cv::Point2f( o[0]/o[2], o[1]/o[2]);
}

cv::Point2f Intersection::absTransform( const cv::Point2f &in ) const
{
	return center + transform( in - center );
}

cv::Point2f Intersection::invTransform( const cv::Point2f &in ) const
{
	cv::Vec3f o( _transform.inv() * cv::Vec3f( in.x, in.y, 1 ));
	return cv::Point2f( o[0]/o[2], o[1]/o[2]);
}

cv::Point2f Intersection::absInvTransform( const cv::Point2f &in ) const
{
	return center + invTransform( in );
}

void Intersection::estimateTransform( void )
{
	// Optimize this later.  Do it by composition
	Matx33f rot( cos(meanAngle()), sin(meanAngle()), 0,
							 -sin(meanAngle()), cos(meanAngle()), 0,
							0,0,1);

	// This presumes basis[1] > basis[0]
	Matx33f scale(1,0,0,0,1,0,0,0,1);

	if( includedAngle() > M_PI/2.0 )
		scale = Matx33f( tan( includedAngle()/2 ), 0, 0,
											0, 1, 0,
											0, 0, 1 );
	else
		scale = Matx33f( 1, 0, 0,
											0, 1.0/tan( includedAngle()/2 ), 0,
											0, 0, 1 );

	_transform = rot.inv() * scale * rot;
}

void Intersection::sample( const Mat &img, float radius )
{
	int neighborhood( 2*radius + 5 );
	CV_Assert( neighborhood % 2 == 1 );

	// float theta = meanAngle();
	// theta defines the mean angle between the x and y axes

	// Generate locally warped version
	Mat ptRoi( img, cv::Rect( center.x - (neighborhood-1)/2, center.y - (neighborhood-1)/2, neighborhood, neighborhood ) );
	Mat warped( neighborhood, neighborhood, ptRoi.type() );
	Matx33f offset( 1, 0, -center.x,
									0, 1, -center.y,
									0, 0, 1 );
	Matx33f offset2( 1, 0, neighborhood*0.5,
									0, 1, neighborhood*0.5,
									0,0,1);

	cv::warpPerspective( img, warped, offset2 * _transform * offset, cv::Size( neighborhood, neighborhood ), cv::INTER_LINEAR, cv::BORDER_REPLICATE );

	// Mat foo(100,200,CV_32FC1);
	// Mat left( foo, cv::Rect(0,0,100,100));
	// Mat right( foo, cv::Rect(100,0,100,100));
	// cv::resize( ptRoi, left, cv::Size(100,100));
	// cv::resize( warped, right, cv::Size(100,100) );
	// imshow( "warped", foo );
	// cv::waitKey();

	// double angles[4] = { theta + M_PI, theta - M_PI/2, theta + M_PI/2, theta };
	for( unsigned int i = 0; i < 4; ++i ) {
		float angle = Corners::angles[i] + basis[0];
		cv::Point2f pt( cv::Point2f(neighborhood*0.5, neighborhood*0.5) + radius * cv::Point2f( cos( angle), sin(angle) ) );
		Mat ptRoi( warped, cv::Rect( pt.x-1, pt.y-1, 3, 3 ));

		samples[i] = cv::mean( ptRoi )[0];
	}
}

void Intersection::identify( const BinaryClassifier &classifier )
{
	unsigned char bits = 0;
	for( unsigned int i = 0; i < 4; ++i ) {
		if( classifier.classify( samples[i] ) == 1 )
			bits |= (1 << i);
	}

	corner = Corners::cornerLUT( bits );
}


//===================================================================

Triangle::Triangle( const std::shared_ptr< Intersection > &v0,
										const std::shared_ptr< Intersection > &v1,
										const std::shared_ptr< Intersection > &v2 )
{
	_vertices[0] = v0;
	_vertices[1] = v1;
	_vertices[2] = v2;

	_neighbors[0] = NULL;
	_neighbors[1] = NULL;
	_neighbors[2] = NULL;
}

bool Triangle::hasEdge( const std::shared_ptr< Intersection > &a, const std::shared_ptr< Intersection > &b )
{
	if( (_vertices[0] == a || _vertices[1] == a || _vertices[2] == a) &&
			(_vertices[0] == b || _vertices[1] == b || _vertices[2] == b) ) return true;

	return false;
}

void Triangle::addNeighbor( unsigned int i, std::shared_ptr< Triangle > &tri )
{
	if( i >= 3 ) return;
	_neighbors[i] = tri;
}

float Triangle::area( void ) const
{
	float ab = MathUtil::distance2D( _vertices[0]->center, _vertices[1]->center ),
				bc = MathUtil::distance2D( _vertices[1]->center, _vertices[2]->center ),
				ca = MathUtil::distance2D( _vertices[2]->center, _vertices[0]->center );
	float s = 0.5 * (ab + bc + ca );

	return sqrt( s * (s-ab) * (s-bc) * (s-ca) );
}

cv::Point2f Triangle::center( void ) const
{
	return cv::Point2f( (_vertices[0]->center.x + _vertices[1]->center.x + _vertices[2]->center.x) / 3.0,
											(_vertices[0]->center.y + _vertices[1]->center.y + _vertices[2]->center.y) / 3.0 );
}

}
