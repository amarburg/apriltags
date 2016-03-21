

#include "CornerDetectorTypes.h"


namespace AprilTags {

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
