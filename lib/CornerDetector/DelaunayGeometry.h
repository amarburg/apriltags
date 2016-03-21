#pragma once


#include <opencv2/core/core.hpp>

#include "CornerDetectorTypes.h"

namespace AprilTags {


class DelaunayGeometry {
public:

	DelaunayGeometry( const cv::Size &sz  );
	DelaunayGeometry( const cv::Size &sz, const Mat &img  );
	~DelaunayGeometry();

	void addVertex( Intersection *i );

	std::vector< std::shared_ptr< Intersection > > &vertices( void )
		{ return _vertices; }

	const std::vector< std::shared_ptr< Intersection > > &vertices( void ) const
		{ return _vertices; }

	std::vector< std::shared_ptr< Triangle > > &triangles( void )
		{ update(); return _triangles; }

	void buildMesh( void );

protected:

	void update( void );

	void project( const shared_ptr<Triangle> &tri, unsigned int idx );
	void projectTriangle( const shared_ptr< Triangle > &tri );


	void drawDebug( void );

	cv::Subdiv2D _delaunay;

	std::vector< std::shared_ptr< Triangle > > _triangles;
	std::vector< std::shared_ptr< Intersection > > _vertices;

	map< shared_ptr< Intersection >, cv::Point2f > _mesh;
	map< shared_ptr< Triangle >, bool > _visited;

	cv::Size _sz;
	bool _dirty;

	cv::Mat _img, _triangleImg;
};

}
