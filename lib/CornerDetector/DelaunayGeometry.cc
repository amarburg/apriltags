#include <iostream>

#include <opencv2/highgui/highgui.hpp>

#include "DelaunayGeometry.h"

#include "Utils/MinMax.h"

namespace AprilTags {

	DelaunayGeometry::DelaunayGeometry( const cv::Size &sz )
		:	_delaunay( cv::Rect(0,0, sz.width,sz.height) ),
			_sz( sz ),
			_dirty( true ),
			_img(),
			_triangleImg()
	{
		;
	}

	DelaunayGeometry::DelaunayGeometry( const cv::Size &sz, const cv::Mat &img )
		:	_delaunay( cv::Rect(0,0, sz.width,sz.height) ),
			_sz( sz ),
			_dirty( true ),
			_img( img ),
			_triangleImg()
	{
		;
	}

	DelaunayGeometry::~DelaunayGeometry()
	{}

	void DelaunayGeometry::addVertex( Intersection *i )
	{
		_vertices.emplace_back( i );
		_delaunay.insert( i->center );
		_dirty = true;
	}

	void DelaunayGeometry::update( void )
	{
		if( !_dirty ) return;

		// Let's be inefficient and rebuild from scratch for now.   Why?
		// because this should only be called once;

		_triangles.clear();
		std::vector<cv::Vec6f> triangleList;
		_delaunay.getTriangleList( triangleList );
		_triangles.reserve( triangleList.size() );

		for( auto const &tri : triangleList ) {
			// Drop triangles which contain points outside the image (those on edge)
			// Check for any outside vertices outside the image (those on the edges)
			if( tri[0] < 0 || tri[2] < 0 || tri[4] < 0 ||
					tri[0] >= _sz.width || tri[2] >= _sz.width || tri[4] >= _sz.width ||
					tri[1] < 0 || tri[3] < 0 || tri[5] < 0 ||
					tri[1] >= _sz.height || tri[3] >= _sz.height || tri[5] >= _sz.height ) continue;

			vector< std::shared_ptr< Intersection > > vt;

			for( int i = 0; i < 3; ++i  ) {
				for( auto &vertex : _vertices ) {
					if( vertex->center.x == tri[2*i] && vertex->center.y == tri[(2*i)+1] ) {
						vt.push_back( vertex );
						break;
					}
				}
			}

			if( vt.size() == 3 )
				_triangles.emplace_back( new Triangle(vt[0], vt[1], vt[2]) );
		}

		// Now build up the neighbors
		for( auto &tri : _triangles ) {
			for( int i = 0; i < 3; ++i ) {
				const std::shared_ptr< Intersection > &a( tri->vertex(i) ),
																							&b( tri->vertex( (i==2) ? 0 : i+1) );

				// Do the full N-by-N search to ensure links are built in both
				// directions.
				for( auto &itr : _triangles ) {
					if( itr == tri ) continue;
					if( itr->hasEdge( a, b ) ) {
						tri->addNeighbor(i,  itr);
						break;
					}
				}
			}
		}


		_dirty = false;
	}

	void DelaunayGeometry::buildMesh( void )
	{
		update();

		// For now, just choose an arbitrary starting point.
		// Choose a triplet to start with, biased towards smaller end of distribution
		std::sort( _triangles.begin(), _triangles.end(),
										[]( const shared_ptr<Triangle> &a, const shared_ptr<Triangle> &b ){ return a->area() > b->area(); } );

		int idx = _triangles.size() * 0.25;
		const shared_ptr<Triangle> &a( _triangles[idx] );

		_mesh.clear();
		_visited.clear();

		_mesh[ a->vertex(0) ] = cv::Point2f( 0, 0 );
		_mesh[ a->vertex(1) ] = cv::Point2f( 1, 0 );

		projectTriangle( a );
	}

	void DelaunayGeometry::projectTriangle( const shared_ptr< Triangle > &tri )
	{
		if( tri.get() == NULL ) return;
		if( _visited.find(tri) != _visited.end() ) return;
		_visited[ tri ] = true;

		if( _mesh.find( tri->vertex(0) ) == _mesh.end() ) project( tri, 0 );
		else if( _mesh.find( tri->vertex(1) ) == _mesh.end() ) project( tri, 1 );
		else if( _mesh.find( tri->vertex(2) ) == _mesh.end() ) project( tri, 2 );

		// std::cout << "Iterating to " << tri->neighborCount() << " neighbors" << std::endl;

		// Iterate to neighbors
		for( int i = 0; i < 3; ++i )
			if( tri->neighbor(i).get() != NULL )
				projectTriangle( tri->neighbor(i) );

		// projectTriangle( tri->neighbor(0) );
		// projectTriangle( tri->neighbor(1) );
		// projectTriangle( tri->neighbor(2) );
	}

	static cv::Point2f projectPoint( const shared_ptr<Intersection> &aInt,
																		const shared_ptr<Intersection> &bInt,
																		const shared_ptr<Intersection> &tInt,
																		const cv::Point2f &aPt,
																		const cv::Point2f &bPt )
	{
		// For point a, warp point b and c by homography.  Find similarity
		// which maps a->warped B to their mesh points.  Use this to estimate
 		// location of point t

		cv::Point2f aWarped( aInt->center ),
							  bWarped( aInt->absTransform( bInt->center ) ),
								tWarped( aInt->absTransform( tInt->center ) );

	// std::cout << "aInt:    " << aInt->center << std::endl;
	// std::cout << "bInt:    " << bInt->center << std::endl;
	// std::cout << "bWarped: " << bWarped << std::endl;
	//
	// std::cout << "tInt:    " << tInt->center << std::endl;
	// std::cout << "tWarped: " << tWarped << std::endl;
	//
	// std::cout << "aPt:     " << aPt << std::endl;
	// std::cout << "bPt:     " << bPt << std::endl;

		// Estimate similarity
		cv::Matx41f b( aPt.x, aPt.y, bPt.x, bPt.y );
		cv::Matx44f A( aWarped.x, aWarped.y, 1, 0,
							 aWarped.y, -aWarped.x, 0, 1,
							 bWarped.x, bWarped.y, 1, 0,
							 bWarped.y, -bWarped.x, 0, 1 );
		cv::Matx41f ans( A.inv()*b ); // [ kcos(theta), ksin(theta), t_x, t_y ]
		float kc = ans(0,0), ks = ans(1,0);
		cv::Point2f t( ans(2,0), ans(3,0) );

//		std::cout << "Soln: " << ans << std::endl;

		return cv::Point2f( kc*tWarped.x + ks*tWarped.y + t.x,
 										   -ks*tWarped.x + kc*tWarped.y + t.y );
}

	void DelaunayGeometry::project( const shared_ptr<Triangle> &tri, unsigned int idx )
	{
		unsigned int aIdx = ( (idx == 0 ) ? 1 : 0 );
		unsigned int bIdx = ( (idx == 2 ) ? 1 : 2 );

		assert( aIdx != bIdx );

		const shared_ptr<Intersection> &aInt( tri->vertex(aIdx) ),
																	 &bInt( tri->vertex(bIdx) ),
																	 &tInt( tri->vertex(idx) );
		const cv::Point2f aPt( _mesh[ aInt ] ), bPt( _mesh[bInt] );

		cv::Point2f tGivenA( projectPoint( aInt, bInt, tInt, aPt, bPt )),
								tGivenB( projectPoint( bInt, aInt, tInt, bPt, aPt ));

		// std::cout << "tGivenA: " << tGivenA << std::endl;
		// std::cout << "tGivenB: " << tGivenB << std::endl;

		if( !_img.empty() ) {
			if( _triangleImg.empty() ) _img.copyTo( _triangleImg );

			Mat imageOut;
			_triangleImg.copyTo( imageOut );

			const cv::Scalar red( 0, 0, 255 ), blue( 255,0,0), green( 0, 255,0), black(0,0,0);
			cv::line( imageOut, aInt->center, bInt->center, red, 3 );
			cv::line( imageOut, bInt->center, tInt->center, red, 3 );
			cv::line( imageOut, tInt->center, aInt->center, red, 3 );

			cv::line( _triangleImg, aInt->center, bInt->center, black, 3 );
			cv::line( _triangleImg, bInt->center, tInt->center, black, 3 );
			cv::line( _triangleImg, tInt->center, aInt->center, black, 3 );

			cv::putText( imageOut, "A", aInt->center + cv::Point2f(10,-10), cv::FONT_HERSHEY_SIMPLEX, 1, red);
			cv::putText( imageOut, "B", bInt->center + cv::Point2f(10,-10), cv::FONT_HERSHEY_SIMPLEX, 1, red);
			cv::putText( imageOut, "T", tInt->center + cv::Point2f(10,-10), cv::FONT_HERSHEY_SIMPLEX, 1, red);

			cv::circle( imageOut, aInt->transform( bInt->center ), 3, blue, -1);
			cv::circle( imageOut, aInt->transform( tInt->center ), 3, green, -1);

			Mat imOut( cv::Size( imageOut.cols*0.5, imageOut.rows*0.5), imageOut.type());
			cv::resize( imageOut, imOut, imOut.size() );
			cv::imshow( "Image", imOut );

			// Now find the bounding box on mesh
			PointMinMax mm;
			for( auto const &p : _mesh ) {
				mm.check( p.second );
			}
			mm.check( tGivenA );
			mm.check( tGivenB );

			float w( mm.x.max-mm.x.min ), h( mm.y.max - mm.y.min );
			float scale = 700 / std::max( h, w );
			cv::Point2f origin( mm.x.min, mm.y.min );
			cv::Point2f offset( 50,50);
			Mat meshOut( cv::Size( 800, 800 ), CV_32FC3, cv::Scalar(255,255,255) );

			for( auto const &p : _mesh ) {
				cv::Scalar color(0,0,0);
				if( p.first == aInt )
					color = red;
				else if( p.first == bInt )
					color = blue;
				else if( p.first == tInt )
					color = green;

				cv::circle( meshOut, scale*(p.second-origin) + offset, 5, color, -1 );
			}

			cv::circle( meshOut, scale*(tGivenA-origin) + offset, 5, red, 2 );
			cv::circle( meshOut, scale*(tGivenB-origin) + offset, 5, blue, 2 );

			imshow("Mesh", meshOut );

			cv::waitKey(10);
		}

		// Check for agreement

		_mesh[ tInt ] = cv::Point2f( ( tGivenA.x + tGivenB.x ) * 0.5,
																 ( tGivenA.y + tGivenB.y ) * 0.5 );

	}





}
