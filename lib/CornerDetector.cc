
#include <iostream>

#include "AprilTags/CornerDetector.h"

#include "Utils/BinaryClassifier.h"
#include "Utils/ImageMasks.h"
#include "Utils/MinMax.h"

#include "Geometry/LineSegment.h"
#include "CornerDetector/CornerDetectorTypes.h"
#include "CornerDetector/DelaunayGeometry.h"

#include <opencv2/highgui/highgui.hpp>


namespace AprilTags {

using cv::Mat;
using cv::Matx33f;

//=========================================================


	static bool angleCheck( float thetaA, float thetaB )
	{
		// Calculate included angle between segments (smallest value 0 < < PI/2)
		float angle = thetaA - thetaB;
		while( angle < 0 ) { angle += M_PI; }
		while( angle >= M_PI) { angle -= M_PI; }

		return ( angle > M_PI/6 && angle < 5*M_PI/6 );
	}

// static void doProject( CornerArray &corners,
// 							std::map<  Intersection &, unsigned int > &map,
// 						 Triplet &triplet,
// 							const Intersection &one, const Intersection &two,
// 							const Intersection &target )
// {
// // 	const CornerArray::ArrayElement &cornerOne( map[ one ] ),
// // 							&cornerTwo( map[two] );
//
// 	// Calculate similarity that takes one->two and maps it to cornerOne->cornerTwo;
// 	// Apply mean affine transform for the triplet, project target
// }

// void project( CornerArray &corners,
// 							std::map<  Intersection &, unsigned int > &map,
// 						 Triplet &triplet )
// {
// 	// // Find which one doesn't exist in the map, abort if anything other
// 	// // than 2 are already in the map.
// 	// auto aitr = map.find( triplet._a ), bitr = map.find( triplet._b ), citr = map.find( triplet._c );
// 	// Intersection *one, *two, *target;
// 	//
// 	// if( aitr != map.end() && bitr != map.end() ) {
// 	// 	doProject( corners, map, triplet, *aitr, *bitr, *citr );
// 	// } else if( bitr != map.end() && citr != map.end() ) {
// 	// 	doProject( corners, map, triplet, *bitr, *citr, *aitr );
// 	// } else if( citr != map.end() && aitr != map.end() ) {
// 	// 	doProject( corners, map, triplet, *citr, *aitr, *bitr );
// 	// } else {
// 	// 	return;
// 	// }
// }

//=========================================================

CornerDetector::CornerDetector( void )
       : DetectorBase( CORNER_DEBUG_IMAGES )
{;}

CornerDetectionArray CornerDetector::detect( const Mat &inImage, const CornerArray &array )
{
	// Calidate input
	CV_Assert( !inImage.empty() );
	CV_Assert(  inImage.channels() == 1 );
	CV_Assert( (inImage.type() == CV_32FC1) || (inImage.type() == CV_8UC1) );

	int width = inImage.size().width;
	int height = inImage.size().height;

	CornerDetectionArray output;

	// Gaussian blur
	Mat blurredImage;
	doImagePrep( inImage, blurredImage );

	// Line and corner detection
	Mat magnitude, angle;
	calculateGradient( blurredImage, magnitude, angle );

	std::vector<Segment> segments;
	fitSegments( magnitude, angle, segments );

	// Find segment pairs with endpoints which are close together and an
	// included angle which is far from either 0 or M_PI

	//DelaunayGeometry delaunay( cv::Size( width, height ), debugImage( OriginalBGRImage ) );
	DelaunayGeometry delaunay( cv::Size( width, height ) );


	int minDistance = 1 * min( 1e-2 * width, 1e-2 * height );
	int minDistanceSq = minDistance * minDistance;

	const float imgMaskRadius = 5;
	ImageMask imgMask( inImage.size(), imgMaskRadius );
	Mask segMask( segments.size(), segments.size() );

  // For now, just brute force it
  for (unsigned i = 0; i < segments.size(); i++) {
		for (unsigned j = i; j < segments.size(); j++) {

			if( !segMask.check( i,j ) ) continue;
			if( !angleCheck( segments[i].getTheta(), segments[j].getTheta() ) ) continue;

			LineSegment aye( segments[i] ), jay( segments[j] );

			for( int z = 0; z < 4; ++z ) {

				if( MathUtil::distance2Dsqr( aye.a(), jay.a() ) < minDistanceSq ) {
					cv::Point2f mean( MathUtil::meanPoint( aye.a(), jay.a() ) );
					if( !imgMask.check(mean) ) continue;

					imgMask.mark( mean );
					segMask.mark( i,j );

					delaunay.addVertex( new Intersection( mean, aye.theta(), jay.theta(), blurredImage ) );
					break;
				}

				jay = jay.flip();
				if( z == 1 ) aye = aye.flip();

			}

    }
  }

	// Use samples from all intersections to train 2-class GMM
	BinaryClassifier classifier( 1.0 );
	classifier.startTraining();

	// And while we're at it, build the Delaunay to compute local triads

	for( auto const &intersection : delaunay.vertices() )
		for( int i = 0; i < 4; ++i )
			classifier.trainingPoint( intersection->samples[i] );

	classifier.endTraining();

	for( auto &intersection : delaunay.vertices() )
		intersection->identify( classifier );

	std::cout << delaunay.vertices().size() << " intersections" << std::endl;
	if( _saveDebugImages ) saveIntersectionImage( delaunay.vertices() );


	std::cout << delaunay.triangles().size() << " Delaunay triangles" << std::endl;


	if( _saveDebugImages ) saveTriangleImage( delaunay.triangles() );


	delaunay.buildMesh();


	if( _saveDebugImages) saveMeshImage( delaunay.mesh() );

	// Now attempt to match mesh to CornerArray

	// Ransac like-approach?
	attemptMatch( array, delaunay.mesh() );

	return output;
}


void CornerDetector::attemptMatch( const CornerArray &array, const std::map< shared_ptr< Intersection >, cv::Point2f > &mesh )
{


}


//=== Debugging functions ====

void CornerDetector::saveIntersectionImage( const vector< shared_ptr<Intersection> > &intersections )
{
	Mat intersectionsImage;
	// Boost back to color imagery for better annotation
	debugImage( OriginalBGRImage ).copyTo( intersectionsImage );

	CV_Assert( intersectionsImage.type() == CV_32FC3 );

	for( auto const &intersection : intersections  ) {
		long r = random();
		cv::Scalar color( (float)(r & 0xFF)/255, (float)((r&0xFF00)>>8)/255, (float)((r&0xFF0000)>>8)/255 );
		float radius = float(intersectionsImage.rows + intersectionsImage.cols)/2 * 0.01;

		// Draw an ellipse
		for( float theta = 0; theta < (2*M_PI+0.1); theta += 0.1 ) {
			cv::Point2f start( radius * cos( theta ), radius * sin(theta )),
									end( radius * cos( theta + 0.1 ), radius * sin(theta + 0.1));
			cv::line( intersectionsImage,
								intersection->absInvTransform(start ),
								intersection->absInvTransform( end ),
								color, 2 );
		}

		// cv::circle( intersectionsImage, intersection.center,
		// 					radius, color, 2 );

		cv::line( intersectionsImage, intersection->center,
							intersection->center + cv::Point2f( radius * cos( intersection->basis[0] ), radius * sin( intersection->basis[0] ) ),
							color, 2 );
		cv::line( intersectionsImage, intersection->center,
							intersection->center + cv::Point2f( radius * cos( intersection->basis[1] ), radius * sin( intersection->basis[1] ) ),
							color, 2 );

		char buf[10];
		snprintf( buf, 10, "%02x", intersection->corner );
		cv::putText( intersectionsImage, buf, intersection->center + cv::Point2f(10,-10), cv::FONT_HERSHEY_SIMPLEX, 2, color);

		float theta = intersection->meanAngle();
		double angles[4] = { theta + M_PI, theta - M_PI/2, theta + M_PI/2, theta };
		for( unsigned int j = 0; j < 4; ++j ) {
			if( intersection->corner & (1 << j) ) {
				int fill = (j==3) ? -1 : 2;
				cv::circle( intersectionsImage, intersection->absInvTransform(radius * cv::Point2f( cos( angles[j] ), sin( angles[j] ) )),
									8, color, fill );
			}
		}


	}

	saveDebugImage( intersectionsImage, IntersectionImage, false );
}


void CornerDetector::saveTriangleImage( const vector< shared_ptr<Triangle> > &triangles )
{
	Mat tripletImage;
	// Boost back to color imagery for better annotation
	debugImage( OriginalBGRImage ).copyTo( tripletImage );

	CV_Assert( tripletImage.type() == CV_32FC3 );

	for( auto const &tri : triangles ) {
		long r = random();
		cv::Scalar color( (float)(r & 0xFF)/255, (float)((r&0xFF00)>>8)/255, (float)((r&0xFF0000)>>8)/255 );

		cv::line( tripletImage, tri->vertex(0)->center, tri->vertex(1)->center,
							color, 2 );
		cv::line( tripletImage, tri->vertex(1)->center, tri->vertex(2)->center,
							color, 2 );
		cv::line( tripletImage, tri->vertex(2)->center, tri->vertex(0)->center,
							color, 2 );

		// std::cout << "Triangle has " << tri->neighborCount() << " neighbors" << std::endl;
 	// 	for( int i = 0; i < 3; ++i )
		// 	if( tri->neighbor(i).get() != NULL )
		// 		cv::line( tripletImage, tri->center(), tri->neighbor(i)->center(),
		// 						cv::Scalar(0,0,255), 2 );


	}

	saveDebugImage( tripletImage, TriangleImage, false );
}

void CornerDetector::saveMeshImage( const map< shared_ptr< Intersection >, cv::Point2f > &mesh )
{
	// Now find the bounding box on mesh
	PointMinMax mm;
	for( auto const &p : mesh ) {
		mm.check( p.second );
	}

	float w( mm.x.max-mm.x.min ), h( mm.y.max - mm.y.min );
	float scale = 700 / std::max( h, w );
	cv::Point2f origin( mm.x.min, mm.y.min );
	cv::Point2f offset( 50,50);
	Mat meshImage( cv::Size( 800, 800 ), CV_32FC3, cv::Scalar(255,255,255) );

	for( auto const &p : mesh ) {
		cv::Scalar color(0,0,0);
		cv::circle( meshImage, scale*(p.second-origin) + offset, 5, color, -1 );
	}

	saveDebugImage( meshImage, MeshImage, false );
}


}
