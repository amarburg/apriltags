
#include <iostream>
#include <unordered_set>

#include "AprilTags/CornerDetector.h"

#include "Utils/BinaryClassifier.h"
#include "Utils/ImageMasks.h"
#include "Utils/MinMax.h"
#include "Utils/MathUtil.h"

#include "Utils/range.hpp"

#include "Apriltags/Corners.h"

#include "Geometry/LineSegment.h"
#include "CornerDetector/CornerDetectorTypes.h"
#include "CornerDetector/DelaunayGeometry.h"

#include "CornerDetector/MatchHypothesis.h"

#include <opencv2/highgui/highgui.hpp>


namespace AprilTags {

using cv::Mat;
using cv::Matx33f;

using std::shared_ptr;
using std::vector;

using util::lang::range;

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
	attemptMatch( array, delaunay );

	return output;
}



void CornerDetector::attemptMatch( const CornerArray &array, const DelaunayGeometry &delaunay )
{
	// Choose an origin intersections at random and look for potential matches in array

	unsigned int originIdx( random() % delaunay.vertices().size() );
	const shared_ptr< Intersection > &origin( delaunay.vertex(originIdx) );


	unsigned int secondIdx( random() % delaunay.vertices().size() );
	const shared_ptr< Intersection > &second( delaunay.vertex(secondIdx) );

	// std::cout << " basis0: " << second->basis[0] * 180.0/M_PI
	// 		 					<< " basis1: " << second->basis[1] * 180.0/M_PI
	// 							 						<< std::endl;

	//multimap< float, RootHypothesis > roots;

	OriginPoint originPoint( origin );
	Point &secondPoint( originPoint.addSecondPoint( second ) );

	// For now, just go linearly
	std::vector< std::pair< int, int > > originHypotheses( array.spinMatch( origin->corner ));
	// std::cout << "Found " << originHypotheses.size() << " possible matches for origin." << std::endl;

	for( auto const &p : originHypotheses ) {
		// Calculate angular conversion from origin to array frame
		//const float theta = M_PI/2.0 * p.second - array[ p.first ].rotation;
		//const float phi = origin->basis[0] + theta;

		// std::cout << p.first << " " << array[p.first].center << " "
		// 					<< std::hex << array[p.first].cornerType << std::dec
		// 					<< " spin: " << p.second
 	// 						<< " rot: " << array[ p.first ].rotation * 180.0/M_PI
		// 					<< " basis0: " << origin->basis[0] * 180.0/M_PI
		// 					<< " basis1: " << origin->basis[1] * 180.0/M_PI
 	// 						<< std::endl;

		originPoint.addHypothesis( array[p.first], p.second );



		// Maintain only 1000 hypotheses...
		// const unsigned int maxHypotheses = 100;
		// if( secondScored.size() > maxHypotheses ) {
		// 	// Drop 90% of the elements in second
		// 	multimap< float, MatchHypothesis >::iterator itr = secondScored.begin();
		// 	for( unsigned int i = 0; i < maxHypotheses; ++i ) ++itr;
		// 	secondScored.erase( itr, secondScored.end() );
		// }

	}

	std::vector< std::pair< int, int > > secondHypotheses( array.spinMatch( second->corner ));
	std::cout << secondHypotheses.size() << " second hypotheses" << std::endl;
	for( auto const &sec : secondHypotheses ) {
		secondPoint.addHypothesis( array[sec.first], sec.second );
	}


	// Created ordered set of points to evaluate radiating from second
	std::deque< std::shared_ptr< Intersection > > pointQueue, workQueue;
	std::unordered_set< std::shared_ptr< Intersection > > neighbors;
	neighbors.emplace( origin );
	neighbors.emplace( second );
	workQueue.push_back( second );

	while( !workQueue.empty() ) {
		const std::shared_ptr< Intersection > &inter( workQueue.front() );
		workQueue.pop_front();

		// Start to evaluate other points.  Start with Delaunay neighbors of second
		const std::vector< std::shared_ptr< Triangle > > &neighborTri( delaunay.membership( inter ) );
		//std::cout << "In " << neighborTri.size() << " triangles" << std::endl;

		// C++11-ism
		for( auto const &tri : neighborTri ) {
			for( unsigned int i = 0; i < 3; ++i )
				if( (neighbors.emplace( tri->vertex(i) )).second ) {
					workQueue.emplace_back( tri->vertex(i) );
					pointQueue.emplace_back( tri->vertex(i) );
				}
		}
	}

	// Mat debugImg;
	// debugImage( OriginalBGRImage ).copyTo( debugImg );
	// cv::circle( debugImg, origin->center, 10, cv::Scalar(0,0,255), -1 );
	// cv::circle( debugImg, second->center, 10, cv::Scalar(0,255,0), -1 );
	// for( auto const &pt : pointQueue ) {
	// 	cv::circle( debugImg, pt->center, 5, cv::Scalar(255,0,0), -1 );
	//
	// 	Mat fooImg;
	// 	cv::resize( debugImg, fooImg, cv::Size( debugImg.cols/4, debugImg.rows/4));
	// 	imshow( "PointQueue", fooImg );
	// 	cv::waitKey( 20 );
	// }



	std::cout << "Point queue contains " << pointQueue.size() << " of " << delaunay.vertices().size() << " vertices" << std::endl;

	// Could do this automatically (eager-load?) but do it explicitly so I know
	// when the expensive calculations are happening.
	originPoint.evaluateInitialHypotheses();
	std::cout << "Considering " << originPoint.eval.size() << " hypothesis combinations." << std::endl;

	originPoint.keepInitialHypotheses( 1000 );
	std::cout << "Keeping only the first " << originPoint.eval.size() << " combinations." << std::endl;

	unsigned long numEvalPoints = 1e9;
	numEvalPoints = std::min(numEvalPoints,pointQueue.size());


for( auto &ePair : originPoint.eval ) {
	InitialHypothesis &hyp( ePair.second );
	//double error = ePair.first;

//
//
// 		// std::cout << "Spins: " << hyp.oSpin << " " << hyp.sSpin << std::endl;
// 		//
// 		// std::cout << "Second basis " << hyp.second->basis[0]*180.0/M_PI << " in array  " << hyp.secondBasisInArray()*180.0/M_PI << std::endl;
// 		// std::cout << "Second theta " << hyp.secondTheta()*180.0/M_PI << std::endl;
// 		//
// 		// std::cout << "Score: " << pear.first << "  angular error: " << hyp.angularError() << "   spin error: "  << hyp.spinError()*180.0/M_PI << std::endl;
//
// 		// Calculate the similarity given origin and second from intersection space
// 		// to CornerArray space
//
// 		// Evaluate cost function
// 		//
		//bool doAbort = false;
// //		for( auto const &pt : pointQueue ) {
 		for( auto i : range(0ul,numEvalPoints) ) {
			const shared_ptr< Intersection > &pt( pointQueue[i] );

			EvalPoint &evalPt( hyp.addPoint( pt ) );
			std::vector< std::pair< int, int > > potentialHypotheses(array.spinMatch( evalPt.intersection->corner, evalPt.cornerPred, 0.2 ));

			// Can't find a match?   Just abort.
			if( potentialHypotheses.size() == 0 ) {
				// This API puts state into the InitialHypothesis, which I don't like
				// but it works for now.
				hyp.abortPoint();
				//break;
			}

			// std::cout << "This point has " << potentialHypotheses.size() << " potential hypotheses" << std::endl;
			//

			for( auto const &pHyp : potentialHypotheses ) {
				evalPt.addHypothesis( array[pHyp.first], pHyp.second );
			}


// 			Mat arrayImg;
// 			CornerArrayImage arrayImage( array.draw( arrayImg, cv::Size(250,250) ) );
// 			float radius = 25;
// 			// cv::circle( originalImage, evalPt.intersection->center, radius, green, -1 );
// 			cv::circle( arrayImage.mat, arrayImage(evalPt.cornerPred), radius, cv::Scalar(0,255,0), 3 );
// 			cv::line( arrayImage.mat, arrayImage(evalPt.cornerPred),
// 								arrayImage( evalPt.cornerPred + radius * RotPoint(evalPt.intersection->basis[0] + hyp.originHyp.theta()) ),
// 								cv::Scalar(0,255,0), 3 );
//
// 			const Hypothesis &hyp( evalPt.hypothesis(0) );
// 			cv::circle( arrayImage.mat, arrayImage(hyp.corner.center), radius, cv::Scalar(255,255,0), 3 );
// 			cv::line( arrayImage.mat,
// 								arrayImage(hyp.corner.center),
// 								arrayImage(hyp.corner.center + radius * RotPoint(hyp.basisInCorner()) ),
// 								cv::Scalar(255,255,0), 3 );
//
// 		std::cout << "Best match basis in corner: " << hyp.basisInCorner() << std::endl;
// std::cout << "Best match spins: " << hyp.spin << std::endl;
// std::cout << "Best rotation: " << hyp.corner.rotation << std::endl;
//
// 			for( auto const &i : range(1ul,evalPt.scored.size()) ) {
// 				const Hypothesis &thisHyp( evalPt.hypothesis(i));
// 				cv::circle( arrayImage.mat, arrayImage(thisHyp.corner.center), radius, cv::Scalar(128,255,255), 3 );
// 				cv::line( arrayImage.mat,
// 									arrayImage(thisHyp.corner.center),
// 									arrayImage(thisHyp.corner.center + radius * RotPoint(thisHyp.basisInCorner()) ),
// 									cv::Scalar(128,255,255), 3 );
//
// 			}
// 			Mat shrunkArr;
// 			cv::resize( arrayImage.mat, shrunkArr, cv::Size( arrayImage.mat.cols/2, arrayImage.mat.rows/2));
// 			cv::imshow( "potentialHypotheses", shrunkArr );
// 			cv::waitKey();

			// For now, discard all but the best match
			//evalPt.keepBestHypothesis();

		}

	}

	originPoint.reevaluateHypotheses( int(floor(0.9*numEvalPoints)) );

	if( originPoint.eval.size() == 0 ) {
		std::cout << "No surviving hypotheses.  Bummer." << std::endl;
		return;
	}

	std::cout << originPoint.eval.size() << " surviving hypotheses" << std::endl;

// 			// Match point to nearest point in array with correct type/spin
// 			cv::Point2f projMatx( hyp.sim * pt->center );
// 			// Predicted rotation??
// 			std::vector< std::pair< int, int > > potential(array.spinMatch( pt->corner, projMatx, 0.2 ));
//
// 			// For now, find just the best match
//
// 			// multimap< float, MatchHypothesis > potentialScored;
// 			for( auto const &pot : potential ) {
// 				hyp.addPoint( pt ); //, projMatx, array[pot.first], pot.second );
// 				// MatchHypothesis hyp( origin, second, array[ p.first ], array[ sec.first ], p.second, sec.second );
// 				// secondScored.insert( std::make_pair(hyp.totalError(), hyp )  );
// 			}
 		//}

		for( auto &ePair : originPoint.eval ) {
			InitialHypothesis &hyp( ePair.second );

		float totalError = hyp.totalError();
		std::cout << "Total error: " << totalError << endl;
		std::cout << "  hypothesis has " << hyp.evalPoints.size() << "/" << numEvalPoints << " (" << float(hyp.evalPoints.size())/numEvalPoints*100 << " pct) eval points " << std::endl;


		bool doDraw = true;
		if( doDraw ) {
			const cv::Scalar red( 0,0,255 ), blue(255,0,0), green(0,255,0);
			Mat originalImage, arrayImg;

			debugImage( OriginalBGRImage ).copyTo( originalImage );
			drawIntersection( originalImage, hyp.origin.intersection, red );
			drawIntersection( originalImage, hyp.second.intersection, blue );
			cv::line( originalImage, hyp.origin.intersection->center, hyp.second.intersection->center, blue, 2 );
			//cv::line( originalImage, hyp.second->center, hyp.second->center + 200 * cv::Point2f( cos( hyp.basis12Image() ), sin( hyp.basis12Image() )), cv::Scalar(255,255,0), 2 );

			CornerArrayImage arrayImage( array.draw( arrayImg, cv::Size(250,250) ) );
			float radius = 25;
			cv::Point2f ptCenter( arrayImage(hyp.originHyp.corner.center ) );
			cv::circle( arrayImage.mat, ptCenter, radius, red, 2 );
			cv::line( arrayImage.mat, ptCenter,
								ptCenter + radius * RotPoint( hyp.originHyp.basisInCorner() ),
								red, 3 );
			for( auto j : range(0,4) ) {
				if( origin->corner & (1 << j) ) {
					int fill = (j==3) ? -1 : 2;
					float angle = Corners::angles[j] + hyp.originHyp.basisInCorner();
					cv::circle( arrayImage.mat, arrayImage(hyp.originHyp.corner.center + 0.1 * RotPoint( angle )),
										8, red, fill );
				}
			}

			// Draw bearing to second
			cv::line( arrayImage.mat, ptCenter,
								ptCenter + 1000 * RotPoint( hyp.imgBearingInCorner() ),
								red, 2 );
			// cv::line( arrayImage.mat, ptCenter,
			// 					ptCenter + 50 * cv::Point2f( cos( secondBasis+theta ),  sin( secondBasis+theta ) ),
			// 					blue, 2 );

			cv::Point2f secondCenter( arrayImage(hyp.secondHyp.corner.center) );
			cv::circle( arrayImage.mat, secondCenter, radius, blue, 3 );

			// // Draw predicted basis
			// cv::line( arrayImage.mat, secondCenter,
			// 					secondCenter + 60 * cv::Point2f( cos( hyp.basisArray() ),  sin( hyp.basisArray() ) ),
			// 					green, 3 );
			cv::line( arrayImage.mat, secondCenter,
								secondCenter + 50 * RotPoint( hyp.secondHyp.basisInCorner() ),
								red, 3 );

			// Draw the first five test points
			for( auto const &evalPt : hyp.evalPoints ) {
				//const Hypothesis &evalHyp( evalPt.bestHypothesis() );
				// const shared_ptr< Intersection > &pt( pointQueue[i] );
				//

				cv::circle( originalImage, evalPt.intersection->center, radius, green, 3 );
				cv::line( originalImage, evalPt.intersection->center,
									evalPt.intersection->center + radius*RotPoint(evalPt.intersection->basis[0]), green, 3 );

				cv::circle( arrayImage.mat, arrayImage(evalPt.cornerPred), radius, green, 3 );
				cv::line( arrayImage.mat, arrayImage(evalPt.cornerPred),
									arrayImage( evalPt.cornerPred + 0.1 * RotPoint(evalPt.intersection->basis[0] + hyp.originHyp.theta()) ),
									green, 3 );


				cv::Point2f evalCenter( arrayImage(evalPt.hypothesis(0).corner.center) );
				cv::circle( arrayImage.mat, evalCenter, radius, cv::Scalar(128,128,255), 3 );
				cv::line( arrayImage.mat,
									evalCenter,
									evalCenter + 25 * RotPoint(evalPt.hypothesis(0).basisInCorner() ),
									cv::Scalar(128,128,255), 3 );

				// for( auto i : range(1ul,evalPt.scored.size()) ) {
				// 	const cv::Scalar blech( 255,128,128 );
				// 	const Hypothesis &badHyp( evalPt.hypothesis( i ) );
				// 	cv::circle( arrayImage.mat, arrayImage(badHyp.corner.center), radius, blech, 3 );
				// 	cv::line( arrayImage.mat, arrayImage(badHyp.corner.center),
				// 						arrayImage( badHyp.corner.center + 0.1 * RotPoint(badHyp.basisInCorner()) ),
				// 						blech, 3 );
				// }

			}



			Mat shrunkArr;
			cv::resize( arrayImage.mat, shrunkArr, cv::Size( arrayImage.mat.cols/2, arrayImage.mat.rows/2));
			cv::imshow( "Array", shrunkArr );
			Mat shrunk;
			cv::resize( originalImage, shrunk, cv::Size( originalImage.cols/4, originalImage.rows/4));
			cv::imshow( "Original", shrunk );
			cv::waitKey();
		}

 	}

}


//=== Debugging functions ====

void CornerDetector::drawIntersection( Mat &img, const shared_ptr<Intersection> &intersection, const cv::Scalar &color )
{
	float radius = float(img.rows + img.cols)/2 * 0.01;

	// Draw an ellipse
	for( float theta = 0; theta < (2*M_PI+0.1); theta += 0.1 ) {
		cv::Point2f start( radius * cos( theta ), radius * sin(theta )),
								end( radius * cos( theta + 0.1 ), radius * sin(theta + 0.1));
		cv::line( img,
							intersection->absInvTransform(start ),
							intersection->absInvTransform( end ),
							color, 2 );
	}

	// cv::circle( intersectionsImage, intersection.center,
	// 					radius, color, 2 );

	cv::line( img, intersection->center,
						intersection->center + cv::Point2f( radius * cos( intersection->basis[0] ), radius * sin( intersection->basis[0] ) ),
						cv::Scalar(0,0,255), 2 );
	// cv::line( img, intersection->center,
	// 					intersection->center + cv::Point2f( radius * cos( intersection->basis[1] ), radius * sin( intersection->basis[1] ) ),
	// 					cv::Scalar(0,255,0), 2 );

	// char buf[10];
	// snprintf( buf, 10, "%02x", intersection->corner );
	// cv::putText( img, buf, intersection->center + cv::Point2f(10,-10), cv::FONT_HERSHEY_SIMPLEX, 2, color);

	// double angles[4] = { theta + M_PI, theta - M_PI/2, theta + M_PI/2, theta };
	for( unsigned int j = 0; j < 4; ++j ) {
		if( intersection->corner & (1 << j) ) {
			int fill = (j==0) ? -1 : 2;
			float angle = Corners::angles[j] + intersection->basis[0];
			cv::circle( img, intersection->absInvTransform(radius * cv::Point2f( cos( angle ), sin( angle ) )),
								8, color, fill );
		}
	}
}

void CornerDetector::saveIntersectionImage( const vector< shared_ptr<Intersection> > &intersections )
{
	Mat intersectionsImage;
	// Boost back to color imagery for better annotation
	debugImage( OriginalBGRImage ).copyTo( intersectionsImage );

	CV_Assert( intersectionsImage.type() == CV_32FC3 );

	for( auto const &intersection : intersections  ) {
		long r = random();
		cv::Scalar color( (float)(r & 0xFF)/255, (float)((r&0xFF00)>>8)/255, (float)((r&0xFF0000)>>8)/255 );

		drawIntersection( intersectionsImage, intersection, color );

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
