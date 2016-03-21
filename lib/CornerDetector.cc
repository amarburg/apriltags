
#include <iostream>

#include "AprilTags/CornerDetector.h"
#include "AprilTags/BinaryClassifier.h"
#include "AprilTags/ImageMasks.h"

#include <opencv2/highgui/highgui.hpp>


namespace AprilTags {

using cv::Mat;

//=========================================================

// TODO.  Merge with GLineSegment2D
struct LineSegment {

	LineSegment( const Segment &seg )
		: _a( seg.getX0(), seg.getY0() ),
 			_b( seg.getX1(), seg.getY1() )
	{;}

	LineSegment( const cv::Point2f &a, const cv::Point2f &b )
		: _a(a), _b(b)
	{;}

	const cv::Point2f &a(void) const { return _a; }
	const cv::Point2f &b(void) const { return _b; }

	const float theta( void ) const { return atan2( _b.y-_a.y, _b.x - _a.x ); }

	LineSegment flip( void ) const { return LineSegment( _b, _a); }

protected:
	cv::Point2f _a, _b;
};

struct Intersection {
public:

	// How to set radius?
	Intersection( const cv::Point2f &c, float basis0, float basis1, const Mat &img, int neighborhood = 13 )
		: center( c )
	{
		basis[0] = basis0;
		basis[1] = basis1;

		sample( img, neighborhood );
	}

	float meanAngle( void ) const
	{ return atan2( sin(basis[0])+sin(basis[1]), cos(basis[0])+cos(basis[1]));}

	void sample( const Mat &img, int neighborhood )
	{
		const float radius = 5;

		float theta = meanAngle();
		// theta defines the mean angle between the x and y axes

		double angles[4] = { theta + M_PI, theta - M_PI/2, theta + M_PI/2, theta };
		for( unsigned int i = 0; i < 4; ++i ) {
			cv::Point2f pt( center + radius * cv::Point2f( cos( angles[i]), sin(angles[i]) ) );
			Mat ptRoi( img, cv::Rect( pt.x-1, pt.y-1, 3, 3 ));

			samples[i] = cv::mean( ptRoi )[0];
		}
	}

	void identify( const BinaryClassifier &classifier )
	{
		unsigned char bits = 0;
		for( unsigned int i = 0; i < 4; ++i ) {
			if( classifier.classify( samples[i] ) == 1 )
				bits |= (1 << i);
		}

		corner = Corners::cornerLUT( bits );
	}


	cv::Point2f center;
	float basis[2];
	float samples[4];

	unsigned char corner;
};

//=========================================================

struct Triplet {

	Triplet(  Intersection &a,  Intersection &b,  Intersection &c )
		: _a(a), _b(b), _c(c)
	{
		_offset[0] = _b.basis[0] - _a.basis[0];
		_offset[1] = _c.basis[0] - _a.basis[0];

		// Bound the differences to +-PI
		while( _offset[0] > M_PI ) _offset[0] -= 2*M_PI;
		while( _offset[1] > M_PI ) _offset[1] -= 2*M_PI;
		while( _offset[0] <= -M_PI ) _offset[0] += 2*M_PI;
		while( _offset[1] <= -M_PI ) _offset[1] += 2*M_PI;
	}

	Triplet &operator=( const Triplet &t )
	{ _a = t._a; _b = t._b; _c = t._c;
		_offset[0] = t._offset[0]; _offset[1] = t._offset[1];
		return *this; }

	const Intersection &operator[]( unsigned int i ) const
	{ return (i==0) ? _a : ((i==1) ? _b : _c); }

	float area( void ) const
	{
		float ab = MathUtil::distance2D( _a.center, _b.center ),
					bc = MathUtil::distance2D( _b.center, _c.center ),
					ca = MathUtil::distance2D( _c.center, _a.center );
 		float s = 0.5 * (ab + bc + ca );

		return sqrt( s * (s-ab) * (s-bc) * (s-ca) );
	}

	bool hasAngularAgreement( float threshold = 0.2 ) const
	{
		float o[2] = { _offset[0], _offset[1] };
		// What is the total distance spanned by the basis[0] vectors, modulo PI/2
		while( o[0] > M_PI/2 ) o[0] -= M_PI/2;
		while( o[1] > M_PI/2 ) o[1] -= M_PI/2;
		while( o[0] <= -M_PI/2 ) o[0] += M_PI/2;
		while( o[1] <= -M_PI/2 ) o[1] += M_PI/2;

		return ( o[0] < threshold && o[1] < threshold &&
						o[0] > -threshold && o[1] > -threshold );
	}

	Intersection &_a, &_b, &_c;
	float _offset[2];


};



//=========================================================

// Kind of strange to do this but seems to be reticence to putting state
// in the detector itself.

	static bool angleCheck( float thetaA, float thetaB )
	{
		// Calculate included angle between segments (smallest value 0 < < PI/2)
		float angle = thetaA - thetaB;
		while( angle < 0 ) { angle += M_PI; }
		while( angle >= M_PI) { angle -= M_PI; }

		return ( angle > M_PI/6 && angle < 5*M_PI/6 );
	}

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

	vector< Intersection > intersections;

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
					intersections.push_back( Intersection( mean, aye.theta(), jay.theta(), blurredImage ) );
					break;
				}

				jay = jay.flip();
				if( z == 1 ) aye = aye.flip();

			}

    }
  }

	// Use samples from all intersections to train 2-class GMM
	BinaryClassifier classifier( 0.8 );
	classifier.startTraining();

	// And while we're at it, build the Delaunay to compute local triads
	cv::Subdiv2D delaunay( cv::Rect(0,0, width,height));

	for( auto const &intersection : intersections ) {
		for( int i = 0; i < 4; ++i )
			classifier.trainingPoint( intersection.samples[i] );

		delaunay.insert( intersection.center );
	}

	classifier.endTraining();

	for( auto &intersection : intersections ) {
		intersection.identify( classifier );
	}

	std::cout << intersections.size() << " intersections" << std::endl;
	if( _saveDebugImages ) saveIntersectionImage( intersections );

	vector< cv::Vec6f > delTriangles;
	delaunay.getTriangleList( delTriangles );
	std::cout << delTriangles.size() << " Delaunay triangles" << std::endl;

	vector< Triplet > triplets;

	// Need to map from Delaunay triangles back to triplets of intersections
	// Brute force it for now
	for( auto const &tri : delTriangles ) {

		// Check for any outside vertices outside the image (those on the edges)
		if( tri[0] < 0 || tri[2] < 0 || tri[4] < 0 ||
				tri[0] >= width || tri[2] >= width || tri[4] >= width ||
 				tri[1] < 0 || tri[3] < 0 || tri[5] < 0 ||
				tri[1] >= height || tri[3] >= height || tri[5] >= height ) continue;

		unsigned int idx[3] = {0,0,0};
		bool set[3] = {false, false, false};
		for( int i = 0; i < 3; ++i  ) {
			for( unsigned int j = 0; j < intersections.size(); ++j ) {
				if( intersections[j].center.x == tri[2*i] && intersections[j].center.y == tri[(2*i)+1] ) {
					idx[i] = j;
					set[i] = true;
					break;
				}
			}
		}
		if( set[0] == false || set[1] == false || set[2] == false ) continue;

		triplets.push_back( Triplet(intersections[idx[0]], intersections[idx[1]], intersections[idx[2]]) );
	}

	std::cout << triplets.size() << " intersection triplets" << std::endl;

	if( _saveDebugImages ) saveTripletImage( triplets );

	// Look at the distribution of areas
	vector< float > areas( triplets.size() );
	std::transform( triplets.begin(), triplets.end(), areas.begin(),
									[]( const Triplet &t ){ return t.area(); });

	// Drop triplets with poor angle agreement
	std::remove_if( triplets.begin(), triplets.end(),
									[]( const Triplet &t ){ return !t.hasAngularAgreement(); } );

		std::cout << triplets.size() << " triplets after filtering" << std::endl;

	return output;
}


//=== Debugging functions ====

void CornerDetector::saveIntersectionImage( const vector< Intersection > &intersections )
{
	Mat intersectionsImage;
	// Boost back to color imagery for better annotation
	debugImage( OriginalBGRImage ).copyTo( intersectionsImage );

	CV_Assert( intersectionsImage.type() == CV_32FC3 );

	for( auto const &intersection : intersections  ) {
		long r = random();
		cv::Scalar color( (float)(r & 0xFF)/255, (float)((r&0xFF00)>>8)/255, (float)((r&0xFF0000)>>8)/255 );
		float radius = float(intersectionsImage.rows + intersectionsImage.cols)/2 * 0.01;

		cv::circle( intersectionsImage, intersection.center,
							radius, color, 2 );

		cv::line( intersectionsImage, intersection.center,
							intersection.center + cv::Point2f( radius * cos( intersection.basis[0] ), radius * sin( intersection.basis[0] ) ),
							color, 2 );
		cv::line( intersectionsImage, intersection.center,
							intersection.center + cv::Point2f( radius * cos( intersection.basis[1] ), radius * sin( intersection.basis[1] ) ),
							color, 2 );

		char buf[10];
		snprintf( buf, 10, "%02x", intersection.corner );
		cv::putText( intersectionsImage, buf, intersection.center + cv::Point2f(10,-10), cv::FONT_HERSHEY_SIMPLEX, 2, color);

		float theta = intersection.meanAngle();
		double angles[4] = { theta + M_PI, theta - M_PI/2, theta + M_PI/2, theta };
		for( unsigned int j = 0; j < 4; ++j ) {
			if( intersection.corner & (1 << j) )
				cv::circle( intersectionsImage, intersection.center + radius * cv::Point2f( cos( angles[j] ), sin( angles[j] ) ),
									8, color, -1 );
		}


	}

	saveDebugImage( intersectionsImage, IntersectionImage, false );
}


void CornerDetector::saveTripletImage( const vector< Triplet > &triplets )
{
	Mat tripletImage;
	// Boost back to color imagery for better annotation
	debugImage( OriginalBGRImage ).copyTo( tripletImage );

	CV_Assert( tripletImage.type() == CV_32FC3 );

	for( auto const &triplet : triplets ) {
		long r = random();
		cv::Scalar color( (float)(r & 0xFF)/255, (float)((r&0xFF00)>>8)/255, (float)((r&0xFF0000)>>8)/255 );

		cv::line( tripletImage, triplet[0].center, triplet[1].center,
							color, 2 );
		cv::line( tripletImage, triplet[1].center, triplet[2].center,
							color, 2 );
		cv::line( tripletImage, triplet[2].center, triplet[0].center,
							color, 2 );


	}

	saveDebugImage( tripletImage, TripletImage, false );
}


}
