
#include <iostream>

#include "AprilTags/CornerDetector.h"

#include <opencv2/highgui/highgui.hpp>


namespace AprilTags {

using cv::Mat;

//=========================================================

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

		corner = sample( img, neighborhood );
	}

	float meanAngle( void ) const
{ return atan2( sin(basis[0])+sin(basis[1]), cos(basis[0])+cos(basis[1]));}

	unsigned char sample( const Mat &img, int neighborhood )
	{
		unsigned char type = 0;

		const float radius = 5;
		// First decide on threshold for intersection
		Mat roi( img, cv::Rect( center.x-(neighborhood-1)/2, center.y-(neighborhood-1)/2, neighborhood, neighborhood ) );
		Mat prod;
		cv::GaussianBlur( roi, prod, cv::Size(neighborhood,neighborhood), radius );
		float mean = cv::mean( prod )[0];

		float theta = meanAngle();
		// theta defines the mean angle between the x and y axes

		double angles[4] = { theta + M_PI, theta - M_PI/2, theta + M_PI/2, theta };
		for( unsigned int i = 0; i < 4; ++i ) {
			cv::Point2f pt( center + radius * cv::Point2f( cos( angles[i]), sin(angles[i]) ) );
			Mat ptRoi( img, cv::Rect( pt.x-1, pt.y-1, 3, 3 ));
			if( cv::mean( ptRoi )[0] > mean )
				type |= (1 << i);
		}

		return type;
	}


	cv::Point2f center;
	float basis[2];

	unsigned char corner;
};



CornerDetector::CornerDetector( void )
	: DetectorBase( CORNER_DEBUG_IMAGES )
{;}

//=========================================================

struct Mask {
	Mask( const cv::Size &sz )
		: _mask( sz, CV_8UC1, cv::Scalar(0))
	{;}

	Mask( unsigned int width, unsigned int height )
		: _mask( height, width, CV_8UC1, cv::Scalar(0))
	{;}

	void mark( unsigned int x, unsigned int y )
	{
		mark( cv::Point2i( x,y ) );
	}

	virtual void mark( const cv::Point2i &pt )
	{
		_mask.at<unsigned char>(pt) = 1;
	}

	bool check( unsigned int x, unsigned int y )
	{
		return check( cv::Point2i( x,y ));
	}

	bool check( const cv::Point2i &pt )
	{
		return _mask.at<unsigned char>( pt ) == 0;
	}

	cv::Mat _mask;
};

struct ImageMask : public Mask {
	ImageMask( const cv::Size &sz, float radius )
		: Mask( sz ),
			_radius( radius )
	{;}

	virtual void mark( const cv::Point2i &pt )
	{
		circle( _mask, pt, _radius, cv::Scalar(1), -1 );
	}

	float _radius;
};



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

	std::cout << intersections.size() << " intersections" << std::endl;

	if( _saveDebugImages ) saveIntersectionImage( intersections );

	// Classify each corner by sampling the four quadrants around it

	return output;
}


//=== Debugging functions ====

void CornerDetector::saveIntersectionImage( const vector< Intersection > &intersections )
{
	Mat intersectionsImage;
	// Boost back to color imagery for better annotation
	debugImage( OriginalBGRImage ).copyTo( intersectionsImage );

	CV_Assert( intersectionsImage.type() == CV_32FC3 );

	for( unsigned int i = 0; i < intersections.size(); ++i ) {
		const Intersection &intersection( intersections[i] );
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

}
