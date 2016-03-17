
#include <iostream>

#include "AprilTags/CornerDetector.h"



namespace AprilTags {

using cv::Mat;

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

	// Find segment pairs with endpoints which are close together and a minimum
	// included angle.

	vector< cv::Point2f > intersections;

	int minDistance = Segment::minimumLineLength * min( 1e-2 * width, 1e-2 * height );
	minDistance *= minDistance;

	cv::Mat mask( segments.size(), segments.size(), CV_8UC1, cv::Scalar(0) );

  // Now, just brute force it
  for (unsigned i = 0; i < segments.size(); i++) {
		for (unsigned j = i; j < segments.size(); j++) {

			// Calculate included angle between segments (smallest value?)
			float angle = segments[i].getTheta() - segments[j].getTheta();
			while( angle < 0 ) { angle += M_PI; }
			while( angle >= M_PI) { angle -= M_PI; }

			if( angle < M_PI/6 || angle > 5*M_PI/6 ) continue;

			if( mask.at<unsigned char>(j,i) != 0 ) continue;

			if( MathUtil::distance2Dsqr( segments[i].one(), segments[j].one() ) < minDistance ) {
				mask.at<unsigned char>(j,i) = 1;
				intersections.push_back( MathUtil::meanPoint( segments[i].one(), segments[j].one() ));
			}
			else if( MathUtil::distance2Dsqr( segments[i].one(), segments[j].two() ) < minDistance ) {
				mask.at<unsigned char>(j,i) = 1;
				intersections.push_back( MathUtil::meanPoint( segments[i].one(), segments[j].two() ));
			}
			else if( MathUtil::distance2Dsqr( segments[i].two(), segments[j].one() ) < minDistance ) {
				mask.at<unsigned char>(j,i) = 1;
				intersections.push_back( MathUtil::meanPoint( segments[i].two(), segments[j].one() ));
			}
			else if( MathUtil::distance2Dsqr( segments[i].two(), segments[j].two() ) < minDistance ) {
				mask.at<unsigned char>(j,i) = 1;
				intersections.push_back( MathUtil::meanPoint( segments[i].two(), segments[j].two() ));
			}

    }
  }

	std::cout << intersections.size() << " intersections" << std::endl;

	if( _saveDebugImages ) saveIntersectionImage( intersections );

	return output;
}


//=== Debugging functions ====

void CornerDetector::saveIntersectionImage( const vector< cv::Point2f > &intersections )
{
	Mat intersectionsImage;
	// Boost back to color imagery for better annotation
	debugImage( OriginalBGRImage ).copyTo( intersectionsImage );

	CV_Assert( intersectionsImage.type() == CV_32FC3 );

	for( vector<cv::Point2f>::const_iterator itr = intersections.begin(); itr != intersections.end(); itr++ ) {
		long r = random();
		cv::circle( intersectionsImage, *itr,
							float(intersectionsImage.rows + intersectionsImage.cols)/2 * 0.01,
							cv::Scalar( (float)(r & 0xFF)/255, (float)((r&0xFF00)>>8)/255, (float)((r&0xFF0000)>>8)/255 ),
							2 );
	}

	saveDebugImage( intersectionsImage, IntersectionImage, false );
}

}
