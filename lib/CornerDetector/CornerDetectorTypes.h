
#pragma once

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include "Utils/BinaryClassifier.h"
#include "Utils/MathUtil.h"
#include "AprilTags/Corners.h"

namespace AprilTags {

	using cv::Mat;
	using cv::Matx33f;

	struct Intersection {
	public:

		// How to set radius?
		Intersection( const cv::Point2f &c, float basis0, float basis1, const Mat &img )
			: center( c )
		{
			basis[0] = basis0;
			basis[1] = basis1;

			// Ensure ordering
			if( basis[1] < basis[0] ) std::swap( basis[1], basis[0]);
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

		float meanAngle( void ) const
		{
			return atan2( sin(basis[0])+sin(basis[1]), cos(basis[0])+cos(basis[1]));
		}

		float includedAngle( void ) const
		{
			// TODO:  More robust algorithm
			float dt = basis[1] - basis[0];
	 		while( dt > M_PI ) dt = 2*M_PI - dt;
			return dt;
		}

		cv::Point2f transform( const cv::Point2f &in ) const
		{
			cv::Vec3f o( _transform * cv::Vec3f( in.x, in.y, 1 ));
			return cv::Point2f( o[0]/o[2], o[1]/o[2]);
		}

		cv::Point2f absTransform( const cv::Point2f &in ) const
		{
			return center + transform( in - center );
		}

		cv::Point2f invTransform( const cv::Point2f &in ) const
		{
			cv::Vec3f o( _transform.inv() * cv::Vec3f( in.x, in.y, 1 ));
			return cv::Point2f( o[0]/o[2], o[1]/o[2]);
		}

		cv::Point2f absInvTransform( const cv::Point2f &in ) const
		{
			return center + invTransform( in );
		}

		void estimateTransform( void )
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

		void sample( const Mat &img, float radius = 11 )
		{
			int neighborhood( 2*radius + 5 );
			CV_Assert( neighborhood % 2 == 1 );

			float theta = meanAngle();
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

			double angles[4] = { theta + M_PI, theta - M_PI/2, theta + M_PI/2, theta };
			for( unsigned int i = 0; i < 4; ++i ) {
				cv::Point2f pt( cv::Point2f(neighborhood*0.5, neighborhood*0.5) + radius * cv::Point2f( cos( angles[i]), sin(angles[i]) ) );
				Mat ptRoi( warped, cv::Rect( pt.x-1, pt.y-1, 3, 3 ));

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

		Matx33f _transform;

		unsigned char corner;
	};

	// //=========================================================
	//
	// struct Triplet {
	//
	// 	Triplet(  Intersection &a,  Intersection &b,  Intersection &c )
	// 		: _a(a), _b(b), _c(c)
	// 	{
	// 		_offset[0] = _b.basis[0] - _a.basis[0];
	// 		_offset[1] = _c.basis[0] - _a.basis[0];
	//
	// 		// Bound the differences to +-PI
	// 		while( _offset[0] > M_PI ) _offset[0] -= 2*M_PI;
	// 		while( _offset[1] > M_PI ) _offset[1] -= 2*M_PI;
	// 		while( _offset[0] <= -M_PI ) _offset[0] += 2*M_PI;
	// 		while( _offset[1] <= -M_PI ) _offset[1] += 2*M_PI;
	// 	}
	//
	// 	Triplet &operator=( const Triplet &t )
	// 	{ _a = t._a; _b = t._b; _c = t._c;
	// 		_offset[0] = t._offset[0]; _offset[1] = t._offset[1];
	// 		return *this; }
	//
	// 	const Intersection &operator[]( unsigned int i ) const
	// 	{ return (i==0) ? _a : ((i==1) ? _b : _c); }
	//
	// 	float area( void ) const
	// 	{
	// 		float ab = MathUtil::distance2D( _a.center, _b.center ),
	// 					bc = MathUtil::distance2D( _b.center, _c.center ),
	// 					ca = MathUtil::distance2D( _c.center, _a.center );
	//  		float s = 0.5 * (ab + bc + ca );
	//
	// 		return sqrt( s * (s-ab) * (s-bc) * (s-ca) );
	// 	}
	//
	// 	bool hasAngularAgreement( float threshold = 0.2 ) const
	// 	{
	// 		float o[2] = { _offset[0], _offset[1] };
	// 		// What is the total distance spanned by the basis[0] vectors, modulo PI/2
	// 		while( o[0] > M_PI/2 ) o[0] -= M_PI/2;
	// 		while( o[1] > M_PI/2 ) o[1] -= M_PI/2;
	// 		while( o[0] <= -M_PI/2 ) o[0] += M_PI/2;
	// 		while( o[1] <= -M_PI/2 ) o[1] += M_PI/2;
	//
	// 		return ( o[0] < threshold && o[1] < threshold &&
	// 						o[0] > -threshold && o[1] > -threshold );
	// 	}
	//
	// 	Matx33f meanTransform( void )
	// 	{
	// 		// What is a mean affine transform anyway?
	//
	// 		return Matx33f();
	// 	}
	//
	// 	Intersection &_a, &_b, &_c;
	// 	float _offset[2];
	//
	//
	// };


	struct Triangle {

		Triangle( const std::shared_ptr< Intersection > &v0,
							const std::shared_ptr< Intersection > &v1,
							const std::shared_ptr< Intersection > &v2 );

		const std::shared_ptr< Intersection > &vertex( unsigned int i ) const { return _vertices[i]; }
		const std::shared_ptr< Triangle > &neighbor( unsigned int i ) const { return _neighbors[i]; }

		bool hasEdge( const std::shared_ptr< Intersection > &a, const std::shared_ptr< Intersection > &b );
		void addNeighbor( unsigned int i, std::shared_ptr< Triangle > &tri );

		float area( void ) const;
		cv::Point2f center( void ) const;

		unsigned int neighborCount( void ) const
		{
			unsigned int c = 0;
			for( unsigned int i = 0; i < 3 ; ++i ) if( _neighbors[i].get() != NULL ) ++c;
			return c;
		}

		std::shared_ptr< Intersection > _vertices[3];
		std::shared_ptr< Triangle > _neighbors[3];

	};


}
