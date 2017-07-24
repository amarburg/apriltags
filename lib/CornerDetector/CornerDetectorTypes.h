
#pragma once

#include <memory>

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include "Utils/BinaryClassifier.h"
#include "Utils/MathUtil.h"
#include "AprilTags/Corners.h"

namespace AprilTags {

	using cv::Mat;
	using cv::Matx33f;

	static inline float ZeroTwoPi( float v )
	{
		while( v >= 2*M_PI ) v -= 2*M_PI;
		while( v < 0 ) v += 2*M_PI;
		return v;
	}

	struct Intersection {
	public:

		// How to set radius?
		Intersection( const cv::Point2f &c, float basis0, float basis1, const Mat &img );

		// TODO:  In the long run, I don't think we need two bases for anything
		// other than affine estimation.   More robust to just use a
		// coordinate system with basis0 as +X
		float meanAngle( void ) const;

		float includedAngle( void ) const;

		cv::Point2f transform( const cv::Point2f &in ) const;

		cv::Point2f absTransform( const cv::Point2f &in ) const;

		cv::Point2f invTransform( const cv::Point2f &in ) const;

		cv::Point2f absInvTransform( const cv::Point2f &in ) const;

		void estimateTransform( void );

		void sample( const Mat &img, float radius = 11 );

		void identify( const BinaryClassifier &classifier );

		cv::Point2f center;
		float basis[2];
		float samples[4];

		Matx33f _transform;

		unsigned char corner;

		// Thought about maintaining neighbors.  Hard to construct
 		// (dealing with repetition).   Simpler to just repeatedly
		// search the triangle list?
		//std::vector< shared_ptr< Intersection > > neighbors;
	};



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
