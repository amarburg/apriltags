
#pragma once

namespace AprilTags {


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

}
