
#pragma once

#include <string>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

inline void write32FC1( const std::string &filename, const cv::Mat &img )
{
  cv::Mat out;
  img.convertTo( out, CV_8UC1, 255. );
  cv::imwrite( filename, out );
}

inline void write32FC3( const std::string &filename, const cv::Mat &img )
{
  cv::Mat out;
  img.convertTo( out, CV_8UC3, 255. );
  cv::imwrite( filename, out );
}
