#ifndef __UTILS_H__
#define __UTILS_H__

#include <opencv2/core/core.hpp>

#include "AprilTags/TagDetection.h"

namespace AprilTags {

  cv::Rect boundingBox( const TagDetection &detection );

  void expandBoundingBox( cv::Rect &bb, float expansion );
  void clipBoundingBox( cv::Rect &bb, const cv::Mat &image );

}

#endif
