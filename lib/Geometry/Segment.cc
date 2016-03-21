#include "Segment.h"
#include <iostream>

namespace AprilTags {

using cv::Mat;

const float Segment::minimumLineLength = 4;

Segment::Segment()
  : children(), x0(0), y0(0), x1(0), y1(0), theta(0), length(0), segmentId(++idCounter) {}

float Segment::segmentLength() {
  return std::sqrt((x1-x0)*(x1-x0) + (y1-y0)*(y1-y0));
}

void Segment::printSegment() {
  std::cout <<"("<< x0 <<","<< y0 <<"), "<<"("<< x1 <<","<< y1 <<")" << std::endl;
}

int Segment::idCounter = 0;

//-----------------------------------------------------------------------------
void Segment::ExtractSegment(
  const GLineSegment2D& gseg,
  const float& length,
  const std::vector<XYWeight>& points,
  const Mat& fimTheta,
  const Mat& fimMag,
  Segment& seg
)
{
  float dy = gseg.getP1().second - gseg.getP0().second;
  float dx = gseg.getP1().first - gseg.getP0().first;

  float tmpTheta = std::atan2(dy,dx);

  seg.setTheta(tmpTheta);
  seg.setLength(length);

  // We add an extra semantic to segments: the vector
  // p1->p2 will have dark on the left, white on the right.
  // To do this, we'll look at every gradient and each one
  // will vote for which way they think the gradient should
  // go. This is way more retentive than necessary: we
  // could probably sample just one point!

  float flip = 0, noflip = 0;
  for (unsigned int i = 0; i < points.size(); i++)
  {
    XYWeight xyw = points[i];

    float theta = fimTheta.at<float>((int) xyw.y, (int) xyw.x );
    float mag = fimMag.at<float>( (int) xyw.y, (int) xyw.x );

    // err *should* be +M_PI/2 for the correct winding, but if we
    // got the wrong winding, it'll be around -M_PI/2.
    float err = MathUtil::mod2pi(theta - seg.getTheta());

    if (err < 0)
      noflip += mag;
    else
      flip += mag;
  }

  if (flip > noflip) {
    float temp = seg.getTheta() + (float)M_PI;
    seg.setTheta(temp);
  }

  float dot = dx*std::cos(seg.getTheta()) + dy*std::sin(seg.getTheta());
  if (dot > 0) {
    seg.setX0(gseg.getP1().first); seg.setY0(gseg.getP1().second);
    seg.setX1(gseg.getP0().first); seg.setY1(gseg.getP0().second);
  }
  else {
    seg.setX0(gseg.getP0().first); seg.setY0(gseg.getP0().second);
    seg.setX1(gseg.getP1().first); seg.setY1(gseg.getP1().second);
  }
}


} // namespace
