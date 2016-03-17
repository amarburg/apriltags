#include <algorithm>
#include <cmath>
#include <climits>
#include <map>
#include <vector>
#include <iostream>

#include <Eigen/Dense>

#include <opencv2/imgproc/imgproc.hpp>

#include "AprilTags/Edge.h"
#include "AprilTags/FloatImage.h"
#include "AprilTags/Gaussian.h"
#include "AprilTags/GrayModel.h"
#include "AprilTags/GLine2D.h"
#include "AprilTags/Gridder.h"
#include "AprilTags/Homography33.h"
#include "AprilTags/MathUtil.h"
#include "AprilTags/Quad.h"
#include "AprilTags/TagFamily.h"
#include "AprilTags/UnionFindSimple.h"

#include "AprilTags/TagDetector.h"


using namespace std;
using namespace cv;

namespace AprilTags {

  //-----------------------------------------------------------------------------
  void TagDetector::SetUseHybridMethod( bool useHybrid)
  {
    m_UseHybrid = useHybrid;
  }


  //-----------------------------------------------------------------------------
  void TagDetector::SetMinSize( float minSize)
  {
    m_MinSize = minSize;
  }


  //-----------------------------------------------------------------------------
  void TagDetector::SetMaxSize( float maxSize)
  {
    m_MaxSize = maxSize;
  }


  //-----------------------------------------------------------------------------
  void TagDetector::SetBlockSize( int blockSize)
  {
    m_BlockSize = blockSize;
  }


  //-----------------------------------------------------------------------------
  void TagDetector::SetOffset( int offset)
  {
    m_Offset = offset;
  }


  //-----------------------------------------------------------------------------
  void TagDetector::SetSigma( float sigma)
  {
    m_Sigma = sigma;
  }


  //-----------------------------------------------------------------------------
  void TagDetector::SetSegmentationSigma( float segmentationSigma)
  {
    m_SegmentationSigma = segmentationSigma;
  }



  //-----------------------------------------------------------------------------
  std::vector<TagDetection> TagDetector::extractTags(const cv::Mat& inImage) {

    // Follow the OpenCV design pattern of issuing a fatal error when
    // Mats are not of the appropriate type.
    CV_Assert( !inImage.empty() );
    CV_Assert(  inImage.channels() == 1 );
    CV_Assert( (inImage.type() == CV_32FC1) || (inImage.type() == CV_8UC1) );

    int width = inImage.size().width;
    int height = inImage.size().height;
    Point2f opticalCenter( width/2, height/2 );

    // AprilTags::FloatImage fimOrig(width, height);
    // int i = 0;
    // for (int y=0; y<height; y++) {
    //   for (int x=0; x<width; x++) {
    //     fimOrig.set(x, y, image.data[i]/255.);
    //     i++;
    //   }
    // }
    // std::pair<int,int> opticalCenter(width/2, height/2);




#ifdef DEBUG_APRIL
#if 0
  { // debug - write
    int height_ = fimOrig.getHeight();
    int width_  = fimOrig.getWidth();
    cv::Mat image(height_, width_, CV_8UC3);
    {
      for (int y=0; y<height_; y++) {
        for (int x=0; x<width_; x++) {
          cv::Vec3b v;
          //        float vf = fimMag.get(x,y);
          float vf = fimOrig.get(x,y);
          int val = (int)(vf * 255.);
          if ((val & 0xffff00) != 0) {printf("problem... %i\n", val);}
          for (int k=0; k<3; k++) {
            v(k) = val;
          }
          image.at<cv::Vec3b>(y, x) = v;
        }
      }
    }
    imwrite("out.bmp", image);
  }
#endif
#if 0
  FloatImage fimOrig = fimOrig_;
  { // debug - read

    cv::Mat image = cv::imread("test.bmp");
    int height_ = fimOrig.getHeight();
    int width_  = fimOrig.getWidth();
    {
      for (int y=0; y<height_; y++) {
        for (int x=0; x<width_; x++) {
          cv::Vec3b v = image.at<cv::Vec3b>(y,x);
          float val = (float)v(0)/255.;
          fimOrig.set(x,y,val);
        }
      }
    }
  }
#endif
#endif

  //================================================================
  // Step one: preprocess image (convert to Float) and low pass if necessary

  // Initial Gaussian smoothing and convert to CV_32F if necessary
  Mat blurredImage;

  if( inImage.type() == CV_8UC1 ) {
    inImage.convertTo( blurredImage, CV_32FC1, 1.0/255.0 );
  }

  if( m_saveDebugImages ) saveOriginalImage( blurredImage.empty() ? inImage : blurredImage );

  if (m_Sigma > 0) {
    int filtsz = ((int) max(3.0f, 3 * m_Sigma)) | 1;

    // Is blurredImage has been converted, I can do the filtering in-place
    // Otherwise the GaussianBlur performs the copy for me
    GaussianBlur( blurredImage.empty() ? inImage : blurredImage,
                  blurredImage, Size(filtsz, filtsz), m_Sigma, m_Sigma );

  } else if( blurredImage.empty() ){
    // Only hit this case if not blurring and inImage is already a 32FC1
    blurredImage = inImage;
  }

  //   std::vector<float> filt = Gaussian::makeGaussianFilter(sigma, filtsz);
  //   fim.filterFactoredCentered(filt, filt);
  //   GaussianBlur( originalImage, originalImage )
  // }

    if( m_saveDebugImages ) saveBlurredImage( blurredImage );

  //! Gaussian smoothing kernel applied to image (0 == no filter).
  /*! Used when sampling bits. Filtering is a good idea in cases
   * where A) a cheap camera is introducing artifical sharpening, B)
   * the bayer pattern is creating artifcats, C) the sensor is very
   * noisy and/or has hot/cold pixels. However, filtering makes it
   * harder to decode very small tags. Reasonable values are 0, or
   * [0.8, 1.5].
   */
//  float sigma = m_Sigma;

  //! Gaussian smoothing kernel applied to image (0 == no filter).
  /*! Used when detecting the outline of the box. It is almost always
   * useful to have some filtering, since the loss of small details
   * won't hurt. Recommended value = 0.8. The case where sigma ==
   * segsigma has been optimized to avoid a redundant filter
   * operation.
   */
//  float segSigma = m_SegmentationSigma;



  //================================================================
  // Step two: Compute the local gradient. We store the direction and magnitude.
  // This step is quite sensitve to noise, since a few bad theta estimates will
  // break up segments, causing us to miss Quads. It is useful to do a Gaussian
  // low pass on this step even if we don't want it for encoding.

  Mat segmentationImage;

  if ( (m_SegmentationSigma > 0)  && (m_SegmentationSigma != m_Sigma)) {

    int filtsz = ((int) max(3.0f, 3 * m_SegmentationSigma)) | 1;
    GaussianBlur( blurredImage, segmentationImage, Size(filtsz, filtsz), m_SegmentationSigma, m_SegmentationSigma );

      // blur anew

      // std::vector<float> filt = Gaussian::makeGaussianFilter(segSigma, filtsz);
      // fimSeg = fimOrig;
      // fimSeg.filterFactoredCentered(filt, filt);

  } else {
    segmentationImage = blurredImage;
  }

  Mat dx, dy;

  // For now, use the 1x3 / 3x1 Sobel operator s.t.
  // the blur (above) is independent from the derivative calculation.
  // Probably more efficient to combine the two.

  // And, more importantly, is this OpenCV implementation faster than the
  // hand-coded OpemMP version which was here originally?

  Sobel( segmentationImage, dx, CV_32F, 1, 0, 1 );
  Sobel( segmentationImage, dy, CV_32F, 0, 1, 1 );

  Mat magnitude( dx.size(), CV_32F ),
      angle( dx.size(), CV_32F );
  cartToPolar( dx, dy, magnitude, angle );

  // FloatImage fimTheta(fimSeg.getWidth(), fimSeg.getHeight());
  // FloatImage fimMag(fimSeg.getWidth(), fimSeg.getHeight());

  if( m_saveDebugImages ) saveMagnitudeImage( magnitude );

//   #pragma omp parallel for
//   for (int y = 1; y < fimSeg.getHeight()-1; y++) {
//     for (int x = 1; x < fimSeg.getWidth()-1; x++) {
//       float Ix = fimSeg.get(x+1, y) - fimSeg.get(x-1, y);
//       float Iy = fimSeg.get(x, y+1) - fimSeg.get(x, y-1);
//
//       float mag = Ix*Ix + Iy*Iy;
// #if 0 // kaess: fast version, but maybe less accurate?
//       float theta = MathUtil::fast_atan2(Iy, Ix);
// #else
//       float theta = atan2(Iy, Ix);
// #endif
//
//       fimTheta.set(x, y, theta);
//       fimMag.set(x, y, mag);
//     }
//   }



  //================================================================
  // Step three. Extract edges by grouping pixels with similar
  // thetas together. This is a greedy algorithm: we start with
  // the most similar pixels.  We use 4-connectivity.
  //
  // TODO:  At this point the edge collection code is requiring approx 50%
  // of the total computational cost

  UnionFindSimple uf( width * height );

  vector<Edge> edges( width*height*4 );
  size_t nEdges = 0;

  // Bounds on the thetas assigned to this group. Note that because
  // theta is periodic, these are defined such that the average
  // value is contained *within* the interval.
  { // limit scope of storage
    /* Previously all this was on the stack, but this is 1.2MB for 320x240 images
     * That's already a problem for OS X (default 512KB thread stack size),
     * could be a problem elsewhere for bigger images... so store on heap */
    vector<float> storage(width*height*4);  // do all the memory in one big block, exception safe
    float * tmin = &storage[width*height*0];
    float * tmax = &storage[width*height*1];
    float * mmin = &storage[width*height*2];
    float * mmax = &storage[width*height*3];

    Point p(0,0);
    for (; p.y < (height-1); ++p.y) {
      for ( p.x = 0; p.x < (width-1); ++p.x ) {
        int offset = p.y*width + p.x;

        float mag0 = magnitude.at<float>(p);
        if (mag0 < Edge::minMag) continue;

        mmax[offset] = mag0;
        mmin[offset] = mag0;

        float theta0 = angle.at<float>(p);
        tmin[offset] = theta0;
        tmax[offset] = theta0;

        // Calculates then adds edges to 'vector<Edge> edges'
        Edge::calcEdges(theta0, p.x, p.y, angle, magnitude, edges, nEdges);

        // XXX Would 8 connectivity help for rotated tags?
        // Probably not much, so long as input filtering hasn't been disabled.
      }
    }

    edges.resize(nEdges);
    std::stable_sort(edges.begin(), edges.end());
    Edge::mergeEdges(edges,uf,tmin,tmax,mmin,mmax, width*height );
  }

  //================================================================
  // Step four: Loop over the pixels again, collecting statistics for each cluster.
  // We will soon fit lines (segments) to these points.

  map<int, vector<XYWeight> > clusters;
  Point p( 0, 0);
  for ( ; p.y < (height-1);  ++p.y ) {
    for ( p.x = 0; p.x < (width-1); ++p.x ) {
      int offset = p.y*width + p.x;

      if (uf.getSetSize(offset) < Segment::minimumSegmentSize)	continue;

      int rep = (int) uf.getRepresentative(offset);

      map<int, vector<XYWeight> >::iterator it = clusters.find(rep);
      if ( it == clusters.end() ) {
      	clusters[rep] = vector<XYWeight>();
      	it = clusters.find(rep);
      }
      vector<XYWeight> &points = it->second;

      points.push_back(XYWeight(p.x,p.y,magnitude.at<float>(p)));
    }
  }

  //================================================================
  // Step five: Loop over the clusters, fitting lines (which we call Segments).
  std::vector<Segment> segments; //used in Step six
  std::map<int, std::vector<XYWeight> >::const_iterator clustersItr;
  for (clustersItr = clusters.begin(); clustersItr != clusters.end(); clustersItr++) {
    std::vector<XYWeight> points = clustersItr->second;
    GLineSegment2D gseg = GLineSegment2D::lsqFitXYW(points);

    // filter short lines
    float length = MathUtil::distance2D(gseg.getP0(), gseg.getP1());
    if (length < Segment::minimumLineLength)
      continue;

    Segment seg;
    Segment::ExtractSegment(gseg, length, points, angle, magnitude, seg);
    segments.push_back(seg);
  }

  if( m_saveDebugImages ) saveLineSegments( segments );

  // Step six: For each segment, find segments that begin where this segment ends.
  // (We will chain segments together next...) The gridder accelerates the search by
  // building (essentially) a 2D hash table.
  Gridder<Segment> gridder(0,0,width,height,10);

  // add every segment to the hash table according to the position of the segment's
  // first point. Remember that the first point has a specific meaning due to our
  // left-hand rule above.
  for (unsigned int i = 0; i < segments.size(); i++) {
    gridder.add(segments[i].getX0(), segments[i].getY0(), &segments[i]);
  }

  // Now, find child segments that begin where each parent segment ends.
  for (unsigned i = 0; i < segments.size(); i++) {
    Segment &parentseg = segments[i];

    //compute length of the line segment
    GLine2D parentLine(std::pair<float,float>(parentseg.getX0(), parentseg.getY0()),
		       std::pair<float,float>(parentseg.getX1(), parentseg.getY1()));

    Gridder<Segment>::iterator iter = gridder.find(parentseg.getX1(), parentseg.getY1(), 0.5f*parentseg.getLength());
    while(iter.hasNext()) {
      Segment &child = iter.next();
      if (MathUtil::mod2pi(child.getTheta() - parentseg.getTheta()) > 0) continue;

      // compute intersection of points
      GLine2D childLine(std::pair<float,float>(child.getX0(), child.getY0()),
			std::pair<float,float>(child.getX1(), child.getY1()));

      std::pair<float,float> p = parentLine.intersectionWith(childLine);
      if (p.first == -1) continue;


      float parentDist = MathUtil::distance2D(p, std::pair<float,float>(parentseg.getX1(),parentseg.getY1()));
      float childDist = MathUtil::distance2D(p, std::pair<float,float>(child.getX0(),child.getY0()));

      if (max(parentDist,childDist) > parentseg.getLength()) continue;

      // everything's OK, this child is a reasonable successor.
      parentseg.children.push_back(&child);
    }
  }

  //================================================================
  // Step seven: Search all connected segments to see if any form a loop of length 4.
  // Add those to the quads list.
  vector<Quad> quads;

  vector<Segment*> tmp(5);
  for (unsigned int i = 0; i < segments.size(); i++) {
    tmp[0] = &segments[i];
    Quad::search(tmp, segments[i], 0, quads, opticalCenter); //fimOrig
  }

  //================================================================
  // Step 7b. Inspired by ARUCO, OpenCV.
  // We perform thresholding followed by OpenCV routines to extract
  // tags, and add to the list of Quads.
  if (m_UseHybrid)
  {
    // adaptiveThreshold requires an 8UC1 image... after all that trouble to
    // produce a 32FC1 image
    Mat image8Bit = inImage;
    if( inImage.type() != CV_8UC1 ) inImage.convertTo( image8Bit, CV_8UC1, 255. );

    // int height_ = fimOrig.getHeight();
    // int width_  = fimOrig.getWidth();
    cv::Mat thresholded( inImage.size(), CV_8UC1);
    cv::adaptiveThreshold ( image8Bit, thresholded, 255, cv::ADAPTIVE_THRESH_MEAN_C, cv::THRESH_BINARY_INV, m_BlockSize, m_Offset);

    unsigned int minSize = m_MinSize*std::max(thresholded.cols,thresholded.rows)*4;
    unsigned int maxSize = m_MaxSize*std::max(thresholded.cols,thresholded.rows)*4;
    std::vector<std::vector<cv::Point> > contours;
    std::vector<cv::Vec4i> hierarchy;

    cv::findContours ( thresholded , contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE );

    std::vector<cv::Point>  approxCurve;
    for ( unsigned int i = 0; i < contours.size(); i++ )
    {
      if ( minSize < contours[i].size() && contours[i].size() < maxSize )
      {
        cv::approxPolyDP(contours[i], approxCurve , double (contours[i].size()) * 0.05, true);
        if ( approxCurve.size() == 4 )
        {
          if ( cv::isContourConvex (cv::Mat ( approxCurve ) ) )
          {
            float minDist = 1e10;
            for ( unsigned int j = 0; j < 4; j++ )
            {
              float d = std::sqrt ( ( float ) ( approxCurve[j].x-approxCurve[ ( j+1 ) %4].x ) * ( approxCurve[j].x-approxCurve[ ( j+1 ) %4].x ) +
                                              ( approxCurve[j].y-approxCurve[ ( j+1 ) %4].y ) * ( approxCurve[j].y-approxCurve[ ( j+1 ) %4].y ) );
              if ( d < minDist )
              {
                minDist=d;
              }
            }
            if ( minDist>10 )
            {
              std::vector<std::pair<float,float> > corners;
              for (unsigned int j = 0; j < 4; j++)
              {
                corners.push_back(std::pair<float, float>(approxCurve[j].x, approxCurve[j].y));
              }
              Quad newQuad(corners, opticalCenter);
              quads.push_back(newQuad);
            } // if larger than 10 pixels
          } // if curve is convex
        } // if curve has 4 sides
      } // if outline within minSize, maxSize
    } // for each contour
  } // if grey, 1 channel image

  if( m_saveDebugImages ) saveQuadImage( quads );

  //================================================================
  // Step eight. Decode the quads. For each quad, we first estimate a
  // threshold color to decide between 0 and 1. Then, we read off the
  // bits and see if they make sense.

  std::vector<TagDetection> detections;

  for (unsigned int qi = 0; qi < quads.size(); qi++ ) {
    Quad &quad = quads[qi];

    // Find a threshold
    GrayModel blackModel, whiteModel;
    const int dd = 2 * thisTagFamily.blackBorder + thisTagFamily.dimension();

    for (int iy = -1; iy <= dd; iy++) {
      float y = (iy + 0.5f) / dd;
      for (int ix = -1; ix <= dd; ix++) {
        float x = (ix + 0.5f) / dd;
        std::pair<float,float> pxy = quad.interpolate01(x, y);

        int irx = (int) (pxy.first + 0.5);
        int iry = (int) (pxy.second + 0.5);
        if (irx < 0 || irx >= width || iry < 0 || iry >= height) continue;

        float v = blurredImage.at<float>(iry,irx);

        if (iy == -1 || iy == dd || ix == -1 || ix == dd) {
          whiteModel.addObservation(x, y, v);
        } else if (iy == 0 || iy == (dd-1) || ix == 0 || ix == (dd-1)){
          blackModel.addObservation(x, y, v);
        }
      }
    }

    bool bad = false;
    uint64_t tagCode = 0;
    for ( int iy = thisTagFamily.dimension()-1; iy >= 0; iy-- ) {
      float y = (thisTagFamily.blackBorder + iy + 0.5f) / dd;
      for (int ix = 0; ix < thisTagFamily.dimension(); ix++ ) {
        float x = (thisTagFamily.blackBorder + ix + 0.5f) / dd;
        std::pair<float,float> pxy = quad.interpolate01(x, y);
        int irx = (int) (pxy.first + 0.5);
        int iry = (int) (pxy.second + 0.5);
        if (irx < 0 || irx >= width || iry < 0 || iry >= height) {
          // cout << "*** bad:  irx=" << irx << "  iry=" << iry << endl;
          bad = true;
          continue;
        }
        float threshold = (blackModel.interpolate(x,y) + whiteModel.interpolate(x,y)) * 0.5f;
        float v = blurredImage.at<float>(iry, irx );
        tagCode = tagCode << 1;
        if ( v > threshold) tagCode |= 1;

        if( m_saveDebugImages ) {
          if (v>threshold){
            drawQuadBit( cv::Point2f(irx, iry), cv::Scalar(0,0,255,0) );
          } else {
            drawQuadBit( cv::Point2f(irx, iry), cv::Scalar(0,255,0,0) );
          }
        }
      }
    }

    if ( !bad ) {
      TagDetection thisTagDetection;
      thisTagFamily.decode(thisTagDetection, tagCode);

      // compute the homography (and rotate it appropriately)
      thisTagDetection.homography = quad.homography.getH();

      // TODO.  Ugh.  Impedence mismatch
      Eigen::Vector2f oc = quad.homography.getCXY();
      thisTagDetection.hxy = std::pair<float,float>( oc.x(), oc.y() );

      float c = std::cos(thisTagDetection.rotation*(float)M_PI/2);
      float s = std::sin(thisTagDetection.rotation*(float)M_PI/2);
      Eigen::Matrix3d R;
      R.setZero();
      R(0,0) = R(1,1) = c;
      R(0,1) = -s;
      R(1,0) = s;
      R(2,2) = 1;
      Eigen::Matrix3d tmp;
      tmp = thisTagDetection.homography * R;
      thisTagDetection.homography = tmp;

      // Rotate points in detection according to decoded
      // orientation.  Thus the order of the points in the
      // detection object can be used to determine the
      // orientation of the target.
      std::pair<float,float> bottomLeft = thisTagDetection.interpolate(-1,-1);
      int bestRot = -1;
      float bestDist = FLT_MAX;
      for ( int i=0; i<4; i++ ) {
      	float const dist = AprilTags::MathUtil::distance2D(bottomLeft, quad.quadPoints[i]);
      	if ( dist < bestDist ) {
      	  bestDist = dist;
      	  bestRot = i;
      	}
      }

      for (int i=0; i< 4; i++)
	      thisTagDetection.p[i] = quad.quadPoints[(i+bestRot) % 4];

      if (thisTagDetection.good) {
      	thisTagDetection.cxy = quad.interpolate01(0.5f, 0.5f);
      	thisTagDetection.observedPerimeter = quad.observedPerimeter;
      	detections.push_back(thisTagDetection);
      }
    }
  }


  //================================================================
  //Step nine: Some quads may be detected more than once, due to
  //partial occlusion and our aggressive attempts to recover from
  //broken lines. When two quads (with the same id) overlap, we will
  //keep the one with the lowest error, and if the error is the same,
  //the one with the greatest observed perimeter.

  std::vector<TagDetection> goodDetections;

  // NOTE: allow multiple non-overlapping detections of the same target.

  for ( vector<TagDetection>::const_iterator it = detections.begin();
	it != detections.end(); it++ ) {
    const TagDetection &thisTagDetection = *it;

    bool newFeature = true;

    for ( unsigned int odidx = 0; odidx < goodDetections.size(); odidx++) {
      TagDetection &otherTagDetection = goodDetections[odidx];

      if ( thisTagDetection.id != otherTagDetection.id ||
	         ! thisTagDetection.overlapsTooMuch(otherTagDetection) ) continue;

      // There's a conflict.  We must pick one to keep.
      newFeature = false;

      // This detection is worse than the previous one... just don't use it.
      if ( thisTagDetection.hammingDistance > otherTagDetection.hammingDistance )	continue;

      // Otherwise, keep the new one if it either has strictly *lower* error, or greater perimeter.
      if ( thisTagDetection.hammingDistance < otherTagDetection.hammingDistance ||
	         thisTagDetection.observedPerimeter > otherTagDetection.observedPerimeter )
	       goodDetections[odidx] = thisTagDetection;
    }

     if ( newFeature ) goodDetections.push_back(thisTagDetection);

  }

  //cout << "AprilTags: edges=" << nEdges << " clusters=" << clusters.size() << " segments=" << segments.size()
  //     << " quads=" << quads.size() << " detections=" << detections.size() << " unique tags=" << goodDetections.size() << endl;

  return goodDetections;
}

/*
 *  Functions related to making debug images
 *
 */

  bool TagDetector::validDebugImage( DebugImages_t which )
  {
    return (which >= 0) && (which < TagDetector::NUM_DEBUG_IMAGES);
  }

  bool TagDetector::SaveDebugImages( bool val )
  {
    return m_saveDebugImages = val;
  }

  Mat TagDetector::debugImage( DebugImages_t which )
  {
    if( validDebugImage( which ) )
      return _debugImages[which];

    return Mat();
  }

  void TagDetector::saveDebugImage( const Mat &img, DebugImages_t which, bool clone )
  {
    if( validDebugImage( which ) ) {
      if( clone )
        img.copyTo(_debugImages[which]);
      else
        _debugImages[which] = img;
    }
  }

  void TagDetector::saveOriginalImage( const Mat &img )
  {
    saveDebugImage( img, OriginalImage );

    // Convenience temporary
    Mat originalBgr;
    cvtColor( img, originalBgr, COLOR_GRAY2BGR );
    saveDebugImage( originalBgr, OriginalBGRImage, false );
  }

  void TagDetector::saveBlurredImage( const Mat &img )
  {
    saveDebugImage( img, BlurredImage );

  }

  void TagDetector::saveMagnitudeImage( const Mat &img )
  {
    double mn, mx;
    minMaxLoc( img, &mn, &mx );

    saveDebugImage( (img * 1.0/mx), MagnitudeImage );
  }

  void TagDetector::saveLineSegments( const vector<Segment> &segments )
  {
    Mat savedLineSegmentsImage;
    // Boost back to color imagery for better annotation
    debugImage( OriginalBGRImage ).copyTo( savedLineSegmentsImage );

    for( vector<Segment>::const_iterator it = segments.begin(); it!=segments.end(); it++ ) {
      long int r = random();
      cv::line(savedLineSegmentsImage,
                cv::Point2f(it->getX0(), it->getY0()),
                cv::Point2f(it->getX1(), it->getY1()),
                cv::Scalar(r%0xff,(r%0xff00)>>8,(r%0xff0000)>>16,0) );
    }

    saveDebugImage( savedLineSegmentsImage, LineSegmentsImage, false );
  }

  void TagDetector::saveQuadImage( const vector<Quad> &quads )
  {
    Mat savedQuadImage;
    debugImage( OriginalBGRImage ).copyTo( savedQuadImage );

      // int height_ = fimOrig.getHeight();
      // int width_  = fimOrig.getWidth();
      // cv::Mat debugImage2(height_, width_, CV_8UC3);
      // for (int y=0; y<height_; y++) {
      //   for (int x=0; x<width_; x++) {
      //     cv::Vec3b v;
      //     for (int k=0; k<3; k++) {
      //       v(k) = 0;
      //     }
      //     debugImage2.at<cv::Vec3b>(y, x) = v;
      //   }
      // }

  const cv::Scalar Red( 0, 0, 255, 0 ),
                   Green( 0, 255, 0, 0 );

    for (unsigned int qi = 0; qi < quads.size(); qi++ ) {
      const Quad &quad = quads[qi];
      std::pair<float, float> p1 = quad.quadPoints[0];
      std::pair<float, float> p2 = quad.quadPoints[1];
      std::pair<float, float> p3 = quad.quadPoints[2];
      std::pair<float, float> p4 = quad.quadPoints[3];
      cv::line(savedQuadImage, cv::Point2f(p1.first, p1.second), cv::Point2f(p2.first, p2.second), Red );
      cv::line(savedQuadImage, cv::Point2f(p2.first, p2.second), cv::Point2f(p3.first, p3.second), Red );
      cv::line(savedQuadImage, cv::Point2f(p3.first, p3.second), cv::Point2f(p4.first, p4.second), Red );
      cv::line(savedQuadImage, cv::Point2f(p4.first, p4.second), cv::Point2f(p1.first, p1.second), Red );

      p1 = quad.interpolate(-1,-1);
      p2 = quad.interpolate(-1,1);
      p3 = quad.interpolate(1,1);
      p4 = quad.interpolate(1,-1);
      cv::circle(savedQuadImage, cv::Point2f(p1.first, p1.second), 3, Green, 1);
      cv::circle(savedQuadImage, cv::Point2f(p2.first, p2.second), 3, Green, 1);
      cv::circle(savedQuadImage, cv::Point2f(p3.first, p3.second), 3, Green, 1);
      cv::circle(savedQuadImage, cv::Point2f(p4.first, p4.second), 3, Green, 1);
    }

    saveDebugImage( savedQuadImage, QuadImage, false );
  }

  // Unlike the other functions, this does not create a new image
  // But simply overlays a point on the quadImage
  void TagDetector::drawQuadBit( const cv::Point2f &pt, const cv::Scalar &color )
  {
    cv::circle( _debugImages[QuadImage], pt, 1, color, 2);
  }

} // namespace
