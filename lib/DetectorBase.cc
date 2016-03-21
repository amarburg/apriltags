
#include "AprilTags/DetectorBase.h"

#include <iostream>
#include <map>

#include "Utils/UnionFindSimple.h"

namespace AprilTags {

using cv::Mat;
using std::vector;
using std::map;

void DetectorBase::doImagePrep( const Mat &inImage, Mat &outImage )
{
	if( inImage.type() == CV_8UC1 ) {
    inImage.convertTo( outImage, CV_32FC1, 1.0/255.0 );
  }

  if( _saveDebugImages ) saveOriginalImage( outImage.empty() ? inImage : outImage );

  if (_blurSigma > 0) {
    int filtsz = ((int) std::max(3.0f, 3 * _blurSigma)) | 1;

    // Is blurredImage has been converted, I can do the filtering in-place
    // Otherwise the GaussianBlur performs the copy for me
    cv::GaussianBlur( outImage.empty() ? inImage : outImage,
                  outImage, cv::Size(filtsz, filtsz), _blurSigma, _blurSigma );

  } else if( outImage.empty() ){
    // Only hit this case if not blurring and inImage is already a 32FC1
    outImage = inImage;
  }

	if( _saveDebugImages ) saveDebugImage( outImage, BlurredImage, true );

}

void DetectorBase::calculateGradient( const Mat &blurredImage, Mat &magnitude, Mat &angle)
{
	Mat segmentationImage;

	  if ( (_segmentationSigma > 0)  && (_segmentationSigma != _blurSigma)) {

	    int filtsz = ((int) max(3.0f, 3 * _segmentationSigma)) | 1;
	    cv::GaussianBlur( blurredImage, segmentationImage, cv::Size(filtsz, filtsz), _segmentationSigma, _segmentationSigma );

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

	  Sobel( segmentationImage, dx, CV_32F, 1, 0, 1 );
	  Sobel( segmentationImage, dy, CV_32F, 0, 1, 1 );

	  magnitude.create( dx.size(), CV_32F ),
	  angle.create( dx.size(), CV_32F );
	  cartToPolar( dx, dy, magnitude, angle );

	  // FloatImage fimTheta(fimSeg.getWidth(), fimSeg.getHeight());
	  // FloatImage fimMag(fimSeg.getWidth(), fimSeg.getHeight());

	if( _saveDebugImages ) {
	  {
	    double mn, mx;
	    minMaxLoc( magnitude, &mn, &mx );

	    saveDebugImage( (magnitude * 1.0/mx), MagnitudeImage );
	  }
	}
}


void DetectorBase::fitSegments( const Mat &magnitude, const Mat &angle, vector<Segment> &segments )
{
	unsigned int width = magnitude.cols, height = magnitude.rows;

  UnionFindSimple uf( width * height );

  //vector<Edge> edges( width*height*4 );
	vector<Edge> edges( width*height*4 );						// This is rather optimistic...
  size_t nEdges = 0;

  // Bounds on the thetas assigned to this group. Note that because
  // theta is periodic, these are defined such that the average
  // value is contained *within* the interval.

  /* Previously all this was on the stack, but this is 1.2MB for 320x240 images
   * That's already a problem for OS X (default 512KB thread stack size),
   * could be a problem elsewhere for bigger images... so store on heap */
  vector<float> storage(width*height*4);  // do all the memory in one big block, exception safe
  float * tmin = &storage[width*height*0];
  float * tmax = &storage[width*height*1];
  float * mmin = &storage[width*height*2];
  float * mmax = &storage[width*height*3];

  for ( cv::Point p(0,0); p.y < (height-1); ++p.y) {
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
  Edge::mergeEdges(edges,uf,tmin,tmax,mmin,mmax, width*height);

	std::cout << "Found " << edges.size() << " edges" << std::endl;

	//================================================================
  // Step four: Loop over the pixels again, collecting statistics for each cluster.
  // We will soon fit lines (segments) to these points.

	map<int, vector<XYWeight> > clusters;

  for ( cv::Point p( 0, 0) ; p.y < (height-1);  ++p.y ) {
    for ( p.x = 0; p.x < (width-1); ++p.x ) {
      int offset = p.y*width + p.x;

      if (uf.getSetSize(offset) < Segment::minimumSegmentSize)	continue;

      int rep = (int) uf.getRepresentative(offset);

      map<int, vector<XYWeight> >::iterator it = clusters.find(rep);
      if ( it == clusters.end() ) {
      	clusters[rep] = vector<XYWeight>();
      	it = clusters.find(rep);
      }
      (it->second).push_back(XYWeight(p.x,p.y,magnitude.at<float>(p)));
    }
  }

	std::cout << "Reduces to " << clusters.size() << " clusters" << std::endl;

	//================================================================
  // Step five: Loop over the clusters, fitting lines (which we call Segments).
  // used in Step six
  std::map<int, std::vector<XYWeight> >::const_iterator clustersItr (clusters.begin() );
  for ( ; clustersItr != clusters.end(); clustersItr++) {
    std::vector<XYWeight> points = clustersItr->second;
    GLineSegment2D gseg = GLineSegment2D::lsqFitXYW(points);

    // filter short lines
    float length = MathUtil::distance2D(gseg.getP0(), gseg.getP1());

		// Min line length is approx 4% of edge length
		int minLineLength = Segment::minimumLineLength * min( 1e-2 * width, 1e-2 * height );
    if (length < minLineLength)
      continue;

    Segment seg;
    Segment::ExtractSegment(gseg, length, points, angle, magnitude, seg);
    segments.push_back(seg);
  }

	std::cout << "Have " << segments.size() << " segments" << std::endl;

  if( _saveDebugImages ) saveLineSegments( segments );
}


///==== Debug image functions =====

bool DetectorBase::validDebugImage( unsigned int which )
{
	return which < _debugImages.size();
}

Mat DetectorBase::debugImage( unsigned int which )
{
	if( validDebugImage( which ) )
		return _debugImages[which];
	return Mat();
}

void DetectorBase::saveDebugImage( const Mat &img, unsigned int which, bool clone )
{
	if( validDebugImage( which ) ) {
		if( clone )
			img.copyTo(_debugImages[which]);
		else
			_debugImages[which] = img;
	}
}

void DetectorBase::saveOriginalImage( const Mat &img )
{
	saveDebugImage( img, OriginalImage );

	// Convenience temporary
	Mat originalBgr;
	cvtColor( img, originalBgr, cv::COLOR_GRAY2BGR );
	saveDebugImage( originalBgr, OriginalBGRImage, false );
}

void DetectorBase::saveLineSegments( const vector<Segment> &segments )
{
	Mat savedLineSegmentsImage;
	// Boost back to color imagery for better annotation
	debugImage( OriginalBGRImage ).copyTo( savedLineSegmentsImage );

	CV_Assert( savedLineSegmentsImage.type() == CV_32FC3 );

	for( vector<Segment>::const_iterator it = segments.begin(); it!=segments.end(); it++ ) {
		long r = random();
		cv::line(savedLineSegmentsImage,
							cv::Point2f(it->getX0(), it->getY0()),
							cv::Point2f(it->getX1(), it->getY1()),
							cv::Scalar( (float)(r & 0xFF)/255, (float)((r&0xFF00)>>8)/255, (float)((r&0xFF0000)>>8)/255 ),
							savedLineSegmentsImage.cols * 0.005 );
	}

	saveDebugImage( savedLineSegmentsImage, LineSegmentsImage, false );
}

}
