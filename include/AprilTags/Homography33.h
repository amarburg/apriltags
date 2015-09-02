//-*-c++-*-

#ifndef HOMOGRAPHY33_H
#define HOMOGRAPHY33_H

#include <utility>
#include <vector>

#include <Eigen/Dense>

// interpolate points instead of using homography
#define INTERPOLATE
// use stable version of homography recover (opencv, includes refinement step)
#define STABLE_H

// TODO:  There's a wierd Opencv-to-Eigen impedence mismatch when entering
//        this class.  Is it strictly necessary?
//        Esp. under the STABLE_H calculation it just farms the homography
//        calculation out to OpenCV anyway...

//! Compute 3x3 homography using Direct Linear Transform
/*
 *
 *  DEPRECATED - DEPRECATED - DEPRECATED - DEPRECATED
 *
 *  use TagDetection::getRelativeTransform() instead
 *
 *
 *  y = Hx (y = image coordinates in homogeneous coordinates, H = 3x3
 *  homography matrix, x = homogeneous 2D world coordinates)
 *
 *  For each point correspondence, constrain y x Hx = 0 (where x is
 *  cross product). This means that they have the same direction, and
 *  ignores scale factor.
 *
 *  We rewrite this as Ah = 0, where h is the 9x1 column vector of the
 *  elements of H. Each point correspondence gives us 3 equations of
 *  rank 2. The solution h is the minimum right eigenvector of A,
 *  which we can obtain via SVD, USV' = A. h is the right-most column
 *  of V'.
 *
 *  We will actually maintain A'A internally, which is 9x9 regardless
 *  of how many correspondences we're given, and has the same
 *  eigenvectors as A.
 */
class Homography33 {
public:
  //! Constructor
  Homography33(const Eigen::Vector2f &opticalCenter);

#ifdef STABLE_H
  void setCorrespondences(const std::vector< std::pair<float,float> > &srcPts,
                          const std::vector< std::pair<float,float> > &dstPts);
#else
  void addCorrespondence(float worldx, float worldy, float imagex, float imagey);
#endif

  //! Note that the returned H matrix does not reflect cxy.
  Eigen::Matrix3d& getH();

  const Eigen::Vector2f getCXY() const { return cxy; }

  void compute();

  std::pair<float,float> project(float worldx, float worldy);

private:
  Eigen::Vector2f cxy;
  Eigen::Matrix<double,9,9> fA;
  Eigen::Matrix3d H;
  bool valid;

#ifdef STABLE_H
  std::vector< std::pair<float,float> > srcPts, dstPts;
#endif
};

#endif
