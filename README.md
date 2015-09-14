AprilTags
=========

 * CI build status:[![Build Status](https://travis-ci.org/amarburg/apriltags.svg?branch=travis_integration)](https://travis-ci.org/amarburg/apriltags)

Detects [AprilTags](https://april.eecs.umich.edu/wiki/index.php/AprilTags)
(2D bar codes) in images; reports unique ID of each
detection, and optionally its position and orientation relative to a
calibrated camera.  

This code is forked from Michael Kaess's [C++ implementation](http://people.csail.mit.edu/kaess/apriltags/)
and maintains his LGPL 2.1 license.   The fork is partly
to explore new ideas and also to find additional performance.  As much as
possible, it relies on [OpenCV](http://opencv.org/) for image
manipulation primitives.  At present I am working with OpenCV 3.0.  I welcome
any patches for backwards compatibility with 2.x.

The code is presently a mishmash of OpenCV and Eigen primitives with a smattering
of hand-written code.  I'm always looking for refactors which reduce
the code complexity (reducing the dependencies and/or the number of times
you have to cross the OpenCV/Eigen interface) while increasing
performance.

It is packaged as a standard [CMake](http://www.cmake.org/) package.    Building
is something like:

    git clone https://github.com/amarburg/apriltags.git
    cd apriltags && mkdir build && cd build
    cmake ..
    make
    make test              (for unit tests)
    make perf              (for performance tests)



TODO:
-----

 * Backwards compatibility with OpenCV 2.x



---

Original README.txt
===================

See examples/apriltags_demo.cpp for a simple example that detects
April tags (see tags/pdf/tag36h11.pdf) in laptop or webcam images and
marks any tags in the live image.

Ubuntu dependencies:
sudo apt-get install subversion cmake libopencv-dev libeigen3-dev libv4l-dev

Mac dependencies:
sudo port install pkgconfig opencv eigen3

Uses the pods build system in connection with cmake, see:
http://sourceforge.net/p/pods/

Michael Kaess
October 2012

----------------------------

AprilTags were developed by Professor Edwin Olson of the University of
Michigan.  His Java implementation is available on this web site:
  http://april.eecs.umich.edu.

Olson's Java code was ported to C++ and integrated into the Tekkotsu
framework by Jeffrey Boyland and David Touretzky.

See this Tekkotsu wiki article for additional links and references:
  http://wiki.tekkotsu.org/index.php/AprilTags

----------------------------

This C++ code was further modified by
Michael Kaess (kaess@mit.edu) and Hordur Johannson (hordurj@mit.edu)
and the code has been released under the LGPL 2.1 license.

- converted to standalone library
- added stable homography recovery using OpenCV
- robust tag code table that does not require a terminating 0
  (omission results in false positives by illegal codes being accepted)
- changed example tags to agree with Ed Olson's Java version and added
  all his other tag families
- added principal point as parameter as in original code - essential
  for homography
- added some debugging code (visualization using OpenCV to show
  intermediate detection steps)
- added fast approximation of arctan2 from Ed's original Java code
- using interpolation instead of homography in Quad: requires less
  homography computations and provides a small improvement in correct
  detections

todo:
- significant speedup could be achieved by performing image operations
  using OpenCV (Gaussian filter, but also operations in
  TagDetector.cc)
- replacing arctan2 by precomputed lookup table
- converting matrix operations to Eigen (mostly for simplifying code,
  maybe some speedup)
