#include <iostream>

#include "AprilTags/TagFamily.h"

#include <opencv2/core.hpp>

#include "AprilTags/Corners.h"

/**

// example of instantiation of tag family:

#include "AprilTags/TagFamily.h"
#include "AprilTags/Tag36h11.h"
TagFamily *tag36h11 = new TagFamily(tagCodes36h11);

// available tag families:

#include "AprilTags/Tag16h5.h"
#include "AprilTags/Tag16h5_other.h"
#include "AprilTags/Tag25h7.h"
#include "AprilTags/Tag25h9.h"
#include "AprilTags/Tag36h11.h"
#include "AprilTags/Tag36h11_other.h"
#include "AprilTags/Tag36h9.h"

*/


namespace AprilTags {


TagFamily::TagFamily(const TagCodes& tagCodes)
  : blackBorder(1),
    //dimension( tagCodes.dimension ),
    minimumHammingDistance(tagCodes.minHammingDistance),
    errorRecoveryBits(1), //codes(),
    _code( tagCodes )
    //corners( _code.size(), cv::Mat() )
{
  ;
}

void TagFamily::setErrorRecoveryBits(int b) {
  errorRecoveryBits = b;
}

void TagFamily::setErrorRecoveryFraction(float v) {
  errorRecoveryBits = (int) (((int) (minimumHammingDistance-1)/2)*v);
}

Code_t TagFamily::rotate90(Code_t w, int d) {
  Code_t wr = 0;
  const Code_t oneLongLong = 1;

  for (int r = d-1; r>=0; r--) {
    for (int c = 0; c<d; c++) {
      int b = r + d*c;
      wr = wr<<1;

      if ((w & (oneLongLong<<b)) != 0)
	wr |= 1;
    }
  }
  return wr;
}

int TagFamily::hammingDistance(Code_t a, Code_t b) {
  return popCount(a^b);
}

unsigned char TagFamily::popCountReal(Code_t w) {
  unsigned char cnt = 0;
  while (w != 0) {
    w &= (w-1);
    ++cnt;
  }
  return cnt;
}

int TagFamily::popCount(Code_t w) {
  int count = 0;
  while (w != 0) {
    count += popCountTable[(unsigned int) (w & (popCountTableSize-1))];
    w >>= popCountTableShift;
  }
  return count;
}

void TagFamily::decode(TagDetection& det, Code_t rCode) const {
  int  bestId = -1;
  int  bestHamming = INT_MAX;
  int  bestRotation = 0;
  Code_t bestCode = 0;

  Code_t rCodes[4];
  rCodes[0] = rCode;
  rCodes[1] = rotate90(rCodes[0], dimension() );
  rCodes[2] = rotate90(rCodes[1], dimension() );
  rCodes[3] = rotate90(rCodes[2], dimension() );

  for (unsigned int id = 0; id < _code.size(); id++) {
    for (unsigned int rot = 0; rot < 4; rot++) {
      int thisHamming = hammingDistance(rCodes[rot], _code[id]);
      if (thisHamming < bestHamming) {
      	bestHamming = thisHamming;
      	bestRotation = rot;
      	bestId = id;
      	bestCode = _code[id];
      }
    }
  }
  det.id = bestId;
  det.hammingDistance = bestHamming;
  det.rotation = bestRotation;
  det.good = (det.hammingDistance <= errorRecoveryBits);
  det.obsCode = rCode;
  det.code = bestCode;
}

// Eager-generate the corners
// const cv::Mat &TagFamily::corner( int idx )
// {
//   if( corners[idx].empty() )
//      corners[idx] = Corners::makeCornerMat( _code, idx, blackBorder );
//
//   return corners[idx];
// }


void TagFamily::printHammingDistances() const {
  vector<int> hammings(dimension()*dimension()+1);
  for (unsigned i = 0; i < _code.size(); i++) {
    Code_t r0 = _code[i];
    Code_t r1 = rotate90(r0, dimension() );
    Code_t r2 = rotate90(r1, dimension() );
    Code_t r3 = rotate90(r2, dimension() );
    for (unsigned int j = i+1; j < _code.size(); j++) {
      int d = min(min(hammingDistance(r0, _code[j]),
		                  hammingDistance(r1, _code[j])),
		              min(hammingDistance(r2, _code[j]),
		                  hammingDistance(r3, _code[j])));
      hammings[d]++;
    }
  }

  for (unsigned int i = 0; i < hammings.size(); i++)
    printf("hammings: %u = %d\n", i, hammings[i]);
}

unsigned char TagFamily::popCountTable[TagFamily::popCountTableSize];

TagFamily::TableInitializer TagFamily::initializer;

} // namespace
