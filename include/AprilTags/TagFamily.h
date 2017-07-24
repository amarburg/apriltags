#ifndef __APRILTAGS_TAGFAMILY_H
#define __APRILTAGS_TAGFAMILY_H

#include <climits>
#include <cmath>
#include <stdio.h>
#include <vector>
#include <map>

#include <assert.h>

#include "AprilTags/Types.h"
#include "AprilTags/TagDetection.h"
using namespace std;

namespace AprilTags {

class TagCodes {
public:
  int dim;
  int minHammingDistance;

  std::vector<Code_t> codes;

public:

  // n.b. the change in API from Kaess' version.  TagCodes are now specified by
  // their dimension (edge length), not the number of bits.   bits = dimension^2
  // but it made far more sense to specify dim and square it then specify
  // bits and sqrt it.  Also allows for rectangular tags in the future...
 TagCodes(int d, int minHammingDistance, const Code_t *codesA, int num)
   : dim( d ),
     minHammingDistance(minHammingDistance),
     codes(codesA, codesA+num) // created vector for all entries of codesA
      { ; }

  Code_t operator[]( unsigned int which ) const
  {
    // TODO: Bounds checking currently a fatal error.
    assert( which < codes.size() );
    return codes[which];
  }

  unsigned int size( void ) const { return codes.size(); }

};

//! Generic class for all tag encoding families
class TagFamily {
public:
  //! The codes array is not copied internally and so must not be modified externally.
  TagFamily(const TagCodes& tagCodes);

  void setErrorRecoveryBits(int b);

  void setErrorRecoveryFraction(float v);

  /* if the bits in w were arranged in a d*d grid and that grid was
   * rotated, what would the new bits in w be?
   * The bits are organized like this (for d = 3):
   *
   *  8 7 6       2 5 8      0 1 2
   *  5 4 3  ==>  1 4 7 ==>  3 4 5    (rotate90 applied twice)
   *  2 1 0       0 3 6      6 7 8
   */
  static Code_t rotate90(Code_t w, int d);

  //! Computes the hamming distance between two uint64_ts.
  static int hammingDistance(Code_t a, Code_t b);

  //! How many bits are set in the uint64_t?
  static unsigned char popCountReal(Code_t w);

  static int popCount(Code_t w);

  //! Given an observed tag with code 'rCode', try to recover the id.
  /*  The corresponding fields of TagDetection will be filled in. */
  void decode( TagDetection& det, Code_t rCode ) const;

  //! Corner matrices are eager-generated */
//  const cv::Mat &corner( int idx );

  //! Prints the hamming distances of the tag codes.
  void printHammingDistances() const;

  //! Numer of pixels wide of the inner black border.
  int blackBorder;

  //! Number of bits in the tag. Must be n^2.
  int bits() const { return _code.dim * _code.dim; }

  //! Dimension of tag. e.g. for 16 bits, dimension=4. Must be sqrt(bits).
  int dimension() const { return _code.dim; }

  //! Minimum hamming distance between any two codes.
  /*  Accounting for rotational ambiguity? The code can recover
   *  (minHammingDistance-1)/2 bit errors.
   */
  int minimumHammingDistance;

  /* The error recovery value determines our position on the ROC
   * curve. We will report codes that are within errorRecoveryBits
   * of a valid code. Small values mean greater rejection of bogus
   * tags (but false negatives). Large values mean aggressive
   * reporting of bad tags (but with a corresponding increase in
   * false positives).
   */
  int errorRecoveryBits;

  //! Specific code (codebook?) in use
  const TagCodes &_code;
  const TagCodes &codes( void ) const { return _code; }

//  std::vector<Code_t> codes;
//  std::vector<cv::Mat> corners;

  static const int  popCountTableShift = 12;
  static const unsigned int popCountTableSize = 1 << popCountTableShift;
  static unsigned char popCountTable[popCountTableSize];

  //! Initializes the static popCountTable
  static class TableInitializer {
  public:
    TableInitializer() {
      for (unsigned int i = 0; i < TagFamily::popCountTableSize; i++)
        TagFamily::popCountTable[i] = TagFamily::popCountReal(i);
    }
  } initializer;
};

} // namespace

#endif
