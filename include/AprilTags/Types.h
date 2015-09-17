#ifndef __TYPES_H__
#define __TYPES_H__

namespace AprilTags {

  // The original unsigned long long is not universally defined (only in the
  // C99 and C++11 standards?).  Use a typedef to allow easy substitution when
  // necessary.  The fact this is typedef'd does not imply that any of
  // the code _doesn't_ assume that it's a 64-bit unsigned int.
  typedef uint64_t Code_t;

}

#endif
