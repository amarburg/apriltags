
#ifndef TEST_DATA_DIR
  #error "CMake should define TEST_DATA_DIR, but it hasn't."
#endif

#define TEST_36H11_JPG            (TEST_DATA_DIR "/test_36h11.jpg")
#define TEST_36H11_GREYSCALE_JPG  (TEST_DATA_DIR "/test_36h11_greyscale.jpg")

#include "AprilTags/TagFamily.h"
#include "AprilTags/Tag36h11.h"
static const AprilTags::TagCodes &whichCode( AprilTags::tagCodes36h11 );


// Ground truth data for Corner detection
struct TagGroundTruth {
  int id;
  std::vector< unsigned char > elem;
  std::vector< unsigned char > corners;

  TagGroundTruth( int id_, const unsigned char *el, int elnum, const unsigned char *corn,  int cornum )
   : id( id_ ),
     elem( el, el+elnum ),
     corners( corn, corn+cornum )
  {;}

  unsigned int numCorners( void ) const { return corners.size(); }
  unsigned char corner( unsigned int which ) const { return corners[which]; }

  unsigned int numElement( void ) const { return elem.size(); }
  unsigned char element( unsigned int which ) const { return elem[which]; }

};

struct TagGroundTruths {
  std::vector< TagGroundTruth > elems;

  TagGroundTruths( const TagGroundTruth *arr, int num )
    : elems( arr, arr+num ) {;}

  const TagGroundTruth &operator[]( unsigned int which ) const { return elems[which]; }
  unsigned int size( void ) const { return elems.size(); }

  int find( unsigned int id ) const
  {
    for( unsigned int i = 0; i < size(); ++i ) {
      if( elems[i].id == id ) return i;
    }
    return -1;
  }

};



const unsigned char t36h11id143Corner[] =
  { 0x47, 0x13, 0x13, 0x13, 0x13, 0x13, 0x13, 0x13, 0x4b,
    0x15, 0x28, 0x24, 0x28, 0x1c, 0x1c, 0x24, 0x00, 0x1a,
    0x15, 0x1a, 0x15, 0x1a, 0x0f, 0x47, 0x21, 0x00, 0x1a,
    0x15, 0x22, 0x21, 0x22, 0x4b, 0x15, 0x00, 0x00, 0x1a,
    0x15, 0x28, 0x1c, 0x1c, 0x86, 0x21, 0x28, 0x24, 0x1a,
    0x15, 0x1a, 0x47, 0x13, 0x89, 0x24, 0x1a, 0x15, 0x1a,
    0x15, 0x22, 0x89, 0x24, 0x22, 0x21, 0x22, 0x21, 0x1a,
    0x15, 0x00, 0x22, 0x21, 0x00, 0x00, 0x00, 0x00, 0x1a,
    0x4d, 0x1c, 0x1c, 0x1c, 0x1c, 0x1c, 0x1c, 0x1c, 0x4e };

// Inefficient, but  most robust (?)
const unsigned char t36h11id143Tag[] =
    { 1, 1, 1, 1, 1, 1, 1, 1, 1, 1,
      1, 0, 0, 0, 0, 0, 0, 0, 0, 1,
      1, 0, 1, 0, 1, 1, 1, 0, 0, 1,
      1, 0, 1, 0, 1, 1, 0, 0, 0, 1,
      1, 0, 0, 0, 0, 1, 0, 0, 0, 1,
      1, 0, 1, 1, 1, 0, 0, 1, 0, 1,
      1, 0, 1, 0, 0, 1, 0, 1, 0, 1,
      1, 0, 0, 1, 0, 0, 0, 0, 0, 1,
      1, 0, 0, 0, 0, 0, 0, 0, 0, 1,
      1, 1, 1, 1, 1, 1, 1, 1, 1, 1 };

static const TagGroundTruth t36h11gt[] = { TagGroundTruth(143, t36h11id143Tag,  sizeof(t36h11id143Tag)/sizeof(t36h11id143Tag[0]),
                                                                    t36h11id143Corner, sizeof(t36h11id143Corner)/sizeof(t36h11id143Corner[0]) ) };

static const TagGroundTruths Tag36H11GroundTruths = TagGroundTruths( t36h11gt, sizeof( t36h11gt )/sizeof(t36h11gt[0]) );
