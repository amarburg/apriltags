
#pragma once

#include <gtest/gtest.h>
#include <opencv2/core/core.hpp>


#ifndef TEST_DATA_DIR
  #error "CMake should define TEST_DATA_DIR, but it hasn't."
#endif

#define TEST_36H11_JPG            (TEST_DATA_DIR "/test_36h11.jpg")
#define TEST_36H11_GREYSCALE_JPG  (TEST_DATA_DIR "/test_36h11_greyscale.jpg")

#define TEST_36H11_OBLIQUE_JPG            (TEST_DATA_DIR "/test_36h11_oblique.jpg")
#define TEST_36H11_OBLIQUE_GREYSCALE_JPG  (TEST_DATA_DIR "/test_36h11_oblique_greyscale.jpg")

#define TEST_36H11_CLOSE_JPG            (TEST_DATA_DIR "/test_36h11_close.jpg")



#include "AprilTags/TagFamily.h"
#include "AprilTags/Tag36h11.h"
#include "AprilTags/TagArray.h"
static const AprilTags::TagCodes &whichCode( AprilTags::tagCodes36h11 );


// Ground truth data for Corner detection
struct TagGroundTruth {
  unsigned int id;
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

namespace AprilTags {

  namespace TestData {

    struct TagArrayGroundTruth {
      std::vector< unsigned char > elems;
      unsigned int cols, rows;
      float dx, dy;

      TagArrayGroundTruth( const unsigned char *arr, unsigned int num, unsigned int c, int r, float x, float y )
        : elems( arr, arr+num ),
          cols( c ),
          rows( r ),
          dx( x ),
          dy( y )
      {
        EXPECT_EQ( num, cols*rows );
      }

      TagArray operator()( void ) const {
        TagArray out( whichCode );
        for( unsigned int y = 0; y < rows; ++y ) {
          for( unsigned int x = 0; x < cols; ++x ) {
            out.add( cv::Point2f( x*dx, y*dy ), elems[y*cols + x] );
          }
        }
        return out;
      }
    };

    const unsigned char t36h11Array[] =
        {  14,15,16,17,18,19,20,21,22,23,
            38,39,40,41,42,43,44,45,46,47,
            62,63,64,65,66,67,68,69,70,71,
            86,87,88,89,90,91,92,93,94,95,
            110,111,112,113,114,115,116,117,118,119,
            134,135,136,137,138,139,140,141,142,143,
            158,159,160,161,162,163,164,165,166,167,
            182,183,184,185,186,187,188,189,190,191, };

    const unsigned int t36h11ArrayWidth = 10;
    const unsigned int t36h11ArrayHeight = 8;
    const unsigned int t36h11ArraySize = t36h11ArrayWidth*t36h11ArrayHeight;


    static const TagArrayGroundTruth GroundTruthArray = TagArrayGroundTruth( t36h11Array, sizeof(t36h11Array)/sizeof( t36h11Array[0]),
                                                                            10, 8, 1.2,1.2 );

  }
}
