#include "rasterizer.h"
#include <stdlib.h>

#ifdef __cplusplus
#include <vector>
#endif

/* Utility Functions */

/*
 *   Function: min
 *  Function Description: Returns the minimum value of two integers and b.
*/
int min(int a, int b)
{
  // START CODE HERE
  return (a < b) ? a : b;
  // END CODE HERE
}

/*
 *   Function: max
 *   Function Description: Returns the maximum value of two integers and b.
*/
int max(int a, int b)
{
  // START CODE HERE
  return (a > b) ? a : b;
  // END CODE HERE
}

/*
/   Function: floor_ss
/   Function Description: Returns a fixed point value rounded down to the subsample grid.
*/
int floor_ss(int val, int r_shift, int ss_w_lg2)
{
  // START CODE HERE
  int bits_to_mask = r_shift - ss_w_lg2;
  int mask = 0xffffffff << bits_to_mask;
  return val & mask;
  // END CODE HERE
}

/*
 *  Function: rastBBox_bbox_fix
 *  Function Description: Determine a bounding box for the triangle.
 *  Note that this is a fixed point function.
*/
BoundingBox get_bounding_box(Triangle triangle, Screen screen, Config config)
{
  BoundingBox bbox;

  // START CODE HERE
  // initialize bounding box to first vertex
  Vertex2D ll;
  Vertex2D ur;
  
  ll.x = triangle.v[0].x;
  ll.y = triangle.v[0].y;
 
  ur.x = triangle.v[0].x;
  ur.y = triangle.v[0].y;

  // iterate over remaining vertices

  for (int i = 1; i < 3; i++){
  	ll.x = min(triangle.v[i].x, ll.x);
	  ll.y = min(triangle.v[i].y, ll.y);
        
  	ur.x = max(triangle.v[i].x, ur.x);
	  ur.y = max(triangle.v[i].y, ur.y);
  }

  // round down to subsample grid
  ll.x = floor_ss(ll.x, config.r_shift, config.ss_w_lg2);
   
  ll.y = floor_ss(ll.y, config.r_shift, config.ss_w_lg2);
   
  ur.x = floor_ss(ur.x, config.r_shift, config.ss_w_lg2);
   
  ur.y = floor_ss(ur.y, config.r_shift, config.ss_w_lg2);
   
  // clip to screen

  if (ll.x < 0){ ll.x = 0; }

  if (ll.y < 0){ ll.y = 0; }

  if (ur.x > screen.width){ ur.x = screen.width; }

  if (ur.y > screen.height){ ur.y = screen.height; }


  // check if bbox is valid
  bbox.lower_left = ll;
  bbox.upper_right = ur;

  bbox.valid = (bbox.upper_right.x >= 0) && (bbox.upper_right.y >= 0) && (bbox.lower_left.x < screen.width) && (bbox.lower_left.y < screen.height);

  // END CODE HERE

  return bbox;
}

/*
 *  Function: sample_test
 *  Function Description: Checks if sample lies inside triangle
 *
 *
 */
bool sample_test(Triangle triangle, Sample sample)
{
  bool isHit;

  // START CODE HERE
  Vertex2D v1,v2,v3;

  v1.x = triangle.v[0].x;
  v1.y = triangle.v[0].y;

  
  v2.x = triangle.v[1].x;
  v2.y = triangle.v[1].y;

  v3.x = triangle.v[2].x;
  v3.y = triangle.v[2].y;

  v1.x = v1.x - sample.x;
  v1.y = v1.y - sample.y;

  v2.x = v2.x - sample.x;
  v2.y = v2.y - sample.y;

  v3.x = v3.x - sample.x;
  v3.y = v3.y - sample.y;
  
  // V1 --> V2 test
  
  bool t1 = (v1.x*v2.y - v2.x*v1.y) <= 0.0;
  // V2 --> V3
  bool t2 = (v2.x*v3.y - v3.x*v2.y) < 0.0;
  // V3 --> V1
  bool t3 = (v3.x*v1.y - v1.x*v3.y) <= 0.0;
  // END CODE HERE
  
  //isHit = (t1 && t2 && t3) || (!t1 && !t2 && !t3);
  isHit = t1 && t2 && t3;
  return isHit;
}

int rasterize_triangle(Triangle triangle, ZBuff *z, Screen screen, Config config)
{
  int hit_count = 0;

  //Calculate BBox
  BoundingBox bbox = get_bounding_box(triangle, screen, config);

  //Iterate over samples and test if in triangle
  Sample sample;
  for (sample.x = bbox.lower_left.x; sample.x <= bbox.upper_right.x; sample.x += config.ss_i)
  {
    for (sample.y = bbox.lower_left.y; sample.y <= bbox.upper_right.y; sample.y += config.ss_i)
    {

      Sample jitter = jitter_sample(sample, config.ss_w_lg2);
      jitter.x = jitter.x << 2;
      jitter.y = jitter.y << 2;

      Sample jittered_sample;
      jittered_sample.x = sample.x + jitter.x;
      jittered_sample.y = sample.y + jitter.y;

      bool hit = sample_test(triangle, jittered_sample);

      if (hit)
      {
        hit_count++;
        if (z != NULL)
        {
          Sample hit_location;
          hit_location.x = sample.x >> config.r_shift;
          hit_location.y = sample.y >> config.r_shift;

          Sample subsample;
          subsample.x = (sample.x - (hit_location.x << config.r_shift)) / config.ss_i;
          subsample.y = (sample.y - (hit_location.y << config.r_shift)) / config.ss_i;

          Fragment f;
          f.z = triangle.v[0].z;
          f.R = triangle.v[0].R;
          f.G = triangle.v[0].G;
          f.B = triangle.v[0].B;

          process_fragment(z, hit_location, subsample, f);
        }
      }
    }
  }

  return hit_count;
}

void hash_40to8(uchar *arr40, ushort *val, int shift)
{
  uchar arr32[4];
  uchar arr16[2];
  uchar arr8;

  ushort mask = 0x00ff;
  mask = mask >> shift;

  arr32[0] = arr40[0] ^ arr40[1];
  arr32[1] = arr40[1] ^ arr40[2];
  arr32[2] = arr40[2] ^ arr40[3];
  arr32[3] = arr40[3] ^ arr40[4];

  arr16[0] = arr32[0] ^ arr32[2];
  arr16[1] = arr32[1] ^ arr32[3];

  arr8 = arr16[0] ^ arr16[1];

  mask = arr8 & mask;
  val[0] = mask;
}

Sample jitter_sample(const Sample sample, const int ss_w_lg2)
{
  long x = sample.x >> 4;
  long y = sample.y >> 4;
  uchar arr40_1[5];
  uchar arr40_2[5];

  long *arr40_1_ptr = (long *)arr40_1;
  long *arr40_2_ptr = (long *)arr40_2;

  ushort val_x[1];
  ushort val_y[1];

  *arr40_1_ptr = (y << 20) | x;
  *arr40_2_ptr = (x << 20) | y;

  hash_40to8(arr40_1, val_x, ss_w_lg2);
  hash_40to8(arr40_2, val_y, ss_w_lg2);

  Sample jitter;
  jitter.x = val_x[0];
  jitter.y = val_y[0];

  return jitter;
}
