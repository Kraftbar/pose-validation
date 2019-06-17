#pragma once


// https://stackoverflow.com/questions/563198/how-do-you-detect-where-two-line-segments-intersect/38112653
// https://math.stackexchange.com/questions/149622/finding-out-whether-two-line-segments-intersect-each-other

// needs check for edgecase
namespace NuGeometry{
bool IsBetween(const double& x0, const double& x, const double& x1){
   return (x >= x0) && (x <= x1);
}

// xy1 -----> xy2
// ab1 -----> ab2

bool FindIntersection (const double& x0, const double& y0, 
                       const double& x1, const double& y1,
                       const double& a0, const double& b0, 
                       const double& a1, const double& b1, 
                             double& xy,       double& ab ) 
{
   // four endpoints are x0, y0 & x1,y1 & a0,b0 & a1,b1
   // returned values xy and ab are the fractional distance along xy and ab
   // and are only defined when the result is true

   bool partial = false;
   // if paralell not intersection 
   double denom = (b0 - b1) * (x0 - x1) - (y0 - y1) * (a0 - a1);
   if (denom == 0) {
      xy = -1;
      ab = -1;
   } else {
      xy = (a0 * (y1 - b1) + a1 * (b0 - y1) + x1 * (b1 - b0)) / denom;
      partial = NuGeometry::IsBetween(0, xy, 1);
      if (partial) {
         // no point calculating this unless xy is between 0 & 1
         ab = (y1 * (x0 - a1) + b1 * (x1 - x0) + y0 * (a1 - x1)) / denom; 
      }
   }
   if ( partial && NuGeometry::IsBetween(0, ab, 1)) {
      ab = 1-ab;
      xy = 1-xy;
      return true;
   }  else return false;
}

}

























// not used 
namespace geohelper  
{ 
bool get_line_intersection(float p0_x, float p0_y, float p1_x, float p1_y, float p2_x, float p2_y, float p3_x, float p3_y, float *i_x, float *i_y)
{
	float s02_x, s02_y, s10_x, s10_y, s32_x, s32_y, s_numer, t_numer, denom, t;
	s10_x = p1_x - p0_x;
	s10_y = p1_y - p0_y;
	s32_x = p3_x - p2_x;
	s32_y = p3_y - p2_y;

	denom = s10_x * s32_y - s32_x * s10_y;
	if (denom == 0)
		return false; // Collinear
	bool denomPositive = denom > 0;

	s02_x = p0_x - p2_x;
	s02_y = p0_y - p2_y;
	s_numer = s10_x * s02_y - s10_y * s02_x;
	if ((s_numer < 0) == denomPositive)
		return false; // No collision

	t_numer = s32_x * s02_y - s32_y * s02_x;
	if ((t_numer < 0) == denomPositive)
		return false; // No collision

	if (((s_numer > denom) == denomPositive) || ((t_numer > denom) == denomPositive))
		return false; // No collision
	// Collision detected
	t = t_numer / denom;
	if (i_x != NULL)
		*i_x = p0_x + (t * s10_x);
	if (i_y != NULL)
		*i_y = p0_y + (t * s10_y);

	return true;
}

}






// https://www.geeksforgeeks.org/check-if-two-given-line-segments-intersect/
// needs diff arguements
// only ints, and does intersection with colinear  
namespace geeks
{
#include <iostream> 
using namespace std; 
  
struct Point 
{ 
    int x; 
    int y; 
}; 
  
// Given three colinear points p, q, r, the function checks if 
// point q lies on line segment 'pr' 
bool onSegment(Point p, Point q, Point r) 
{ 
    if (q.x <= max(p.x, r.x) && q.x >= min(p.x, r.x) && 
        q.y <= max(p.y, r.y) && q.y >= min(p.y, r.y)) 
       return true; 
  
    return false; 
} 
  
// To find orientation of ordered triplet (p, q, r). 
// The function returns following values 
// 0 --> p, q and r are colinear 
// 1 --> Clockwise 
// 2 --> Counterclockwise 
int orientation(Point p, Point q, Point r) 
{ 
    // See https://www.geeksforgeeks.org/orientation-3-ordered-points/ 
    // for details of below formula. 
    int val = (q.y - p.y) * (r.x - q.x) - 
              (q.x - p.x) * (r.y - q.y); 
  
    if (val == 0) return 0;  // colinear 
  
    return (val > 0)? 1: 2; // clock or counterclock wise 
} 
  
// The main function that returns true if line segment 'p1q1' 
// and 'p2q2' intersect. 
bool doIntersect(      const int& x0, const int& y0, 
                       const int& x1, const int& y1,
                       const int& a0, const int& b0, 
                       const int& a1, const int& b1) 
{ 

    struct Point p1 = {x0, y0}, q1 = {x1, y1}; 
    struct Point p2 = {a0, b0}, q2 = {a1, b1}; 



    // Find the four orientations needed for general and 
    // special cases 
    int o1 = orientation(p1, q1, p2); 
    int o2 = orientation(p1, q1, q2); 
    int o3 = orientation(p2, q2, p1); 
    int o4 = orientation(p2, q2, q1); 
  
    // General case 
    if (o1 != o2 && o3 != o4) 
        return true; 
  
    // Special Cases 
    // p1, q1 and p2 are colinear and p2 lies on segment p1q1 
    if (o1 == 0 && onSegment(p1, p2, q1)) return true; 
  
    // p1, q1 and q2 are colinear and q2 lies on segment p1q1 
    if (o2 == 0 && onSegment(p1, q2, q1)) return true; 
  
    // p2, q2 and p1 are colinear and p1 lies on segment p2q2 
    if (o3 == 0 && onSegment(p2, p1, q2)) return true; 
  
     // p2, q2 and q1 are colinear and q1 lies on segment p2q2 
    if (o4 == 0 && onSegment(p2, q1, q2)) return true; 
  
    return false; // Doesn't fall in any of the above cases 
} 
  

}


