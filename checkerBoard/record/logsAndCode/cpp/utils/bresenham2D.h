// taken, inspired from
// https://gist.github.com/yamamushi/5823518

void SetPixel(int x,int y,int  color){};

// swap
#include <algorithm>    // std::swap

#include "display.h"





void Line( float x1, float y1,  float x2, float y2, cv::viz::Viz3d visualizer  )
{

   int gridCellSize;
   //   Bresenham's line algorithm
  const bool steep = (fabs(y2 - y1) > fabs(x2 - x1));
  if(steep)
  {
    std::swap(x1, y1);
    std::swap(x2, y2);
  }
 
  if(x1 > x2)
  {
    std::swap(x1, x2);
    std::swap(y1, y2);
  }
 
  const float dx = (x2 - x1);
  const float dy = (fabs(y2 - y1));
 
  float error = dx / 2.0f;
  const int ystep = (y1 < y2) ? 30 : -30;
  int y = (int)y1;
 
  const int maxX = (int)x2;
 
  for(int x=(int)x1; x<maxX; x=x+30)
  {
    if(steep)
    {

        paintGrid(y,x, visualizer,30);
    }
    else
    {
        paintGrid(x,y,  visualizer,30);
    }
 
    error -= dy;
    if(error < 0)
    {
        y += ystep;
        error += dx;
    }
  }
}
 
