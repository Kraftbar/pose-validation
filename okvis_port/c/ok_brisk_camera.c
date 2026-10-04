/* SPDX-License-Identifier: BSD-3-Clause AND Apache-2.0
 * BRISK camera warp adapted to scalar C99; OpenCV 4.6 Matx vector operations,
 * GEMM float/double accumulation and the two-dimensional Jacobi eigenvalue
 * path. See ../reference_brisk/LICENSE-BRISK and LICENSE-OpenCV*. */
#include <math.h>
#include <float.h>
#include <stddef.h>
static void cross(const float*a,const float*b,float*r){r[0]=a[1]*b[2]-a[2]*b[1];r[1]=a[2]*b[0]-a[0]*b[2];r[2]=a[0]*b[1]-a[1]*b[0];}
static float dot(const float*a){float s=0;for(int i=0;i<3;i++)s+=a[i]*a[i];return s;}
static void normalize(float*a){double s=0;for(int i=0;i<3;i++){double v=a[i];s+=v*v;}double n=sqrt(s),scale=n?1./n:0;for(int i=0;i<3;i++)a[i]=(float)(a[i]*scale);}
static float cvhypot(float a,float b){a=fabsf(a);b=fabsf(b);if(a>b){b/=a;return a*sqrtf(1+b*b);}if(b>0){a/=b;return b*sqrtf(1+a*a);}return 0;}
int ok_brisk_camera_warp(float x,float y,int w,const float*rays,const float*jacs,float focal,const float dir[3],float warp[6]){
 int x0=(int)floorf(x),y0=(int)floorf(y);float dx=x-x0,dy=y-y0,wt[4]={(1.f-dx)*(1.f-dy),dx*(1.f-dy),(1.f-dx)*dy,dx*dy};
 const float*r[4]={rays+3*((size_t)y0*w+x0),rays+3*((size_t)y0*w+x0+1),rays+3*((size_t)(y0+1)*w+x0),rays+3*((size_t)(y0+1)*w+x0+1)};
 for(int i=0;i<4;i++)if(!(dot(r[i])>1e-12))return 0;
 float ray[3],eu[3],ev[3];for(int j=0;j<3;j++)ray[j]=((wt[0]*r[0][j]+wt[1]*r[1][j])+wt[2]*r[2][j])+wt[3]*r[3][j];
 cross(dir,ray,eu);warp[5]=(eu[0]*eu[0]+eu[1]*eu[1]+eu[2]*eu[2]>.01)?1.f:0.f;normalize(eu);float f=1.f/focal;for(int j=0;j<3;j++)eu[j]=f*eu[j];cross(ray,eu,ev);normalize(ev);for(int j=0;j<3;j++)ev[j]=f*ev[j];
 int xm=(int)roundf(x),ym=(int)roundf(y);const float*J=jacs+6*((size_t)ym*w+xm);
 for(int r0=0;r0<2;r0++){warp[r0*2]=(float)(((double)J[r0*3]*eu[0]+(double)J[r0*3+1]*eu[1])+(double)J[r0*3+2]*eu[2]);warp[r0*2+1]=(float)(((double)J[r0*3]*ev[0]+(double)J[r0*3+1]*ev[1])+(double)J[r0*3+2]*ev[2]);}
 /* cv::eigen treats only the upper triangle, even for a nonsymmetric warp. */
 float a=warp[0],b=warp[3],p=warp[1];
 if(fabsf(p)>FLT_EPSILON){float y0=(float)((b-a)*.5);float t=fabsf(y0)+cvhypot(p,y0);t=(p/t)*p;if(y0<0)t=-t;a-=t;b+=t;}
 warp[4]=(float)(.5f*(fabs(a)+fabs(b)));return 1;
}
