/* SPDX-License-Identifier: BSD-3-Clause
 * Modified C99 adaptation of BRISK harris-scores.cc, harris-score-calculator.cc,
 * scale-space-layer-inl.h and uniformity-enforcement-inl.h.
 * See ../reference_brisk/LICENSE-BRISK for retained upstream notices.
 */
#include "ok_brisk.h"
#include <math.h>
#include <limits.h>
#include <stdlib.h>
#include <string.h>
#include <stdbool.h>
static void emit(const ok_brisk_trace*t,const char*s,const void*p,size_t n){if(t&&t->emit)t->emit(t->user,s,p,n);}
static int arshift(int x,unsigned n){return x>=0?x/(1<<n):-1-((-1-x)/(1<<n));}
static int smooth(const int16_t*p,int w){return arshift(4*p[0]+2*(p[-w]+p[w]+p[-1]+p[1])+p[-w-1]+p[-w+1]+p[w-1]+p[w+1],4);}
/* Independent comparison-sort implementation, validated against the native
 * std::sort outputs, including ties. No libstdc++ source used/copied.
 * Median-of-three partition, 16-element cutoff, then insertion finishing. */
static int less(ok_brisk_maximum a,ok_brisk_maximum b){return a.score>b.score;}
static void swap(ok_brisk_maximum*a,ok_brisk_maximum*b){ok_brisk_maximum v=*a;*a=*b;*b=v;}
static void heapdown(ok_brisk_maximum*a,size_t n,size_t k){
 for(;;){size_t c=k*2+1;if(c>=n)break;if(c+1<n&&!less(a[c+1],a[c]))c++;if(!less(a[k],a[c]))break;swap(a+k,a+c);k=c;}
}
static void quick(ok_brisk_maximum*a,size_t n,unsigned depth){
 while(n>16){
  if(!depth){for(size_t k=n/2;k>0;)heapdown(a,n,--k);for(size_t k=n;k>1;){swap(a,a+--k);heapdown(a,k,0);}return;}depth--;
  size_t x=1,y=n/2,z=n-1,m;
  if(less(a[x],a[y]))m=less(a[y],a[z])?y:(less(a[x],a[z])?z:x);
  else m=less(a[x],a[z])?x:(less(a[y],a[z])?z:y);
  swap(a,a+m);size_t i=1,j=n;
  for(;;){while(less(a[i],a[0]))i++;do{j--;}while(less(a[0],a[j]));if(i>=j)break;swap(a+i,a+j);i++;}
  quick(a+i,n-i,depth);n=i;
 }
}
static void sort(ok_brisk_maximum*a,size_t n){unsigned d=0;for(size_t v=n;v>1;v>>=1)d++;quick(a,n,2*d);for(size_t i=1;i<n;i++){ok_brisk_maximum v=a[i];size_t j=i;while(j&&less(v,a[j-1])){a[j]=a[j-1];j--;}a[j]=v;}}
#define delta_x (*dx)
#define delta_y (*dy)
static float subpixel(
    const double s_0_0, const double s_0_1, const double s_0_2,
    const double s_1_0, const double s_1_1, const double s_1_2,
    const double s_2_0, const double s_2_1, const double s_2_2, float* dx,
    float* dy) {
  // The coefficients of the 2d quadratic function least-squares fit:
  double tmp1 = s_0_0 + s_0_2 - 2 * s_1_1 + s_2_0 + s_2_2;
  double coeff1 = 3 * (tmp1 + s_0_1 - ((s_1_0 + s_1_2) / 2.0) + s_2_1);
  double coeff2 = 3 * (tmp1 - ((s_0_1 + s_2_1) / 2.0) + s_1_0 + s_1_2);
  double tmp2 = s_0_2 - s_2_0;
  double tmp3 = (s_0_0 + tmp2 - s_2_2);
  double tmp4 = tmp3 - 2 * tmp2;
  double coeff3 = -3 * (tmp3 + s_0_1 - s_2_1);
  double coeff4 = -3 * (tmp4 + s_1_0 - s_1_2);
  double coeff5 = (s_0_0 - s_0_2 - s_2_0 + s_2_2) / 4.0;
  double coeff6 = -(s_0_0 + s_0_2
      - ((s_1_0 + s_0_1 + s_1_2 + s_2_1) / 2.0) - 5 * s_1_1 + s_2_0 + s_2_2)
      / 2.01;

  // 2nd derivative test:
  double H_det = 4 * coeff1 * coeff2 - coeff5 * coeff5;

  if (H_det == 0) {
    delta_x = 0.0;
    delta_y = 0.0;
    return (float)(coeff6) / 18.0;
  }

  if (!(H_det > 0 && coeff1 < 0)) {
    // The maximum must be at the one of the 4 patch corners.
    int tmp_max = coeff3 + coeff4 + coeff5;
    delta_x = 1.0;
    delta_y = 1.0;

    int tmp = -coeff3 + coeff4 - coeff5;
    if (tmp > tmp_max) {
      tmp_max = tmp;
      delta_x = -1.0;
      delta_y = 1.0;
    }
    tmp = coeff3 - coeff4 - coeff5;
    if (tmp > tmp_max) {
      tmp_max = tmp;
      delta_x = 1.0;
      delta_y = -1.0;
    }
    tmp = -coeff3 - coeff4 + coeff5;
    if (tmp > tmp_max) {
      tmp_max = tmp;
      delta_x = -1.0;
      delta_y = -1.0;
    }
    return (float)(tmp_max + coeff1 + coeff2 + coeff6) / 18.0;
  }

  // This is hopefully the normal outcome of the Hessian test.
  delta_x = (float)(2 * coeff2 * coeff3 - coeff4 * coeff5) /
      (float)(-H_det);
  delta_y = (float)(2 * coeff1 * coeff4 - coeff3 * coeff5) /
      (float)(-H_det);
  // TODO(lestefan): this is not correct, but easy, so perform a real boundary
  // maximum search:
  bool tx = false;
  bool tx_ = false;
  bool ty = false;
  bool ty_ = false;
  if (delta_x > 1.0)
    tx = true;
  else if (delta_x < -1.0)
    tx_ = true;
  if (delta_y > 1.0)
    ty = true;
  if (delta_y < -1.0)
    ty_ = true;

  if (tx || tx_ || ty || ty_) {
    // Get two candidates:
    float delta_x1 = 0.0, delta_x2 = 0.0, delta_y1 = 0.0, delta_y2 = 0.0;
    if (tx) {
      delta_x1 = 1.0;
      delta_y1 = -(float)(coeff4 + coeff5) /
          (float)(2 * coeff2);
      if (delta_y1 > 1.0)
        delta_y1 = 1.0;
      else if (delta_y1 < -1.0)
        delta_y1 = -1.0;
    } else if (tx_) {
      delta_x1 = -1.0;
      delta_y1 = -(float)(coeff4 - coeff5) /
          (float)(2 * coeff2);
      if (delta_y1 > 1.0)
        delta_y1 = 1.0;
      else if (delta_y1 < -1.0)
        delta_y1 = -1.0;
    }
    if (ty) {
      delta_y2 = 1.0;
      delta_x2 = -(float)(coeff3 + coeff5) /
          (float)(2 * coeff1);
      if (delta_x2 > 1.0)
        delta_x2 = 1.0;
      else if (delta_x2 < -1.0)
        delta_x2 = -1.0;
    } else if (ty_) {
      delta_y2 = -1.0;
      delta_x2 = -(float)(coeff3 - coeff5) /
          (float)(2 * coeff1);
      if (delta_x2 > 1.0)
        delta_x2 = 1.0;
      else if (delta_x2 < -1.0)
        delta_x2 = -1.0;
    }
    // Insert both options for evaluation which to pick.
    float max1 = (coeff1 * delta_x1 * delta_x1 + coeff2 * delta_y1 * delta_y1
        + coeff3 * delta_x1 + coeff4 * delta_y1 + coeff5 * delta_x1 * delta_y1
        + coeff6) / 18.0;
    float max2 = (coeff1 * delta_x2 * delta_x2 + coeff2 * delta_y2 * delta_y2
        + coeff3 * delta_x2 + coeff4 * delta_y2 + coeff5 * delta_x2 * delta_y2
        + coeff6) / 18.0;
    if (max1 > max2) {
      delta_x = delta_x1;
      delta_y = delta_x1;
      return max1;
    } else {
      delta_x = delta_x2;
      delta_y = delta_x2;
      return max2;
    }
  }
  // This is the case of the maximum inside the boundaries:
  return (coeff1 * delta_x * delta_x + coeff2 * delta_y * delta_y
      + coeff3 * delta_x + coeff4 * delta_y + coeff5 * delta_x * delta_y
      + coeff6) / 18.0;
}

#undef delta_x
#undef delta_y
int ok_brisk_detect(const uint8_t*im,int w,int h,double radius,int threshold,size_t maximum,
                    ok_brisk_keypoint**out,size_t*count,const ok_brisk_trace*t){
 if(!out||!count)return -1;
 *out=NULL;*count=0;
 if(!im||w<20||h<20||w>65535||h>65535||(size_t)w*h>INT_MAX/255||!isfinite(radius)||radius<1||threshold<1||maximum==0)return -1;
 size_t n=(size_t)w*h;int16_t *xx=calloc(n,2),*xy=calloc(n,2),*yy=calloc(n,2);int32_t*s=calloc(n,4);ok_brisk_maximum*p=malloc(n*sizeof(*p));
 if(!xx||!xy||!yy||!s||!p){free(xx);free(xy);free(yy);free(s);free(p);return -1;}
 for(int y=1;y<h-1;y++)for(int x=1;x<w-1;x++){
  size_t i=(size_t)y*w+x;const uint8_t*q=im+i;
  int dx=8*(10*(q[-1]-q[1])+3*(q[-w-1]-q[-w+1])+3*(q[w-1]-q[w+1]));
  int dy=8*(10*(q[-w]-q[w])+3*(q[-w-1]-q[w-1])+3*(q[-w+1]-q[w+1]));
  xx[i]=arshift(dx*dx,16);xy[i]=arshift(dx*dy,16);yy[i]=arshift(dy*dy,16);
 }
 for(int y=2;y<h-2;y++)for(int x=2;x<w-2;x++){
  size_t i=(size_t)y*w+x;int a=smooth(xx+i,w),b=smooth(xy+i,w),c=smooth(yy+i,w),tr=(a+c)/2;s[i]=a*c-b*b-(tr*tr)/5;
 }
 free(xx);free(xy);free(yy);emit(t,"scores",s,n*4);
 size_t np=0;
 for(int y=2;y<h-2;y++)for(int x=2;x<w-2;x++){
  const int32_t*q=s+(size_t)y*w+x;int v=*q;if(v<threshold)continue;
  if(q[-1]>v||q[1]>v||q[-w]>v||q[w]>v||q[-w-1]>v||q[-w+1]>v||q[w-1]>v||q[w+1]>v)continue;
  p[np++]=(ok_brisk_maximum){v,(uint16_t)x,(uint16_t)y};
 }
 emit(t,"maxima",p,np*sizeof(*p));
 if(!np){free(p);free(s);emit(t,"detected",NULL,0);return 0;}
 sort(p,np);emit(t,"sorted",p,np*sizeof(*p));
 float lut[31*31];for(int x=0;x<31;x++)for(int y=0;y<31;y++){double a=1-sqrt(sqrt(sqrt((double)((15-x)*(15-x)+(15-y)*(15-y)))))/sqrt(sqrt(sqrt(225.0)));lut[y*31+x]=(float)(a>0?a:0);}
 float scaling=15.0/(float)radius;int ow=w*(int)ceil(scaling)+32,oh=h*(int)ceil(scaling)+32;uint8_t*occ=calloc((size_t)ow*oh,1);
 if(!occ){free(p);free(s);return -1;}float maxscore=p[0].score;size_t kept=0;
 for(size_t i=0;i<np;i++){
  int cx=p[i].x*scaling+16,cy=p[i].y*scaling+16;float nsc1=sqrtf(sqrtf(p[i].score/maxscore))*255.0f;
  if(nsc1<occ[(size_t)cy*ow+cx])continue;
  float nsc=.99f*nsc1;
  for(int y=0;y<31;y++)for(int x=0;x<31;x++){size_t j=(size_t)(cy+y-15)*ow+cx+x-15;unsigned v=occ[j]+(unsigned)ceil(lut[y*31+x]*nsc);occ[j]=v>255?255:v;}
  p[kept++]=p[i];if(kept==maximum)break;
 }
 free(occ);emit(t,"selected",p,kept*sizeof(*p));ok_brisk_keypoint*k=calloc(kept,sizeof(*k));if(!k&&kept){free(p);free(s);return -1;}
 for(size_t i=0;i<kept;i++){int x=p[i].x,y=p[i].y;const int32_t*q=s+(size_t)y*w+x;float dx,dy;subpixel(q[-w-1],q[-w],q[-w+1],q[-1],q[0],q[1],q[w-1],q[w],q[w+1],&dx,&dy);k[i]=(ok_brisk_keypoint){x+dx,y+dy,12.f,-1.f,(float)p[i].score,0,-1};}
 free(s);free(p);emit(t,"detected",k,kept*sizeof(*k));*out=k;*count=kept;return 0;
}
