/* SPDX-License-Identifier: BSD-3-Clause
 * Modified C99 specialization of BRISK brisk-descriptor-extractor.cc.
 * See ../reference_brisk/LICENSE-BRISK. OpenCV camera arithmetic is isolated
 * in ok_brisk_camera.c. No SIMD, C++ runtime, image decoder or external libs.
 */
#include "ok_brisk.h"
#include <math.h>
#include <limits.h>
#include <stdlib.h>
#include <string.h>
#define PI 3.14159265358979323846
#include "ok_brisk_pattern.h"
typedef struct{uint32_t i,j;int32_t dx,dy;} long_pair;
struct ok_brisk_context {ok_brisk_point *pattern;long_pair pairs[856];uint32_t border;int scale;};
static void emit(const ok_brisk_trace*t,const char*s,const void*p,size_t n){if(t&&t->emit)t->emit(t->user,s,p,n);}
void ok_brisk_destroy(ok_brisk_context*c){if(c){free(c->pattern);free(c);}}
ok_brisk_context*ok_brisk_create(const ok_brisk_trace*t){
 ok_brisk_context*c=calloc(1,sizeof(*c));if(!c)return NULL;c->pattern=malloc(1024*66*sizeof(*c->pattern));if(!c->pattern){free(c);return NULL;}
 const float log2=0.693147180559945,lb=log(30.f)/log2,b06=12.f*0.6;
 c->scale=(int)(64/lb*(log(1.45*12.f/b06)/log2)+.5);
 const float lbscale=log(30.f)/log(2.0),step=lbscale/64;float scale=pow(2.0,(double)(c->scale*step));
 for(unsigned rot=0;rot<1024;rot++)for(unsigned i=0;i<66;i++){
  double a=(double)rot*2*PI/1024.0;const ok_brisk_point*b=base_pattern+i;ok_brisk_point*p=c->pattern+rot*66+i;
  p->x=scale*(b->x*cos(a)-b->y*sin(a));p->y=scale*(b->x*sin(a)+b->y*cos(a));p->sigma=1.3f*scale*b->sigma;
  unsigned size=ceil(sqrt(p->x*p->x+p->y*p->y)+p->sigma)+1;if(size>c->border)c->border=size;
 }
 for(unsigned k=0;k<856;k++){unsigned i=long_indices[k][0],j=long_indices[k][1];float dx=base_pattern[j].x-base_pattern[i].x,dy=base_pattern[j].y-base_pattern[i].y,n=dx*dx+dy*dy;c->pairs[k]=(long_pair){i,j,(int)((dx/n)*2048.0+.5),(int)((dy/n)*2048.0+.5)};}
 emit(t,"scale",&c->scale,4);emit(t,"border",&c->border,4);emit(t,"pattern",c->pattern,1024*66*12);emit(t,"short_pairs",short_pairs,sizeof(short_pairs));emit(t,"long_pairs",c->pairs,sizeof(c->pairs));return c;
}
static int intensity(const ok_brisk_context*ctx,const uint8_t*im,int w,int h,const int*integral,float key_x,float key_y,unsigned rot,unsigned point,const float*warp,float warpScale){
  // Get the float position.
  const ok_brisk_point briskPoint0 = ctx->pattern[rot * 66 + point];

  ok_brisk_point briskPoint1 = briskPoint0;
  if(warp) {
    // account for camera model
    briskPoint1.x = warp[0]*briskPoint0.x + warp[1]*briskPoint0.y;
    briskPoint1.y = warp[2]*briskPoint0.x + warp[3]*briskPoint0.y;
    briskPoint1.sigma = warpScale * briskPoint0.sigma; // this should be 2d transformed in theory...
  }
  const ok_brisk_point briskPoint = warp ? briskPoint1 : briskPoint0;

  const float xf = briskPoint.x + key_x;
  const float yf = briskPoint.y + key_y;
  const int x = (int)(xf);
  const int y = (int)(yf);
  const int imagecols = w;

  // Get the sigma:
  const float sigma_half = briskPoint.sigma;
  const float area = 4.0 * sigma_half * sigma_half;

  // Calculate borders.
  const float x_1 = xf - sigma_half;
  const float x1 = xf + sigma_half;
  const float y_1 = yf - sigma_half;
  const float y1 = yf + sigma_half;

  // Calculate output:
  int ret_val;
  if (sigma_half < 0.5) {
    // check outside image
    if(x < 0) return -1;
    if(x > (w-2)) return -1;
    if(y < 0) return -1;
    if(y > (h-2)) return -1;

    // Interpolation multipliers:
    const int r_x = (xf - x) * 1024;
    const int r_y = (yf - y) * 1024;
    const int r_x_1 = (1024 - r_x);
    const int r_y_1 = (1024 - r_y);
    const uint8_t* ptr = im + x
        + y * imagecols;
    // Just interpolate:
    ret_val = (r_x_1 * r_y_1 * (int)(*ptr));
    ptr++;
    ret_val += (r_x * r_y_1 * (int)(*ptr));
    ptr += imagecols;
    ret_val += (r_x * r_y * (int)(*ptr));
    ptr--;
    ret_val += (r_x_1 * r_y * (int)(*ptr));
    return (ret_val) / 1024;
  }

  // This is the standard case (simple, not speed optimized yet):
  if(x_1 < 0.0f) return -1;
  if(x1 > (float)(w-1)) return -1;
  if(y_1 < 0.0f) return -1;
  if(y1 > (float)(h-1)) return -1;

  // Scaling:
  const int scaling = 4194304.0 / area;
  const int scaling2 = (float)(scaling) * area / 1024.0;

  // The integral image is larger:
  const int integralcols = imagecols + 1;

  const int x_left = (int)(x_1 + 0.5);
  const int y_top = (int)(y_1 + 0.5);
  const int x_right = (int)(x1 + 0.5);
  const int y_bottom = (int)(y1 + 0.5);

  // Overlap area - multiplication factors:
  const float r_x_1 = (float)(x_left) - x_1 + 0.5;
  const float r_y_1 = (float)(y_top) - y_1 + 0.5;
  const float r_x1 = x1 - (float)(x_right) + 0.5;
  const float r_y1 = y1 - (float)(y_bottom) + 0.5;
  const int dx = x_right - x_left - 1;
  const int dy = y_bottom - y_top - 1;
  const int A = (r_x_1 * r_y_1) * scaling;
  const int B = (r_x1 * r_y_1) * scaling;
  const int C = (r_x1 * r_y1) * scaling;
  const int D = (r_x_1 * r_y1) * scaling;
  const int r_x_1_i = r_x_1 * scaling;
  const int r_y_1_i = r_y_1 * scaling;
  const int r_x1_i = r_x1 * scaling;
  const int r_y1_i = r_y1 * scaling;

  if (dx + dy > 2) {
    // Now the calculation:
    const uint8_t* ptr = im + x_left
        + imagecols * y_top;
    // First the corners:
    ret_val = A * (int)(*ptr);
    ptr += dx + 1;
    ret_val += B * (int)(*ptr);
    ptr += dy * imagecols + 1;
    ret_val += C * (int)(*ptr);
    ptr -= dx + 1;
    ret_val += D * (int)(*ptr);

    // Next the edges:
    const int* ptr_integral = integral + x_left + integralcols * y_top + 1;
    // Find a simple path through the different surface corners.
    const int tmp1 = (*ptr_integral);
    ptr_integral += dx;
    const int tmp2 = (*ptr_integral);
    ptr_integral += integralcols;
    const int tmp3 = (*ptr_integral);
    ptr_integral++;
    const int tmp4 = (*ptr_integral);
    ptr_integral += dy * integralcols;
    const int tmp5 = (*ptr_integral);
    ptr_integral--;
    const int tmp6 = (*ptr_integral);
    ptr_integral += integralcols;
    const int tmp7 = (*ptr_integral);
    ptr_integral -= dx;
    const int tmp8 = (*ptr_integral);
    ptr_integral -= integralcols;
    const int tmp9 = (*ptr_integral);
    ptr_integral--;
    const int tmp10 = (*ptr_integral);
    ptr_integral -= dy * integralcols;
    const int tmp11 = (*ptr_integral);
    ptr_integral++;
    const int tmp12 = (*ptr_integral);

    // Assign the weighted surface integrals:
    const int upper = (tmp3 - tmp2 + tmp1 - tmp12) * r_y_1_i;
    const int middle = (tmp6 - tmp3 + tmp12 - tmp9) * scaling;
    const int left = (tmp9 - tmp12 + tmp11 - tmp10) * r_x_1_i;
    const int right = (tmp5 - tmp4 + tmp3 - tmp6) * r_x1_i;
    const int bottom = (tmp7 - tmp6 + tmp9 - tmp8) * r_y1_i;

    return (int)(
        (ret_val + upper + middle + left + right + bottom) / scaling2);
  }

  // Now the calculation:
  const uint8_t* ptr = im + x_left
      + imagecols * y_top;
  // First row:
  ret_val = A * (int)(*ptr);
  ptr++;
  const uint8_t* end1 = ptr + dx;
  for (; ptr < end1; ptr++) {
    ret_val += r_y_1_i * (int)(*ptr);
  }
  ret_val += B * (int)(*ptr);
  // Middle ones:
  ptr += imagecols - dx - 1;
  const uint8_t* end_j = ptr + dy * imagecols;
  for (; ptr < end_j; ptr += imagecols - dx - 1) {
    ret_val += r_x_1_i * (int)(*ptr);
    ptr++;
    const uint8_t* end2 = ptr + dx;
    for (; ptr < end2; ptr++) {
      ret_val += (int)(*ptr) * scaling;
    }
    ret_val += r_x1_i * (int)(*ptr);
  }
  // Last row:
  ret_val += D * (int)(*ptr);
  ptr++;
  const uint8_t* end3 = ptr + dx;
  for (; ptr < end3; ptr++) {
    ret_val += r_y1_i * (int)(*ptr);
  }
  ret_val += C * (int)(*ptr);

  return (int)((ret_val) / scaling2);
}
/* Camera helper uses OpenCV scalar evaluation order. */
extern int ok_brisk_camera_warp(float x,float y,int w,const float*rays,const float*jacs,float focal,const float dir[3],float warp[6]);
int ok_brisk_describe(ok_brisk_context*c,const uint8_t*im,int w,int h,const float*rays,const float*jacs,float focal,const float dir[3],ok_brisk_keypoint*k,size_t*count,uint8_t**desc,const ok_brisk_trace*t){
 if(!desc||!count)return -1;
 *desc=NULL;
 if(!c||!im||w<20||h<20||w>65535||h>65535||(size_t)w*h>INT_MAX/255||(*count&&!k)||(!rays!=!jacs)||(rays&&(!dir||!isfinite(focal)||focal<=0)))return -1;
 size_t kept=0;int border=c->border;
 for(size_t i=0;i<*count;i++){
  if(!isfinite(k[i].x)||!isfinite(k[i].y)||!isfinite(k[i].angle)||k[i].angle < -360.f||k[i].angle>360.f)return -1;
  if(k[i].x<border||k[i].y<border||k[i].x>=w-border||k[i].y>=h-border)continue;
  k[kept++]=k[i];
 }
 *count=kept;emit(t,"filtered",k,kept*sizeof(*k));
 int *ii=calloc((size_t)(w+1)*(h+1),sizeof(int));uint8_t*d=calloc(kept?kept:1,48);
 if(!ii||!d){free(ii);free(d);return -1;}
 for(int y=0;y<h;y++){int row=0;for(int x=0;x<w;x++){row+=im[(size_t)y*w+x];ii[(size_t)(y+1)*(w+1)+x+1]=row+ii[(size_t)y*(w+1)+x+1];}}
 emit(t,"integral",ii,(size_t)(w+1)*(h+1)*4);
 for(size_t a=0;a<kept;a++){
  float warp[6]={0,0,0,0,1,0};float*wp=NULL;int directional=0;
  if(rays&&ok_brisk_camera_warp(k[a].x,k[a].y,w,rays,jacs,focal,dir,warp)){wp=warp;directional=(int)warp[5];if(directional)k[a].angle=atan2(warp[3],warp[1])/PI*180.0;}
  emit(t,"warp",warp,sizeof(warp));int theta,values[66];
  if(k[a].angle==-1){
   for(unsigned i=0;i<66;i++)values[i]=intensity(c,im,w,h,ii,k[a].x,k[a].y,0,i,NULL,1);
   emit(t,"orientation_values",values,sizeof(values));int32_t d0=0,d1=0;
   for(unsigned i=0;i<856;i++){long_pair p=c->pairs[i];int delta=values[p.i]-values[p.j];d0+=delta*p.dx/1024;d1+=delta*p.dy/1024;}
   k[a].angle=atan2((float)d1,(float)d0)/PI*180.0;theta=(int)((1024*k[a].angle)/360.0+.5);
  }else theta=(int)(1024*(k[a].angle/360.0)+.5);
  if(theta<0)theta+=1024;
  if(theta>=1024)theta-=1024;
  for(unsigned i=0;i<66;i++)values[i]=intensity(c,im,w,h,ii,k[a].x,k[a].y,directional?0:theta,i,wp,warp[4]);
  emit(t,"values",values,sizeof(values));for(unsigned i=0;i<384;i++)if(values[short_pairs[i][0]]>values[short_pairs[i][1]])d[a*48+i/8]|=(uint8_t)(1u<<(i%8));
 }
 free(ii);emit(t,"described",k,kept*sizeof(*k));emit(t,"descriptors",d,kept*48);*desc=d;return 0;
}
