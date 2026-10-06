/* SPDX-License-Identifier: Apache-2.0 AND BSD-3-Clause
 * Modified C99 adaptation of OpenCV 4.6.0 clahe.cpp, pyramids.cpp,
 * lkpyramid.cpp and core/types.hpp. Copyright (C) 2013 NVIDIA Corporation;
 * Copyright (C) 2014 Itseez Inc.; Copyright (C) 2000 Intel Corporation.
 * See ../reference_cv/LICENSE-OpenCV-source for full retained notices.
 */
#include "rd_cv.h"
#include <string.h>
#include <math.h>
#include <float.h>

static int reflect(int x, int n) {
    if (n == 1) return 0;
    while (x < 0 || x >= n) x = x < 0 ? -x : 2*n-x-2;
    return x;
}
static int dimensions(int w, int h) { return w > 0 && h > 0 && w <= 16384 && h <= 16384; }
static uint8_t byte_round(float f) {
    int v = (int)lrintf(f);
    return (uint8_t)(v < 0 ? 0 : v > 255 ? 255 : v);
}
int rd_cv_clahe(const uint8_t *src, int w, int h, uint8_t *dst) {
    uint8_t lut[64][256];
    if (!src || !dst || !dimensions(w,h)) return 0;
    /* OpenCV adds a WHOLE tile-count border on a divisible axis when the
     * other axis is not divisible. Do not replace this with ceil(w/8). */
    int tw=w/8, th=h/8;
    if (w%8 || h%8) { tw=(w+8-w%8)/8; th=(h+8-h%8)/8; }
    int area=tw*th, clip=(int)(6.0*area/256);
    if (clip<1) clip=1;
    float scale=255.f/area;
    for (int ty=0;ty<8;ty++) for (int tx=0;tx<8;tx++) {
        int hist[256]={0}, clipped=0;
        for (int y=0;y<th;y++) for (int x=0;x<tw;x++)
            hist[src[(size_t)reflect(ty*th+y,h)*w+reflect(tx*tw+x,w)]]++;
        for (int i=0;i<256;i++) if(hist[i]>clip) { clipped+=hist[i]-clip; hist[i]=clip; }
        int batch=clipped/256, residual=clipped-batch*256;
        for(int i=0;i<256;i++) hist[i]+=batch;
        if(residual) { int step=256/residual; if(step<1)step=1;
            for(int i=0;i<256&&residual;i+=step,residual--) hist[i]++; }
        int sum=0;
        for(int i=0;i<256;i++) { sum+=hist[i]; lut[ty*8+tx][i]=byte_round(sum*scale); }
    }
    float iw=1.f/tw, ih=1.f/th;
    for(int y=0;y<h;y++) {
        float yf=y*ih-.5f; int y1=(int)floorf(yf), y2=y1+1;
        float ya=yf-y1, ya1=1.f-ya;
        if(y1<0)y1=0;
        if(y2>7)y2=7;
        for(int x=0;x<w;x++) {
            float xf=x*iw-.5f; int x1=(int)floorf(xf),x2=x1+1;
            float xa=xf-x1, xa1=1.f-xa;
            if(x1<0)x1=0;
            if(x2>7)x2=7;
            int v=src[(size_t)y*w+x];
            float r=(lut[y1*8+x1][v]*xa1+lut[y1*8+x2][v]*xa)*ya1
                   +(lut[y2*8+x1][v]*xa1+lut[y2*8+x2][v]*xa)*ya;
            dst[(size_t)y*w+x]=byte_round(r);
        }
    }
    return 1;
}
void rd_cv_free_pyramid(rd_cv_pyramid *p) {
    if(!p)return;
    for(int l=0;l<4;l++) { free(p->level[l].image); free(p->level[l].deriv); }
    memset(p,0,sizeof(*p));
}
int rd_cv_build_pyramid(const uint8_t *src,int w,int h,rd_cv_pyramid *p) {
    if(!src||!p||!dimensions(w,h)||p->count)return 0;
    static const int k[5]={1,4,6,4,1};
    for(int l=0;l<4;l++) {
        rd_cv_level *q=&p->level[l];
        q->width=w; q->height=h; q->step=w+42;
        size_t n=(size_t)(w+42)*(h+42);
        q->image=malloc(n); q->deriv=calloc(n,2*sizeof(int16_t));
        if(!q->image||!q->deriv) { rd_cv_free_pyramid(p); return 0; }
        uint8_t *roi=q->image+21*q->step+21;
        for(int y=0;y<h;y++) for(int x=0;x<w;x++) {
            int v;
            if(!l) v=src[(size_t)y*w+x];
            else {
                rd_cv_level *a=&p->level[l-1];
                const uint8_t *r=a->image+21*a->step+21;
                int s=0;
                for(int j=-2;j<=2;j++) for(int i=-2;i<=2;i++)
                    s+=k[j+2]*k[i+2]*r[reflect(2*y+j,a->height)*a->step+reflect(2*x+i,a->width)];
                v=(s+128)/256;
            }
            roi[y*q->step+x]=(uint8_t)v;
        }
        for(int y=-21;y<h+21;y++) for(int x=-21;x<w+21;x++)
            if(y<0||y>=h||x<0||x>=w) roi[y*q->step+x]=roi[reflect(y,h)*q->step+reflect(x,w)];
        for(int y=0;y<h;y++) for(int x=0;x<w;x++) {
            int xm=reflect(x-1,w),xp=reflect(x+1,w),ym=reflect(y-1,h),yp=reflect(y+1,h);
            int dx=3*(roi[ym*q->step+xp]-roi[ym*q->step+xm])
                  +10*(roi[y*q->step+xp]-roi[y*q->step+xm])
                  +3*(roi[yp*q->step+xp]-roi[yp*q->step+xm]);
            int dy=3*(roi[yp*q->step+xm]-roi[ym*q->step+xm])
                  +10*(roi[yp*q->step+x]-roi[ym*q->step+x])
                  +3*(roi[yp*q->step+xp]-roi[ym*q->step+xp]);
            size_t idx=(size_t)(y+21)*q->step+x+21;
            q->deriv[2*idx]=(int16_t)dx; q->deriv[2*idx+1]=(int16_t)dy;
        }
        p->count=l+1;
        w=(w+1)/2; h=(h+1)/2;
        if(w<=21||h<=21)break;
    }
    return 1;
}
double rd_cv_norm(rd_cv_point p) { return sqrt((double)p.x*p.x+(double)p.y*p.y); }
void rd_cv_track_status(const rd_cv_point *a,const rd_cv_point *b,const rd_cv_point *r,
                       uint8_t *s,const uint8_t *rs,size_t n,int w,int h) {
    for(size_t i=0;i<n;i++) {
        if(b[i].x<20||b[i].y<20||b[i].x>=w-20||b[i].y>=h-20)s[i]=0;
        rd_cv_point d={b[i].x-a[i].x,b[i].y-a[i].y};
        if(s[i]&&rd_cv_norm(d)>h/4)s[i]=0;
        d.x=a[i].x-r[i].x; d.y=a[i].y-r[i].y;
        if(s[i]&&(!rs[i]||rd_cv_norm(d)>.5))s[i]=0;
    }
}
