/* SPDX-License-Identifier: Apache-2.0 AND BSD-3-Clause
 * Modified scalar C99 model of OpenCV 4.6.0 video/src/lkpyramid.cpp's
 * CV_SIMD128 && !CV_NEON path (SSE baseline, not a runtime AVX2 kernel).
 * Copyright (C) 2000 Intel Corporation. Full retained upstream notices in
 * ../reference_cv/LICENSE-OpenCV-source.
 */
#include "rd_cv.h"
#include <math.h>
#include <float.h>
#include <string.h>
static int descale(int v,int n) {
    v+=1<<(n-1);
    return v>=0 ? v/(1<<n) : -1-((-1-v)/(1<<n));
}
static void weights(float a,float b,int *w) {
    w[0]=(int)lrintf((1.f-a)*(1.f-b)*16384);
    w[1]=(int)lrintf(a*(1.f-b)*16384);
    w[2]=(int)lrintf((1.f-a)*b*16384);
    w[3]=16384-w[0]-w[1]-w[2];
}
static int sample(const uint8_t *s,int stride,const int *w) {
    return descale(s[0]*w[0]+s[1]*w[1]+s[stride]*w[2]+s[stride+1]*w[3],9);
}
static float reduce(const float *q) { return (q[0]+q[2])+(q[1]+q[3]); }
static int valid_point(rd_cv_point p) {
    return isfinite(p.x)&&isfinite(p.y)&&fabsf(p.x)<=1e6f&&fabsf(p.y)<=1e6f;
}
int rd_cv_lk(const rd_cv_pyramid *prev,const rd_cv_pyramid *next,
             const rd_cv_point *pts,rd_cv_point *flow,uint8_t *status,float *err,size_t n) {
    if(!prev||!next||prev->count<1||prev->count>4||next->count<1||next->count>4)return 0;
    if(n&&(!pts||!flow||!status))return 0;
    int levels=prev->count<next->count?prev->count:next->count;
    for(int l=0;l<levels;l++) {
        const rd_cv_level *a=&prev->level[l],*b=&next->level[l];
        if(a->width!=b->width||a->height!=b->height||!a->image||!a->deriv||!b->image)return 0;
    }
    for(size_t p=0;p<n;p++)if(!valid_point(pts[p])||!valid_point(flow[p]))return 0;
    if(n)memset(status,1,n);
    for(int l=levels-1;l>=0;l--) {
        const rd_cv_level *I=&prev->level[l],*J=&next->level[l];
        const uint8_t *ir=I->image+21*I->step+21,*jr=J->image+21*J->step+21;
        const int16_t *dr=I->deriv+2*(21*I->step+21);
        for(size_t p=0;p<n;p++) {
            float scale=1.f/(1<<l);
            rd_cv_point pp={pts[p].x*scale-10.f,pts[p].y*scale-10.f};
            rd_cv_point np={flow[p].x*(l==levels-1?scale:2.f),flow[p].y*(l==levels-1?scale:2.f)};
            flow[p]=np;
            int ix=(int)floorf(pp.x),iy=(int)floorf(pp.y),w[4];
            if(ix < -21||ix>=I->width||iy < -21||iy>=I->height) {
                if(l==0) { status[p]=0; if(err)err[p]=0; } continue;
            }
            weights(pp.x-ix,pp.y-iy,w);
            int16_t patch[441],gx[441],gy[441];
            float sa=0,sb=0,sc=0,qa[4]={0},qb[4]={0},qc[4]={0};
            for(int y=0;y<21;y++)for(int x=0;x<21;x++) {
                int off=(iy+y)*I->step+ix+x, t=y*21+x;
                patch[t]=(int16_t)sample(ir+off,I->step,w);
                const int16_t *d=dr+2*off;
                int dx=descale(d[0]*w[0]+d[2]*w[1]+d[2*I->step]*w[2]+d[2*I->step+2]*w[3],14);
                int dy=descale(d[1]*w[0]+d[3]*w[1]+d[2*I->step+1]*w[2]+d[2*I->step+3]*w[3],14);
                gx[t]=(int16_t)dx; gy[t]=(int16_t)dy;
                /* 16 SIMD pixels, four lanes updated four times per row;
                 * the remaining five pixels have separate scalar sums. */
                if(x<16) { int k=x%4; qa[k]+=(float)(dx*dx); qb[k]+=(float)(dx*dy); qc[k]+=(float)(dy*dy); }
                else { sa+=(float)(dx*dx); sb+=(float)(dx*dy); sc+=(float)(dy*dy); }
            }
            float A11=(sa+reduce(qa))*(1.f/1048576);
            float A12=(sb+reduce(qb))*(1.f/1048576);
            float A22=(sc+reduce(qc))*(1.f/1048576);
            float D=A11*A22-A12*A12;
            float mineig=(A22+A11-sqrtf((A11-A22)*(A11-A22)+4.f*A12*A12))/882;
            if(mineig<1e-4f||D<FLT_EPSILON) { if(l==0)status[p]=0; continue; }
            D=1.f/D;
            np.x-=10.f; np.y-=10.f;
            rd_cv_point pd={0,0};
            for(int j=0;j<30;j++) {
                int nx=(int)floorf(np.x),ny=(int)floorf(np.y);
                if(nx < -21||nx>=J->width||ny < -21||ny>=J->height) { if(l==0)status[p]=0; break; }
                weights(np.x-nx,np.y-ny,w);
                float s1=0,s2=0,q0[4]={0},q1[4]={0};
                for(int y=0;y<21;y++) {
                    int diff[21];
                    for(int x=0;x<21;x++)diff[x]=sample(jr+(ny+y)*J->step+nx+x,J->step,w)-patch[y*21+x];
                    /* SSE dotprod sums integer products at x and x+4 BEFORE
                     * conversion; two accumulators hold [bX0,bY0,bX1,bY1]
                     * and [bX2,bY2,bX3,bY3]. */
                    for(int x=0;x<16;x+=8)for(int k=0;k<4;k++) {
                        int t=y*21+x+k;
                        float *q=k<2?q0:q1; int c=(k%2)*2;
                        q[c]+=(float)(diff[x+k]*gx[t]+diff[x+k+4]*gx[t+4]);
                        q[c+1]+=(float)(diff[x+k]*gy[t]+diff[x+k+4]*gy[t+4]);
                    }
                    for(int x=16;x<21;x++) { int t=y*21+x;
                        s1+=(float)(diff[x]*gx[t]); s2+=(float)(diff[x]*gy[t]); }
                }
                float b1=(s1+((q0[0]+q1[0])+(q0[2]+q1[2])))*(1.f/1048576);
                float b2=(s2+((q0[1]+q1[1])+(q0[3]+q1[3])))*(1.f/1048576);
                rd_cv_point delta={(A12*b2-A22*b1)*D,(A12*b1-A11*b2)*D};
                np.x+=delta.x; np.y+=delta.y;
                flow[p].x=np.x+10.f; flow[p].y=np.y+10.f;
                if((double)delta.x*delta.x+(double)delta.y*delta.y<=.01*.01)break;
                if(j>0&&fabsf(delta.x+pd.x)<.01&&fabsf(delta.y+pd.y)<.01) {
                    flow[p].x-=delta.x*.5f; flow[p].y-=delta.y*.5f; break;
                }
                pd=delta;
            }
            if(status[p]&&err&&l==0) {
                float px=flow[p].x-10.f,py=flow[p].y-10.f;
                int nx=(int)floorf(px),ny=(int)floorf(py);
                if(nx < -21||nx>=J->width||ny < -21||ny>=J->height) { status[p]=0; continue; }
                weights(px-nx,py-ny,w);
                float e=0;
                for(int y=0;y<21;y++)for(int x=0;x<21;x++)
                    e+=fabsf((float)(sample(jr+(ny+y)*J->step+nx+x,J->step,w)-patch[y*21+x]));
                err[p]=e/14112;
            }
        }
    }
    return 1;
}
