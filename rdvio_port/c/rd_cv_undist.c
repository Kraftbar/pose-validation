/* SPDX-License-Identifier: Apache-2.0 AND BSD-3-Clause
 * C99 adaptation of OpenCV 4.6 undistort.simd.hpp, undistort.dispatch.cpp,
 * fisheye.cpp, core/operations.hpp and imgwarp.cpp. Retained notices in
 * ../reference_cv/LICENSE-M7b-OpenCV. No SIMD intrinsics. */
#include "rd_cv_undist.h"
#include <stdlib.h>
#include <string.h>
#include <math.h>
static int dims(int w,int h) { return w>0&&h>0&&w<32767&&h<32767; }
int rd_cv_undistort_maps(const double K[4],const double D[4],int fish,int w,int h,float *m1,float *m2) {
    if(!K||!D||!m1||!m2||!dims(w,h)||K[0]==0||K[1]==0)return 0;
    for(int k=0;k<4;k++)if(!isfinite(K[k])||!isfinite(D[k]))return 0;
    /* Both Mat::inv for this 3x3 and Matx33::inv (even DECOMP_SVD!) use
     * cofactors. Do not simplify these entries to 1/fx or -cx/fx. */
    double inv=1.0/(K[0]*K[1]);
    double ix=K[1]*inv,iy=K[0]*inv,ox=(-K[2]*K[1])*inv,oy=(-K[0]*K[3])*inv;
    double iw=(K[0]*K[1])*inv;
    for(int y=0;y<h;y++) {
        /* The installed AVX2 object also contracts scalar row setup/tails. */
        double xx=ox,yy=fish?y*iy+oy:fma((double)y,iy,oy),ww=iw;
        int x=0;
        if(!fish)for(;x<=w-8;x+=8,xx+=8*ix)for(int lane=0;lane<8;lane++) {
            double a=(xx+ix*lane)*(1.0/ww),b=(yy+0.0)*(1.0/ww);
            double aa=a*a,bb=b*b,r2=aa+bb;
            double kr=fma(fma(D[1],r2,D[0]),r2,1.0);
            double xy=(a*b)*2.0;
            double u=fma(D[2],xy,fma(fma(2.0,aa,r2),D[3],a*kr));
            double v=fma(D[3],xy,fma(fma(2.0,bb,r2),D[2],b*kr));
            /* Zero tilt/prism additions also canonicalize signed zeros. */
            u=fma(0.0,r2,fma(0.0,r2*r2,u));
            v=fma(0.0,r2,fma(0.0,r2*r2,v));
            double tu=fma(1.0,u,fma(0.0,v,0.0));
            double tv=fma(0.0,u,fma(1.0,v,0.0));
            m1[(size_t)y*w+x+lane]=(float)fma(K[0],tu,K[2]);
            m2[(size_t)y*w+x+lane]=(float)fma(K[1],tv,K[3]);
        }
        for(;x<w;x++,xx+=ix) {
            double a=fish?xx/ww:xx*(1.0/ww),b=fish?yy/ww:yy*(1.0/ww),u,v;
            if(fish) {
                double r=sqrt(a*a+b*b),th=atan(r),t2=th*th,t4=t2*t2,t6=t4*t2,t8=t4*t4;
                double td=th*(1+D[0]*t2+D[1]*t4+D[2]*t6+D[3]*t8);
                double scale=r==0?1.0:td/r;
                u=K[0]*a*scale+K[2]; v=K[1]*b*scale+K[3];
            } else {
                double aa=a*a,bb=b*b,r2=aa+bb,xy=2*a*b;
                double kr=fma(fma(D[1],r2,D[0]),r2,1.0);
                double xd=fma(D[3],fma(2.0,aa,r2),fma(a,kr,D[2]*xy));
                double yd=fma(D[3],xy,fma(b,kr,D[2]*fma(2.0,bb,r2)));
                xd=fma(0.0*r2,r2,fma(0.0,r2,xd));
                yd=fma(0.0*r2,r2,fma(0.0,r2,yd));
                u=fma(K[0],fma(0.0,yd,xd)+0.0,K[2]);
                v=fma(K[1],fma(0.0,xd,yd)+0.0,K[3]);
            }
            m1[(size_t)y*w+x]=(float)u;m2[(size_t)y*w+x]=(float)v;
        }
    }
    return 1;
}
static int quantize(float x) {
    float f=x*32.f;
    if(!isfinite(f)||f>=2147483648.f||f< -2147483648.f)return (-2147483647-1);
    return (int)lrintf(f);
}
static int floor32(int x) { return x>=0?x/32:-1-((-1-x)/32); }
int rd_cv_remap_linear(const uint8_t *src,int w,int h,const float *m1,const float *m2,uint8_t *dst) {
    if(!src||!m1||!m2||!dst||!dims(w,h))return 0;
    uint8_t *copy=NULL;
    if(src==dst) {copy=malloc((size_t)w*h);if(!copy)return 0;memcpy(copy,src,(size_t)w*h);src=copy;}
    for(size_t i=0;i<(size_t)w*h;i++) {
        int u=quantize(m1[i]),v=quantize(m2[i]),x=floor32(u),y=floor32(v);
        int a=(int)((uint32_t)u&31),b=(int)((uint32_t)v&31);
        if(x< -32768)x=-32768;
        if(x>32767)x=32767;
        if(y< -32768)y=-32768;
        if(y>32767)y=32767;
        int wt[4]={32*(32-a)*(32-b),32*a*(32-b),32*(32-a)*b,32*a*b};
        if(!a&&!b) {wt[0]=32767;wt[3]=1;}
        int sum=0;
        for(int j=0;j<2;j++)for(int k=0;k<2;k++)
            if(x+k>=0&&x+k<w&&y+j>=0&&y+j<h)sum+=src[(size_t)(y+j)*w+x+k]*wt[j*2+k];
        dst[i]=(uint8_t)((sum+16384)/32768);
    }
    free(copy);return 1;
}
