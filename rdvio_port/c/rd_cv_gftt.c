/* SPDX-License-Identifier: Apache-2.0 AND BSD-3-Clause
 * Modified C99 adaptation of OpenCV 4.6.0 corner.cpp, corner.avx.cpp,
 * deriv.cpp, filter.simd.hpp, box_filter.simd.hpp and featureselect.cpp.
 * Copyright (C) 2000-2008 Intel Corporation; (C) 2009 Willow Garage Inc.;
 * (C) 2014-2015 Itseez Inc. Full notices: ../reference_cv/LICENSE-OpenCV-source.
 */
#include "rd_cv.h"
#include <math.h>
#include <float.h>
#include <string.h>
static int reflect(int x,int n) { if(n==1)return 0; return x<0?-x:x>=n?2*n-x-2:x; }
static void emit(const rd_cv_trace *t,const char *s,const void *p,size_t n) { if(t&&t->emit)t->emit(t->user,s,p,n); }
int rd_cv_corner_response(const uint8_t *src,int w,int h,int harris,float *dst,const rd_cv_trace *t) {
    if(!src||!dst||w<1||h<1||w>16384||h>16384)return 0;
    size_t n=(size_t)w*h;
    float *mem=malloc(n*7*sizeof(float));
    double *rows=malloc(n*3*sizeof(double));
    if(!mem||!rows) { free(mem); free(rows); return 0; }
    float *rx=mem,*ry=rx+n,*dx=ry+n,*dy=dx+n,*cov=dy+n;
    float s=(float)(1.0/(4*3*255)),s2=(float)(2.0/(4*3*255));
    for(int y=0;y<h;y++)for(int x=0;x<w;x++) {
        int a=src[(size_t)y*w+reflect(x-1,w)],b=src[(size_t)y*w+x],c=src[(size_t)y*w+reflect(x+1,w)];
        rx[(size_t)y*w+x]=(float)(c-a);
        ry[(size_t)y*w+x]=x<w/32*32 ? fmaf((float)c,s,fmaf((float)b,s2,(float)a*s))
            : ((float)a*s+(float)b*s2)+(float)c*s;
    }
    for(int y=0;y<h;y++)for(int x=0;x<w;x++) {
        size_t a=(size_t)reflect(y-1,h)*w+x,b=(size_t)y*w+x,c=(size_t)reflect(y+1,h)*w+x;
        dx[b]=fmaf(rx[a]+rx[c],s,rx[b]*s2);
        dy[b]=ry[c]-ry[a]+0.f;
        cov[3*b]=dx[b]*dx[b]; cov[3*b+1]=dx[b]*dy[b]; cov[3*b+2]=dy[b]*dy[b];
    }
    emit(t,"sobel_dx",dx,n*4); emit(t,"sobel_dy",dy,n*4);
    /* boxFilter<float> uses DOUBLE row sums and a double vertical sliding
     * accumulator, then rounds once to float. */
    for(int y=0;y<h;y++)for(int x=0;x<w;x++)for(int c=0;c<3;c++) {
        size_t a=((size_t)y*w+reflect(x-1,w))*3+c,b=((size_t)y*w+x)*3+c,d=((size_t)y*w+reflect(x+1,w))*3+c;
        rows[b]=((double)cov[a]+cov[b])+cov[d];
    }
    for(int x=0;x<w;x++)for(int c=0;c<3;c++) {
        double sum=0;
        sum+=rows[((size_t)reflect(-1,h)*w+x)*3+c];
        sum+=rows[x*3+c];
        for(int y=0;y<h;y++) {
            double v=sum+rows[((size_t)reflect(y+1,h)*w+x)*3+c];
            cov[((size_t)y*w+x)*3+c]=(float)v;
            sum=v-rows[((size_t)reflect(y-1,h)*w+x)*3+c];
        }
    }
    emit(t,"covariance",cov,n*12);
    for(size_t i=0;i<n;i++) {
        float a=cov[3*i],b=cov[3*i+1],c=cov[3*i+2];
        if(harris) {
            float ac=a+c;
            if(i<n/8*8)dst[i]=(a*c-b*b)-.04f*(ac*ac); /* AVX */
            else if(i<n/4*4)dst[i]=(a*c-b*b)-(.04f*ac)*ac; /* SSE */
            else dst[i]=(float)((double)(a*c-b*b)-.04*ac*ac);
        } else {
            a*=.5f; c*=.5f;
            dst[i]=(a+c)-sqrtf((a-c)*(a-c)+b*b);
        }
    }
    free(rows); free(mem); return 1;
}
/* Independent median-partition introsort with libstdc++'s equal-key ordering.
 * No GPL source read/copied. The same sort model is used by the accepted
 * OKVIS leaves. GFTT itself breaks ties by decreasing image address; RD-VIO's
 * SECOND sort compares only response and consequently permutes equal scores. */
static int less(rd_cv_keypoint a,rd_cv_keypoint b,int tie) {
    if(a.response!=b.response)return a.response>b.response;
    return tie&&(a.pt.y>b.pt.y||(a.pt.y==b.pt.y&&a.pt.x>b.pt.x));
}
static void swap(rd_cv_keypoint *a,rd_cv_keypoint *b) { rd_cv_keypoint t=*a; *a=*b; *b=t; }
static void sift(rd_cv_keypoint *a,size_t hole,size_t n,rd_cv_keypoint v,int tie) {
    size_t top=hole,child=hole;
    while(child<(n-1)/2) {
        child=2*(child+1);
        if(less(a[child],a[child-1],tie))child--;
        a[hole]=a[child]; hole=child;
    }
    if(!(n&1)&&child==(n-2)/2) { child=2*(child+1); a[hole]=a[child-1]; hole=child-1; }
    while(hole>top) { size_t parent=(hole-1)/2; if(!less(a[parent],v,tie))break; a[hole]=a[parent]; hole=parent; }
    a[hole]=v;
}
static void quick(rd_cv_keypoint *a,size_t n,unsigned depth,int tie) {
    while(n>16) {
        if(!depth) {
            for(size_t k=n/2;k>0;) { --k; sift(a,k,n,a[k],tie); }
            for(size_t k=n;k>1;) { --k; rd_cv_keypoint v=a[k]; a[k]=a[0]; sift(a,0,k,v,tie); }
            return;
        }
        depth--;
        size_t x=1,y=n/2,z=n-1,m;
        if(less(a[x],a[y],tie))m=less(a[y],a[z],tie)?y:less(a[x],a[z],tie)?z:x;
        else m=less(a[x],a[z],tie)?x:less(a[y],a[z],tie)?z:y;
        swap(a,a+m); size_t i=1,j=n;
        for(;;) {
            while(less(a[i],a[0],tie))i++;
            do { j--; } while(less(a[0],a[j],tie));
            if(i>=j)break;
            swap(a+i,a+j); i++;
        }
        quick(a+i,n-i,depth,tie); n=i;
    }
}
static void sort(rd_cv_keypoint *a,size_t n,int tie) {
    unsigned depth=0; for(size_t v=n;v>1;v>>=1)depth++;
    quick(a,n,depth*2,tie);
    for(size_t i=1;i<n;i++) { rd_cv_keypoint v=a[i]; size_t j=i;
        while(j&&less(v,a[j-1],tie)) { a[j]=a[j-1]; j--; } a[j]=v; }
}
void rd_cv_sort_keypoints(rd_cv_keypoint *p,size_t n) { if(p)sort(p,n,0); }
int rd_cv_gftt(const uint8_t *src,int w,int h,int max_points,int harris,rd_cv_keypoint **out,size_t *count) {
    if(!out||!count||!src||w<1||h<1||w>16384||h>16384||max_points<0)return 0;
    *out=NULL; *count=0;
    size_t n=(size_t)w*h;
    float *r=malloc(n*sizeof(float));
    rd_cv_keypoint *a=malloc(n*sizeof(*a));
    if(!r||!a||!rd_cv_corner_response(src,w,h,harris,r,NULL)) { free(r); free(a); return 0; }
    float max=-FLT_MAX;
    for(size_t i=0;i<n;i++)if(r[i]>max)max=r[i];
    float threshold=(float)((double)max*.001);
    for(size_t i=0;i<n;i++)if(!(r[i]>threshold))r[i]=0;
    size_t nc=0;
    for(int y=1;y<h-1;y++)for(int x=1;x<w-1;x++) {
        float v=r[(size_t)y*w+x]; if(v==0)continue;
        int good=1;
        for(int j=-1;j<=1;j++)for(int i=-1;i<=1;i++)if(r[(size_t)(y+j)*w+x+i]>v)good=0;
        if(good) { rd_cv_keypoint k={{(float)x,(float)y},3.f,-1.f,v,0,-1}; a[nc++]=k; }
    }
    sort(a,nc,1);
    int gw=(w+19)/20,gh=(h+19)/20;
    int *head=malloc((size_t)gw*gh*sizeof(int)),*link=malloc((nc+1)*sizeof(int));
    if(!head||!link) { free(head);free(link);free(r);free(a);return 0; }
    for(int i=0;i<gw*gh;i++)head[i]=-1;
    size_t keep=0;
    for(size_t i=0;i<nc;i++) {
        int x=(int)a[i].pt.x,y=(int)a[i].pt.y,cx=x/20,cy=y/20,good=1;
        for(int yy=cy-1;yy<=cy+1;yy++)for(int xx=cx-1;xx<=cx+1;xx++) {
            if(xx<0||yy<0||xx>=gw||yy>=gh)continue;
            for(int j=head[yy*gw+xx];j>=0;j=link[j]) {
                float dx=x-a[j].pt.x,dy=y-a[j].pt.y;
                if(dx*dx+dy*dy<400.0)good=0;
            }
        }
        if(good) {
            a[keep]=a[i]; link[keep]=head[cy*gw+cx]; head[cy*gw+cx]=(int)keep; keep++;
            if(max_points>0&&keep==(size_t)max_points)break;
        }
    }
    free(head);free(link);free(r);
    *out=a; *count=keep; return 1;
}
