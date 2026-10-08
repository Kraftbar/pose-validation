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
#if defined(__SSE2__) && !defined(RD_CV_NO_SIMD)
#include <emmintrin.h>
#define RD_SSE2 1
#endif
static int reflect(int x,int n) { if(n==1)return 0; return x<0?-x:x>=n?2*n-x-2:x; }
static void emit(const rd_cv_trace *t,const char *s,const void *p,size_t n) { if(t&&t->emit)t->emit(t->user,s,p,n); }

/* Streaming evaluation of the same arithmetic as the earlier whole-image
 * passes: Sobel row pair -> derivative / covariance row -> double horizontal
 * sum (ring of 3 rows) -> double vertical sliding sum -> float -> response.
 * Every value is produced by exactly the same operations in the same order. */
typedef struct {
    const uint8_t *src; int w,h;
    float *rx,*ry;      /* full images, produced row by row on demand */
    int src_done;       /* source rows 0..src_done-1 are in rx/ry */
    double *R;          /* ring: [slot][c][x] */
    int rs_done;
    float *ca,*cb,*cc;  /* one covariance row, planar */
    float *ta,*tb,*tc;  /* output covariance row */
    float *dxo,*dyo,*covo; /* trace-only full outputs */
} corner_ctx;

/* The row kernels are plain C. On x86-64 GCC/Clang a second copy is compiled
 * for AVX2+FMA (fmaf becomes one vfmadd instruction instead of a libm call,
 * loops auto-vectorise; all operations stay IEEE-exact elementwise, no
 * reassociation) and chosen at run time; elsewhere only the portable copy exists. */
#if (defined(__GNUC__)||defined(__clang__)) && defined(__x86_64__) && !defined(RD_CV_NO_SIMD) && !defined(RD_CV_NO_AVX2)
#define RD_FMA_CLONE 1
#define RD_ALWAYS __attribute__((always_inline)) inline
#define RD_FMA_TARGET __attribute__((target("avx2,fma"),optimize("O3")))
#else
#define RD_ALWAYS
#endif
static RD_ALWAYS void src_px(corner_ctx *k,int y,int x) {
    const int w=k->w;
    const float s=(float)(1.0/(4*3*255)),s2=(float)(2.0/(4*3*255));
    const uint8_t *r=k->src+(size_t)y*w;
    int a=r[reflect(x-1,w)],b=r[x],c=r[reflect(x+1,w)];
    k->rx[(size_t)y*w+x]=(float)(c-a);
    k->ry[(size_t)y*w+x]=x<w/32*32 ? fmaf((float)c,s,fmaf((float)b,s2,(float)a*s))
        : ((float)a*s+(float)b*s2)+(float)c*s;
}
static RD_ALWAYS void src_row_body(corner_ctx *k,int y) {
    const int w=k->w;
    const float s=(float)(1.0/(4*3*255)),s2=(float)(2.0/(4*3*255));
    const uint8_t *r=k->src+(size_t)y*w;
    float *rx=k->rx+(size_t)y*w,*ry=k->ry+(size_t)y*w;
    const int lim=w/32*32;
    src_px(k,y,0);
    if(w>1) src_px(k,y,w-1);
    int hi=w-1; /* interior [1,hi) */
    int m=lim<hi?lim:hi; if(m<1)m=1;
    for(int x=1;x<m;x++) {
        float a=(float)r[x-1],b=(float)r[x],c=(float)r[x+1];
        rx[x]=c-a; /* exact: integers below 2^24 */
        ry[x]=fmaf(c,s,fmaf(b,s2,a*s));
    }
    for(int x=m;x<hi;x++) {
        float a=(float)r[x-1],b=(float)r[x],c=(float)r[x+1];
        rx[x]=c-a;
        ry[x]=((a*s+b*s2)+c*s);
    }
}
#ifdef RD_FMA_CLONE
static void cov_interior_avx2(const float *ca,const float *cb,const float *cc,double *oa,double *ob,double *oc,int w);
#endif
static RD_ALWAYS void cov_row_body(corner_ctx *k,int y,double *out,int wide) {
    const int w=k->w,h=k->h;
    const float s=(float)(1.0/(4*3*255)),s2=(float)(2.0/(4*3*255));
    const float *xa=k->rx+(size_t)reflect(y-1,h)*w,*xb=k->rx+(size_t)y*w,*xc=k->rx+(size_t)reflect(y+1,h)*w;
    const float *ya=k->ry+(size_t)reflect(y-1,h)*w,*yc=k->ry+(size_t)reflect(y+1,h)*w;
    float *ca=k->ca,*cb=k->cb,*cc=k->cc;
    for(int x=0;x<w;x++) {
        float dx=fmaf(xa[x]+xc[x],s,xb[x]*s2);
        float dy=yc[x]-ya[x]+0.f;
        ca[x]=dx*dx; cb[x]=dx*dy; cc[x]=dy*dy;
    }
    if(k->dxo) for(int x=0;x<w;x++) {
        float dx=fmaf(xa[x]+xc[x],s,xb[x]*s2);
        float dy=yc[x]-ya[x]+0.f;
        k->dxo[(size_t)y*w+x]=dx; k->dyo[(size_t)y*w+x]=dy;
    }
    double *oa=out,*ob=out+w,*oc=out+2*w;
    for(int x=0;x<w;x++) if(x==0||x==w-1) {
        int l=reflect(x-1,w),r=reflect(x+1,w);
        oa[x]=((double)ca[l]+ca[x])+ca[r];
        ob[x]=((double)cb[l]+cb[x])+cb[r];
        oc[x]=((double)cc[l]+cc[x])+cc[r];
    }
    int x=1;
#ifdef RD_FMA_CLONE
    if(wide) { cov_interior_avx2(ca,cb,cc,oa,ob,oc,w); x=w-1; }
#endif
    (void)wide;
#ifdef RD_SSE2
    /* two columns per step; float->double conversion is exact, adds are IEEE double */
#define LD2(p) _mm_cvtps_pd(_mm_castsi128_ps(_mm_loadl_epi64((const __m128i*)(p))))
    for(;x+1<w-1;x+=2) {
        _mm_storeu_pd(oa+x,_mm_add_pd(_mm_add_pd(LD2(ca+x-1),LD2(ca+x)),LD2(ca+x+1)));
        _mm_storeu_pd(ob+x,_mm_add_pd(_mm_add_pd(LD2(cb+x-1),LD2(cb+x)),LD2(cb+x+1)));
        _mm_storeu_pd(oc+x,_mm_add_pd(_mm_add_pd(LD2(cc+x-1),LD2(cc+x)),LD2(cc+x+1)));
    }
#undef LD2
#endif
    for(;x<w-1;x++) {
        oa[x]=((double)ca[x-1]+ca[x])+ca[x+1];
        ob[x]=((double)cb[x-1]+cb[x])+cb[x+1];
        oc[x]=((double)cc[x-1]+cc[x])+cc[x+1];
    }
}
/* vertical sliding double sums of one output row -> float covariance row */
static RD_ALWAYS void slide_row_body(int w,double *sa,double *sb,double *sc,const double *rna,const double *rnb,const double *rnc,
                                     const double *roa,const double *rob,const double *roc,float *fa,float *fb,float *fc) {
    int x=0;
#ifdef RD_SSE2
    for(;x+1<w;x+=2) {
        __m128d va=_mm_add_pd(_mm_loadu_pd(sa+x),_mm_loadu_pd(rna+x));
        __m128d vb=_mm_add_pd(_mm_loadu_pd(sb+x),_mm_loadu_pd(rnb+x));
        __m128d vc=_mm_add_pd(_mm_loadu_pd(sc+x),_mm_loadu_pd(rnc+x));
        _mm_storeu_pd(sa+x,_mm_sub_pd(va,_mm_loadu_pd(roa+x)));
        _mm_storeu_pd(sb+x,_mm_sub_pd(vb,_mm_loadu_pd(rob+x)));
        _mm_storeu_pd(sc+x,_mm_sub_pd(vc,_mm_loadu_pd(roc+x)));
        _mm_storel_epi64((__m128i*)(fa+x),_mm_castps_si128(_mm_cvtpd_ps(va)));
        _mm_storel_epi64((__m128i*)(fb+x),_mm_castps_si128(_mm_cvtpd_ps(vb)));
        _mm_storel_epi64((__m128i*)(fc+x),_mm_castps_si128(_mm_cvtpd_ps(vc)));
    }
#endif
    for(;x<w;x++) {
        double va=sa[x]+rna[x],vb=sb[x]+rnb[x],vc=sc[x]+rnc[x];
        sa[x]=va-roa[x]; sb[x]=vb-rob[x]; sc[x]=vc-roc[x];
        fa[x]=(float)va; fb[x]=(float)vb; fc[x]=(float)vc;
    }
}
static void slide_row_plain(int w,double *sa,double *sb,double *sc,const double *rna,const double *rnb,const double *rnc,
                            const double *roa,const double *rob,const double *roc,float *fa,float *fb,float *fc) {
    slide_row_body(w,sa,sb,sc,rna,rnb,rnc,roa,rob,roc,fa,fb,fc); }
static RD_ALWAYS void harris_row_body(const float *fa,const float *fb,const float *fc,float *dr,int w,int e1,int e2) {
    for(int x=0;x<e1;x++) { float a=fa[x],b=fb[x],c=fc[x],ac=a+c; dr[x]=(a*c-b*b)-.04f*(ac*ac); }
    for(int x=e1;x<e2;x++) { float a=fa[x],b=fb[x],c=fc[x],ac=a+c; dr[x]=(a*c-b*b)-(.04f*ac)*ac; }
    for(int x=e2>e1?e2:e1;x<w;x++) { float a=fa[x],b=fb[x],c=fc[x],ac=a+c; dr[x]=(float)((double)(a*c-b*b)-.04*ac*ac); }
}
static void harris_row_plain(const float *fa,const float *fb,const float *fc,float *dr,int w,int e1,int e2) { harris_row_body(fa,fb,fc,dr,w,e1,e2); }
static void src_row_plain(corner_ctx *k,int y) { src_row_body(k,y); }
static void cov_row_plain(corner_ctx *k,int y,double *out) { cov_row_body(k,y,out,0); }
#ifdef RD_FMA_CLONE
RD_FMA_TARGET static void src_row_fma(corner_ctx *k,int y) { src_row_body(k,y); }
RD_FMA_TARGET static void harris_row_fma(const float *fa,const float *fb,const float *fc,float *dr,int w,int e1,int e2) { harris_row_body(fa,fb,fc,dr,w,e1,e2); }
RD_FMA_TARGET static void cov_row_fma(corner_ctx *k,int y,double *out) { cov_row_body(k,y,out,1); }
#include <immintrin.h>
/* interior columns 1..w-2: ((double)c[x-1]+c[x])+c[x+1], four columns per step */
__attribute__((target("avx2"),noinline)) static void cov_interior_avx2(const float *ca,const float *cb,const float *cc,double *oa,double *ob,double *oc,int w) {
    int x=1;
#define LD4(p) _mm256_cvtps_pd(_mm_loadu_ps(p))
    for(;x+3<=w-2;x+=4) {
        _mm256_storeu_pd(oa+x,_mm256_add_pd(_mm256_add_pd(LD4(ca+x-1),LD4(ca+x)),LD4(ca+x+1)));
        _mm256_storeu_pd(ob+x,_mm256_add_pd(_mm256_add_pd(LD4(cb+x-1),LD4(cb+x)),LD4(cb+x+1)));
        _mm256_storeu_pd(oc+x,_mm256_add_pd(_mm256_add_pd(LD4(cc+x-1),LD4(cc+x)),LD4(cc+x+1)));
    }
#undef LD4
    for(;x<w-1;x++) {
        oa[x]=((double)ca[x-1]+ca[x])+ca[x+1];
        ob[x]=((double)cb[x-1]+cb[x])+cb[x+1];
        oc[x]=((double)cc[x-1]+cc[x])+cc[x+1];
    }
}
__attribute__((target("avx2"),noinline)) static void slide_row_avx2(int w,double *sa,double *sb,double *sc,const double *rna,const double *rnb,const double *rnc,
                                                                    const double *roa,const double *rob,const double *roc,float *fa,float *fb,float *fc) {
    int x=0;
    for(;x+4<=w;x+=4) {
        __m256d va=_mm256_add_pd(_mm256_loadu_pd(sa+x),_mm256_loadu_pd(rna+x));
        __m256d vb=_mm256_add_pd(_mm256_loadu_pd(sb+x),_mm256_loadu_pd(rnb+x));
        __m256d vc=_mm256_add_pd(_mm256_loadu_pd(sc+x),_mm256_loadu_pd(rnc+x));
        _mm256_storeu_pd(sa+x,_mm256_sub_pd(va,_mm256_loadu_pd(roa+x)));
        _mm256_storeu_pd(sb+x,_mm256_sub_pd(vb,_mm256_loadu_pd(rob+x)));
        _mm256_storeu_pd(sc+x,_mm256_sub_pd(vc,_mm256_loadu_pd(roc+x)));
        _mm_storeu_ps(fa+x,_mm256_cvtpd_ps(va));
        _mm_storeu_ps(fb+x,_mm256_cvtpd_ps(vb));
        _mm_storeu_ps(fc+x,_mm256_cvtpd_ps(vc));
    }
    for(;x<w;x++) {
        double va=sa[x]+rna[x],vb=sb[x]+rnb[x],vc=sc[x]+rnc[x];
        sa[x]=va-roa[x]; sb[x]=vb-rob[x]; sc[x]=vc-roc[x];
        fa[x]=(float)va; fb[x]=(float)vb; fc[x]=(float)vc;
    }
}
static int have_fma(void) {
    static int v=-1;
    if(v<0) { __builtin_cpu_init(); v=__builtin_cpu_supports("avx2")&&__builtin_cpu_supports("fma"); }
    return v;
}
static void slide_row(int w,double *sa,double *sb,double *sc,const double *rna,const double *rnb,const double *rnc,
                      const double *roa,const double *rob,const double *roc,float *fa,float *fb,float *fc) {
    if(have_fma())slide_row_avx2(w,sa,sb,sc,rna,rnb,rnc,roa,rob,roc,fa,fb,fc); else slide_row_plain(w,sa,sb,sc,rna,rnb,rnc,roa,rob,roc,fa,fb,fc); }
static void src_row(corner_ctx *k,int y) { if(have_fma())src_row_fma(k,y); else src_row_plain(k,y); }
static void harris_row(const float *fa,const float *fb,const float *fc,float *dr,int w,int e1,int e2) {
    if(have_fma())harris_row_fma(fa,fb,fc,dr,w,e1,e2); else harris_row_plain(fa,fb,fc,dr,w,e1,e2); }
static void cov_row(corner_ctx *k,int y,double *out) { if(have_fma())cov_row_fma(k,y,out); else cov_row_plain(k,y,out); }
#else
static void harris_row(const float *fa,const float *fb,const float *fc,float *dr,int w,int e1,int e2) { harris_row_plain(fa,fb,fc,dr,w,e1,e2); }
static void slide_row(int w,double *sa,double *sb,double *sc,const double *rna,const double *rnb,const double *rnc,
                      const double *roa,const double *rob,const double *roc,float *fa,float *fb,float *fc) {
    slide_row_plain(w,sa,sb,sc,rna,rnb,rnc,roa,rob,roc,fa,fb,fc); }
static void src_row(corner_ctx *k,int y) { src_row_plain(k,y); }
static void cov_row(corner_ctx *k,int y,double *out) { cov_row_plain(k,y,out); }
#endif
static const double *row_sum(corner_ctx *k,int r) {
    return k->R+(size_t)(r%3)*3*k->w;
}
static void need(corner_ctx *k,int r) {
    if(r>=k->h)r=k->h-1;
    int lastsrc=r+1; if(lastsrc>=k->h)lastsrc=k->h-1;
    while(k->src_done<=lastsrc) src_row(k,k->src_done++);
    while(k->rs_done<=r) { cov_row(k,k->rs_done,k->R+(size_t)(k->rs_done%3)*3*k->w); k->rs_done++; }
}
int rd_cv_corner_response(const uint8_t *src,int w,int h,int harris,float *dst,const rd_cv_trace *t) {
    if(!src||!dst||w<1||h<1||w>16384||h>16384)return 0;
    size_t n=(size_t)w*h;
    int tr=t&&t->emit;
    corner_ctx k; memset(&k,0,sizeof k);
    k.src=src; k.w=w; k.h=h;
    float *mem=malloc((n*2+6*(size_t)w+(tr?n*5:0))*sizeof(float));
    double *dm=malloc(((size_t)9*w+3*(size_t)w)*sizeof(double));
    if(!mem||!dm) { free(mem); free(dm); return 0; }
    k.rx=mem; k.ry=mem+n; k.ca=mem+2*n; k.cb=k.ca+w; k.cc=k.cb+w; k.ta=k.cc+w; k.tb=k.ta+w; k.tc=k.tb+w;
    if(tr) { k.dxo=k.tc+w; k.dyo=k.dxo+n; k.covo=k.dyo+n; }
    k.R=dm; double *sum=dm+(size_t)9*w; /* [c][x] */
    /* boxFilter<float> uses DOUBLE row sums and a double vertical sliding
     * accumulator, then rounds once to float. */
    need(&k,h>1?1:0);
    {
        const double *r0=row_sum(&k,0),*rm=row_sum(&k,reflect(-1,h));
        for(int i=0;i<3*w;i++) { double sm=0; sm+=rm[i]; sm+=r0[i]; sum[i]=sm; }
    }
    const size_t n8=n/8*8,n4=n/4*4;
    for(int y=0;y<h;y++) {
        need(&k,y+1);
        const double *rn=row_sum(&k,reflect(y+1,h)),*ro=row_sum(&k,reflect(y-1,h));
        const double *rna=rn,*rnb=rn+w,*rnc=rn+2*w,*roa=ro,*rob=ro+w,*roc=ro+2*w;
        float *dr=dst+(size_t)y*w;
        float *fa=k.ta,*fb=k.tb,*fc=k.tc;
        slide_row(w,sum,sum+w,sum+2*w,rna,rnb,rnc,roa,rob,roc,fa,fb,fc);
        if(tr) for(int x=0;x<w;x++) { size_t i=(size_t)y*w+x; k.covo[3*i]=fa[x]; k.covo[3*i+1]=fb[x]; k.covo[3*i+2]=fc[x]; }
        if(harris) {
            /* index-dependent evaluation: AVX lanes, then SSE lanes, then scalar tail */
            size_t base=(size_t)y*w;
            int e1=base>=n8?0:(n8-base<(size_t)w?(int)(n8-base):w);
            int e2=base>=n4?0:(n4-base<(size_t)w?(int)(n4-base):w);
            harris_row(fa,fb,fc,dr,w,e1,e2);
        } else for(int x=0;x<w;x++) {
            float a=fa[x]*.5f,b=fb[x],c=fc[x]*.5f;
            dr[x]=(a+c)-sqrtf((a-c)*(a-c)+b*b);
        }
    }
    if(tr) {
        emit(t,"sobel_dx",k.dxo,n*4); emit(t,"sobel_dy",k.dyo,n*4);
        emit(t,"covariance",k.covo,n*12);
    }
    free(dm); free(mem); return 1;
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
/* Scratch buffers kept between calls (single-threaded use, like the rest of
 * the leaf): a 1.4 MB response image and a candidate list are otherwise
 * allocated, page-faulted and freed for every frame. */
static float *g_resp; static size_t g_resp_cap;
static rd_cv_keypoint *g_cand; static size_t g_cand_cap;
int rd_cv_gftt(const uint8_t *src,int w,int h,int max_points,int harris,rd_cv_keypoint **out,size_t *count) {
    if(!out||!count||!src||w<1||h<1||w>16384||h>16384||max_points<0)return 0;
    *out=NULL; *count=0;
    size_t n=(size_t)w*h;
    if(n>g_resp_cap) { free(g_resp); g_resp=malloc(n*sizeof(float)); g_resp_cap=g_resp?n:0; }
    float *r=g_resp;
    if(!r||!rd_cv_corner_response(src,w,h,harris,r,NULL)) return 0;
    float max=-FLT_MAX;
    {
        size_t i=0;
#ifdef RD_SSE2
        __m128 m=_mm_set1_ps(-FLT_MAX);
        for(;i+4<=n;i+=4) m=_mm_max_ps(m,_mm_loadu_ps(r+i));
        float t[4]; _mm_storeu_ps(t,m);
        for(int k=0;k<4;k++) if(t[k]>max) max=t[k];
#endif
        for(;i<n;i++)if(r[i]>max)max=r[i];
    }
    float threshold=(float)((double)max*.001);
    {
        size_t i=0;
#ifdef RD_SSE2
        const __m128 th=_mm_set1_ps(threshold);
        for(;i+4<=n;i+=4) { __m128 v=_mm_loadu_ps(r+i); _mm_storeu_ps(r+i,_mm_and_ps(v,_mm_cmpgt_ps(v,th))); }
#endif
        for(;i<n;i++)if(!(r[i]>threshold))r[i]=0;
    }
    size_t nc=0;
    /* local maxima (ties allowed): v != 0 and no 8-neighbour greater than v, in scan order */
#define PUSH(X,Y,V) do { \
        if(nc==g_cand_cap) { \
            size_t nn=g_cand_cap?g_cand_cap*2:4096; rd_cv_keypoint *tmp=realloc(g_cand,nn*sizeof *tmp); \
            if(!tmp) { return 0; } \
            g_cand=tmp; g_cand_cap=nn; } \
        rd_cv_keypoint kk={{(float)(X),(float)(Y)},3.f,-1.f,(V),0,-1}; g_cand[nc++]=kk; } while(0)
    for(int y=1;y<h-1;y++) {
        const float *up=r+(size_t)(y-1)*w,*cu=r+(size_t)y*w,*dn=r+(size_t)(y+1)*w;
        int x=1;
#ifdef RD_SSE2
        const __m128 zero=_mm_setzero_ps();
        for(;x+3<=w-2;x+=4) {
            __m128 v=_mm_loadu_ps(cu+x);
            __m128 m=_mm_max_ps(_mm_max_ps(_mm_loadu_ps(up+x-1),_mm_loadu_ps(up+x)),_mm_loadu_ps(up+x+1));
            m=_mm_max_ps(m,_mm_max_ps(_mm_max_ps(_mm_loadu_ps(dn+x-1),_mm_loadu_ps(dn+x)),_mm_loadu_ps(dn+x+1)));
            m=_mm_max_ps(m,_mm_max_ps(_mm_loadu_ps(cu+x-1),_mm_loadu_ps(cu+x+1)));
            int mask=_mm_movemask_ps(_mm_andnot_ps(_mm_cmpgt_ps(m,v),_mm_cmpneq_ps(v,zero)));
            for(int k=0;k<4;k++) { if(mask&(1<<k)) { PUSH(x+k,y,cu[x+k]); } }
        }
#endif
        for(;x<w-1;x++) {
            float v=cu[x]; if(v==0)continue;
            int good=1;
            for(int i=-1;i<=1;i++) if(up[x+i]>v||cu[x+i]>v||dn[x+i]>v) good=0;
            if(good) PUSH(x,y,v);
        }
    }
#undef PUSH
    rd_cv_keypoint *a=g_cand;
    sort(a,nc,1);
    int gw=(w+19)/20,gh=(h+19)/20;
    int *head=malloc((size_t)gw*gh*sizeof(int)),*link=malloc((nc+1)*sizeof(int));
    if(!head||!link) { free(head);free(link);return 0; }
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
    free(head);free(link);
    rd_cv_keypoint *res=malloc((keep?keep:1)*sizeof *res);
    if(!res) return 0;
    memcpy(res,a,keep*sizeof *res);
    *out=res; *count=keep; return 1;
}
