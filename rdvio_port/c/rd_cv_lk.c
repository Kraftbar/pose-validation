/* SPDX-License-Identifier: Apache-2.0 AND BSD-3-Clause
 * Modified C99 model of OpenCV 4.6.0 video/src/lkpyramid.cpp's
 * CV_SIMD128 && !CV_NEON path (SSE baseline, not a runtime AVX2 kernel).
 * The arithmetic is the reference's: integer fixed-point bilinear samples,
 * float sums accumulated in the reference's lane order. With SSE2 the integer
 * parts run in pmaddwd form (bit-identical: all products/sums are exact ints);
 * the generic C path below the #else is the plain scalar model.
 * Copyright (C) 2000 Intel Corporation. Full retained upstream notices in
 * ../reference_cv/LICENSE-OpenCV-source.
 */
#include "rd_cv.h"
#include <math.h>
#include <float.h>
#include <string.h>
#if defined(__SSE2__) && !defined(RD_CV_NO_SIMD)
#include <emmintrin.h>
#define RD_LK_SSE2 1
#endif
/* Floor division by 2^n (arithmetic shift): identical to the former
 * v>=0 ? v/(1<<n) : -1-((-1-v)/(1<<n)) after the rounding offset is added. */
static inline int descale(int v,int n) {
    return (v+(1<<(n-1)))>>n; /* arithmetic shift == floor division */
}
/* lrintf (round-half-even) without the libm call: adding 2^23 rounds exactly
 * like the default rounding mode for 0 <= f < 2^22. */
static inline int rnd(float f) {
    if(f>=0.f&&f<4194304.f) { float t=f+8388608.f; return (int)(t-8388608.f); }
    return (int)lrintf(f);
}
static inline void weights(float a,float b,int *w) {
    w[0]=rnd((1.f-a)*(1.f-b)*16384);
    w[1]=rnd(a*(1.f-b)*16384);
    w[2]=rnd((1.f-a)*b*16384);
    w[3]=16384-w[0]-w[1]-w[2];
}

static float reduce(const float *q) { return (q[0]+q[2])+(q[1]+q[3]); }

#ifdef RD_LK_SSE2
/* ---------------------------------------------------------------- SSE2 */
/* Window data of the template patch. Columns 0..15 are stored in "pair"
 * layout: 32-bit lane k of a group of 8 columns holds column k in its low 16
 * bits and column k+4 in its high 16 bits, so a single pmaddwd gives
 * d[k]*g[k]+d[k+4]*g[k+4] as the reference's SSE dotprod does. Columns
 * 16..20 (the scalar tail) are stored naturally. */
typedef struct {
    int16_t pp[21*16],gxp[21*16],gyp[21*16];
    int16_t pt[21*8],gxt[21*8],gyt[21*8];
} lk_win;
static inline __m128i ld8(const uint8_t *p) { return _mm_unpacklo_epi8(_mm_loadl_epi64((const __m128i*)p),_mm_setzero_si128()); }
static inline __m128i wpair(int a,int b) { return _mm_set1_epi32((int)(((unsigned)(b&0xffff)<<16)|((unsigned)a&0xffff))); }
static inline __m128i pairv(__m128i lo,__m128i hi) {
    return _mm_or_si128(_mm_and_si128(lo,_mm_set1_epi32(0xffff)),_mm_slli_epi32(hi,16));
}
/* Products of one image row with both weight pairs. A = row used as the top
 * row of the bilinear footprint (w0,w1), B = as the bottom row (w2,w3).
 * Output row y = A(y) + B(y+1): every row is loaded and widened once. */
static inline void row_prod(const uint8_t *s,__m128i W01,__m128i W23,__m128i *A,__m128i *B) {
    const __m128i Z=_mm_setzero_si128();
    __m128i p=_mm_loadu_si128((const __m128i*)s),q=_mm_loadu_si128((const __m128i*)(s+1));
    __m128i a[3]={_mm_unpacklo_epi8(p,Z),_mm_unpackhi_epi8(p,Z),ld8(s+16)};
    __m128i b[3]={_mm_unpacklo_epi8(q,Z),_mm_unpackhi_epi8(q,Z),ld8(s+17)};
    for(int g=0;g<3;g++) {
        __m128i lo=_mm_unpacklo_epi16(a[g],b[g]),hi=_mm_unpackhi_epi16(a[g],b[g]);
        A[2*g]=_mm_madd_epi16(lo,W01); A[2*g+1]=_mm_madd_epi16(hi,W01);
        B[2*g]=_mm_madd_epi16(lo,W23); B[2*g+1]=_mm_madd_epi16(hi,W23);
    }
}
/* the same for the interleaved (dx,dy) derivative row: 3 groups x 2 chunks of 4 px x (lo,hi) */
static inline void deriv_prod(const int16_t *d,__m128i W01,__m128i W23,__m128i *A,__m128i *B) {
    for(int c=0;c<6;c++) {
        const int16_t *pa=d+2*4*c;
        __m128i a=_mm_loadu_si128((const __m128i*)pa),b=_mm_loadu_si128((const __m128i*)(pa+2));
        __m128i lo=_mm_unpacklo_epi16(a,b),hi=_mm_unpackhi_epi16(a,b);
        A[2*c]=_mm_madd_epi16(lo,W01); A[2*c+1]=_mm_madd_epi16(hi,W01);
        B[2*c]=_mm_madd_epi16(lo,W23); B[2*c+1]=_mm_madd_epi16(hi,W23);
    }
}
static inline int win_patch_at(const lk_win *W,int y,int x) {
    if(x>=16) return W->pt[y*8+x-16];
    return W->pp[y*16+(x>>3)*8+2*(x&3)+((x>>2)&1)];
}
/* One output row of the template/derivative patch from the products of rows y (SA/DA) and y+1 (SB/DB). */
static inline void patch_row(lk_win *W,int y,const __m128i *SA,const __m128i *SB,const __m128i *DA,const __m128i *DB,
                             __m128 *va,__m128 *vb,__m128 *vc,float *sa,float *sb,float *sc) {
    const __m128i R9=_mm_set1_epi32(1<<8),R14=_mm_set1_epi32(1<<13),M=_mm_set1_epi32(0xffff);
    for(int g=0;g<3;g++) {
        __m128i lo=_mm_srai_epi32(_mm_add_epi32(_mm_add_epi32(SA[2*g],SB[2*g]),R9),9);
        __m128i hi=_mm_srai_epi32(_mm_add_epi32(_mm_add_epi32(SA[2*g+1],SB[2*g+1]),R9),9);
        if(g<2) _mm_storeu_si128((__m128i*)(W->pp+y*16+g*8),pairv(lo,hi));
        else _mm_storeu_si128((__m128i*)(W->pt+y*8),_mm_packs_epi32(lo,hi));
        __m128i X[2],Y[2];
        for(int j=0;j<2;j++) {
            int c=g*2+j;
            __m128i Lo=_mm_srai_epi32(_mm_add_epi32(_mm_add_epi32(DA[2*c],DB[2*c]),R14),14);
            __m128i Hi=_mm_srai_epi32(_mm_add_epi32(_mm_add_epi32(DA[2*c+1],DB[2*c+1]),R14),14);
            X[j]=_mm_castps_si128(_mm_shuffle_ps(_mm_castsi128_ps(Lo),_mm_castsi128_ps(Hi),0x88));
            Y[j]=_mm_castps_si128(_mm_shuffle_ps(_mm_castsi128_ps(Lo),_mm_castsi128_ps(Hi),0xDD));
        }
        if(g<2) {
            _mm_storeu_si128((__m128i*)(W->gxp+y*16+g*8),pairv(X[0],X[1]));
            _mm_storeu_si128((__m128i*)(W->gyp+y*16+g*8),pairv(Y[0],Y[1]));
            /* four float lanes, pixels 0..15 of the row in order */
            for(int j=0;j<2;j++) {
                __m128i xm=_mm_and_si128(X[j],M),ym=_mm_and_si128(Y[j],M);
                *va=_mm_add_ps(*va,_mm_cvtepi32_ps(_mm_madd_epi16(xm,xm)));
                *vb=_mm_add_ps(*vb,_mm_cvtepi32_ps(_mm_madd_epi16(xm,ym)));
                *vc=_mm_add_ps(*vc,_mm_cvtepi32_ps(_mm_madd_epi16(ym,ym)));
            }
        } else {
            _mm_storeu_si128((__m128i*)(W->gxt+y*8),_mm_packs_epi32(X[0],X[1]));
            _mm_storeu_si128((__m128i*)(W->gyt+y*8),_mm_packs_epi32(Y[0],Y[1]));
        }
    }
    for(int x=0;x<5;x++) { int dx=W->gxt[y*8+x],dy=W->gyt[y*8+x];
        *sa+=(float)(dx*dx); *sb+=(float)(dx*dy); *sc+=(float)(dy*dy); }
}
/* Template patch + derivative patch for the 21x21 window at (ix,iy) and the
 * three structure-tensor sums (reference lane order). */
static inline void lk_patch(const uint8_t *ir,const int16_t *dr,int S,int ix,int iy,const int *w,
                            lk_win *W,float *oA11,float *oA12,float *oA22) {
    const __m128i W01=wpair(w[0],w[1]),W23=wpair(w[2],w[3]);
    __m128 va=_mm_setzero_ps(),vb=_mm_setzero_ps(),vc=_mm_setzero_ps();
    float sa=0,sb=0,sc=0;
    __m128i SA0[6],SA1[6],SB[6],DA0[12],DA1[12],DB[12];
    row_prod(ir+(size_t)iy*S+ix,W01,W23,SA0,SB);
    deriv_prod(dr+2*((size_t)iy*S+ix),W01,W23,DA0,DB);
    for(int y=0;y<21;y+=2) {
        size_t off=(size_t)(iy+y+1)*S+ix;
        row_prod(ir+off,W01,W23,SA1,SB);
        deriv_prod(dr+2*off,W01,W23,DA1,DB);
        patch_row(W,y,SA0,SB,DA0,DB,&va,&vb,&vc,&sa,&sb,&sc);
        if(y==20) break;
        off+=S;
        row_prod(ir+off,W01,W23,SA0,SB);
        deriv_prod(dr+2*off,W01,W23,DA0,DB);
        patch_row(W,y+1,SA1,SB,DA1,DB,&va,&vb,&vc,&sa,&sb,&sc);
    }
    float qa[4],qb[4],qc[4];
    _mm_storeu_ps(qa,va); _mm_storeu_ps(qb,vb); _mm_storeu_ps(qc,vc);
    *oA11=(sa+reduce(qa))*(1.f/1048576);
    *oA12=(sb+reduce(qb))*(1.f/1048576);
    *oA22=(sc+reduce(qc))*(1.f/1048576);
}
/* one Gauss-Newton residual accumulation: b1,b2 */
static inline void lk_dot(const uint8_t *jb,int S,const int *w,const lk_win *W,float *ob1,float *ob2) {
    const __m128i W01=wpair(w[0],w[1]),W23=wpair(w[2],w[3]),R9=_mm_set1_epi32(1<<8);
    __m128 ax=_mm_setzero_ps(),ay=_mm_setzero_ps(),s1=_mm_setzero_ps(),s2=_mm_setzero_ps();
    __m128i A[6],B[6],An[6];
    row_prod(jb,W01,W23,A,B);
    for(int y=0;y<21;y++) {
        row_prod(jb+(size_t)(y+1)*S,W01,W23,An,B);
        for(int g=0;g<2;g++) {
            __m128i lo=_mm_srai_epi32(_mm_add_epi32(_mm_add_epi32(A[2*g],B[2*g]),R9),9);
            __m128i hi=_mm_srai_epi32(_mm_add_epi32(_mm_add_epi32(A[2*g+1],B[2*g+1]),R9),9);
            __m128i d=_mm_sub_epi16(pairv(lo,hi),_mm_loadu_si128((const __m128i*)(W->pp+y*16+g*8)));
            ax=_mm_add_ps(ax,_mm_cvtepi32_ps(_mm_madd_epi16(d,_mm_loadu_si128((const __m128i*)(W->gxp+y*16+g*8)))));
            ay=_mm_add_ps(ay,_mm_cvtepi32_ps(_mm_madd_epi16(d,_mm_loadu_si128((const __m128i*)(W->gyp+y*16+g*8)))));
        }
        {   /* columns 16..20, one product at a time in column order */
            __m128i lo=_mm_srai_epi32(_mm_add_epi32(_mm_add_epi32(A[4],B[4]),R9),9);
            __m128i hi=_mm_srai_epi32(_mm_add_epi32(_mm_add_epi32(A[5],B[5]),R9),9);
            __m128i d=_mm_sub_epi16(_mm_packs_epi32(lo,hi),_mm_loadu_si128((const __m128i*)(W->pt+y*8)));
            __m128i gx=_mm_loadu_si128((const __m128i*)(W->gxt+y*8)),gy=_mm_loadu_si128((const __m128i*)(W->gyt+y*8));
            __m128i xl=_mm_mullo_epi16(d,gx),xh=_mm_mulhi_epi16(d,gx),yl=_mm_mullo_epi16(d,gy),yh=_mm_mulhi_epi16(d,gy);
            __m128 px0=_mm_cvtepi32_ps(_mm_unpacklo_epi16(xl,xh)),px1=_mm_cvtepi32_ps(_mm_unpackhi_epi16(xl,xh));
            __m128 py0=_mm_cvtepi32_ps(_mm_unpacklo_epi16(yl,yh)),py1=_mm_cvtepi32_ps(_mm_unpackhi_epi16(yl,yh));
            s1=_mm_add_ss(s1,px0); s1=_mm_add_ss(s1,_mm_shuffle_ps(px0,px0,1));
            s1=_mm_add_ss(s1,_mm_shuffle_ps(px0,px0,2)); s1=_mm_add_ss(s1,_mm_shuffle_ps(px0,px0,3));
            s1=_mm_add_ss(s1,px1);
            s2=_mm_add_ss(s2,py0); s2=_mm_add_ss(s2,_mm_shuffle_ps(py0,py0,1));
            s2=_mm_add_ss(s2,_mm_shuffle_ps(py0,py0,2)); s2=_mm_add_ss(s2,_mm_shuffle_ps(py0,py0,3));
            s2=_mm_add_ss(s2,py1);
        }
        for(int i=0;i<6;i++) A[i]=An[i];
    }
    float X[4],Y[4];
    _mm_storeu_ps(X,ax); _mm_storeu_ps(Y,ay);
    *ob1=(_mm_cvtss_f32(s1)+((X[0]+X[2])+(X[1]+X[3])))*(1.f/1048576);
    *ob2=(_mm_cvtss_f32(s2)+((Y[0]+Y[2])+(Y[1]+Y[3])))*(1.f/1048576);
}

#if (defined(__GNUC__)||defined(__clang__)) && defined(__x86_64__) && !defined(RD_CV_NO_AVX2)
/* ---------------------------------------------------------------- AVX2 */
/* Same arithmetic with columns 0..15 in one 256-bit register (low half =
 * columns 0..7, high half = 8..15; every operation used is in-lane, so the
 * halves behave exactly like the two SSE groups) and the five tail columns
 * in the SSE form. Chosen at run time; results are identical to lk_patch /
 * lk_dot above (the integer parts are exact, the float adds keep their order). */
#define RD_LK_AVX2 1
#include <immintrin.h>
#define AVX2_FN __attribute__((target("avx2"))) static inline
AVX2_FN __m256i wpair256(int a,int b) { return _mm256_set1_epi32((int)(((unsigned)(b&0xffff)<<16)|((unsigned)a&0xffff))); }
AVX2_FN __m256i pairv256(__m256i lo,__m256i hi) {
    return _mm256_or_si256(_mm256_and_si256(lo,_mm256_set1_epi32(0xffff)),_mm256_slli_epi32(hi,16));
}
/* A[0..1],B[0..1]: columns 0..15 (lo,hi of the in-lane unpack); At/Bt: columns 16..23 */
AVX2_FN void row_prod256(const uint8_t *s,__m256i W01,__m256i W23,__m256i *A,__m256i *B,__m128i *At,__m128i *Bt) {
    __m256i a=_mm256_cvtepu8_epi16(_mm_loadu_si128((const __m128i*)s)),b=_mm256_cvtepu8_epi16(_mm_loadu_si128((const __m128i*)(s+1)));
    __m256i lo=_mm256_unpacklo_epi16(a,b),hi=_mm256_unpackhi_epi16(a,b);
    A[0]=_mm256_madd_epi16(lo,W01); A[1]=_mm256_madd_epi16(hi,W01);
    B[0]=_mm256_madd_epi16(lo,W23); B[1]=_mm256_madd_epi16(hi,W23);
    __m128i ta=ld8(s+16),tb=ld8(s+17),w01=_mm256_castsi256_si128(W01),w23=_mm256_castsi256_si128(W23);
    __m128i tl=_mm_unpacklo_epi16(ta,tb),th=_mm_unpackhi_epi16(ta,tb);
    At[0]=_mm_madd_epi16(tl,w01); At[1]=_mm_madd_epi16(th,w01);
    Bt[0]=_mm_madd_epi16(tl,w23); Bt[1]=_mm_madd_epi16(th,w23);
}
/* derivative row: chunk j (0,1) holds columns 4j..4j+3 of group 0 (low half) and 8+4j.. of group 1 (high half) */
AVX2_FN void deriv_prod256(const int16_t *d,__m256i W01,__m256i W23,__m256i *A,__m256i *B,__m128i *At,__m128i *Bt) {
    for(int j=0;j<2;j++) {
        const int16_t *p0=d+2*4*j,*p1=d+2*(8+4*j);
        __m256i a=_mm256_inserti128_si256(_mm256_castsi128_si256(_mm_loadu_si128((const __m128i*)p0)),_mm_loadu_si128((const __m128i*)p1),1);
        __m256i b=_mm256_inserti128_si256(_mm256_castsi128_si256(_mm_loadu_si128((const __m128i*)(p0+2))),_mm_loadu_si128((const __m128i*)(p1+2)),1);
        __m256i lo=_mm256_unpacklo_epi16(a,b),hi=_mm256_unpackhi_epi16(a,b);
        A[2*j]=_mm256_madd_epi16(lo,W01); A[2*j+1]=_mm256_madd_epi16(hi,W01);
        B[2*j]=_mm256_madd_epi16(lo,W23); B[2*j+1]=_mm256_madd_epi16(hi,W23);
    }
    __m128i w01=_mm256_castsi256_si128(W01),w23=_mm256_castsi256_si128(W23);
    for(int j=0;j<2;j++) {
        const int16_t *pa=d+2*(16+4*j);
        __m128i a=_mm_loadu_si128((const __m128i*)pa),b=_mm_loadu_si128((const __m128i*)(pa+2));
        __m128i lo=_mm_unpacklo_epi16(a,b),hi=_mm_unpackhi_epi16(a,b);
        At[2*j]=_mm_madd_epi16(lo,w01); At[2*j+1]=_mm_madd_epi16(hi,w01);
        Bt[2*j]=_mm_madd_epi16(lo,w23); Bt[2*j+1]=_mm_madd_epi16(hi,w23);
    }
}
AVX2_FN void patch_row256(lk_win *W,int y,const __m256i *SA,const __m256i *SB,const __m128i *SAt,const __m128i *SBt,
                          const __m256i *DA,const __m256i *DB,const __m128i *DAt,const __m128i *DBt,
                          __m128 *va,__m128 *vb,__m128 *vc,float *sa,float *sb,float *sc) {
    const __m256i R9=_mm256_set1_epi32(1<<8),R14=_mm256_set1_epi32(1<<13),M=_mm256_set1_epi32(0xffff);
    __m256i lo=_mm256_srai_epi32(_mm256_add_epi32(_mm256_add_epi32(SA[0],SB[0]),R9),9);
    __m256i hi=_mm256_srai_epi32(_mm256_add_epi32(_mm256_add_epi32(SA[1],SB[1]),R9),9);
    _mm256_storeu_si256((__m256i*)(W->pp+y*16),pairv256(lo,hi));
    __m256i X[2],Y[2];
    for(int j=0;j<2;j++) {
        __m256i Lo=_mm256_srai_epi32(_mm256_add_epi32(_mm256_add_epi32(DA[2*j],DB[2*j]),R14),14);
        __m256i Hi=_mm256_srai_epi32(_mm256_add_epi32(_mm256_add_epi32(DA[2*j+1],DB[2*j+1]),R14),14);
        X[j]=_mm256_castps_si256(_mm256_shuffle_ps(_mm256_castsi256_ps(Lo),_mm256_castsi256_ps(Hi),0x88));
        Y[j]=_mm256_castps_si256(_mm256_shuffle_ps(_mm256_castsi256_ps(Lo),_mm256_castsi256_ps(Hi),0xDD));
    }
    _mm256_storeu_si256((__m256i*)(W->gxp+y*16),pairv256(X[0],X[1]));
    _mm256_storeu_si256((__m256i*)(W->gyp+y*16),pairv256(Y[0],Y[1]));
    {   /* lane order: group 0 chunk 0, chunk 1, group 1 chunk 0, chunk 1 */
        __m256 pa[2],pb[2],pc[2];
        for(int j=0;j<2;j++) {
            __m256i xm=_mm256_and_si256(X[j],M),ym=_mm256_and_si256(Y[j],M);
            pa[j]=_mm256_cvtepi32_ps(_mm256_madd_epi16(xm,xm));
            pb[j]=_mm256_cvtepi32_ps(_mm256_madd_epi16(xm,ym));
            pc[j]=_mm256_cvtepi32_ps(_mm256_madd_epi16(ym,ym));
        }
        *va=_mm_add_ps(*va,_mm256_castps256_ps128(pa[0])); *va=_mm_add_ps(*va,_mm256_castps256_ps128(pa[1]));
        *va=_mm_add_ps(*va,_mm256_extractf128_ps(pa[0],1)); *va=_mm_add_ps(*va,_mm256_extractf128_ps(pa[1],1));
        *vb=_mm_add_ps(*vb,_mm256_castps256_ps128(pb[0])); *vb=_mm_add_ps(*vb,_mm256_castps256_ps128(pb[1]));
        *vb=_mm_add_ps(*vb,_mm256_extractf128_ps(pb[0],1)); *vb=_mm_add_ps(*vb,_mm256_extractf128_ps(pb[1],1));
        *vc=_mm_add_ps(*vc,_mm256_castps256_ps128(pc[0])); *vc=_mm_add_ps(*vc,_mm256_castps256_ps128(pc[1]));
        *vc=_mm_add_ps(*vc,_mm256_extractf128_ps(pc[0],1)); *vc=_mm_add_ps(*vc,_mm256_extractf128_ps(pc[1],1));
    }
    {   /* columns 16..20 */
        const __m128i r9=_mm_set1_epi32(1<<8),r14=_mm_set1_epi32(1<<13);
        __m128i tlo=_mm_srai_epi32(_mm_add_epi32(_mm_add_epi32(SAt[0],SBt[0]),r9),9);
        __m128i thi=_mm_srai_epi32(_mm_add_epi32(_mm_add_epi32(SAt[1],SBt[1]),r9),9);
        _mm_storeu_si128((__m128i*)(W->pt+y*8),_mm_packs_epi32(tlo,thi));
        __m128i Xt[2],Yt[2];
        for(int j=0;j<2;j++) {
            __m128i Lo=_mm_srai_epi32(_mm_add_epi32(_mm_add_epi32(DAt[2*j],DBt[2*j]),r14),14);
            __m128i Hi=_mm_srai_epi32(_mm_add_epi32(_mm_add_epi32(DAt[2*j+1],DBt[2*j+1]),r14),14);
            Xt[j]=_mm_castps_si128(_mm_shuffle_ps(_mm_castsi128_ps(Lo),_mm_castsi128_ps(Hi),0x88));
            Yt[j]=_mm_castps_si128(_mm_shuffle_ps(_mm_castsi128_ps(Lo),_mm_castsi128_ps(Hi),0xDD));
        }
        _mm_storeu_si128((__m128i*)(W->gxt+y*8),_mm_packs_epi32(Xt[0],Xt[1]));
        _mm_storeu_si128((__m128i*)(W->gyt+y*8),_mm_packs_epi32(Yt[0],Yt[1]));
    }
    for(int x=0;x<5;x++) { int dx=W->gxt[y*8+x],dy=W->gyt[y*8+x];
        *sa+=(float)(dx*dx); *sb+=(float)(dx*dy); *sc+=(float)(dy*dy); }
}
__attribute__((target("avx2"))) static void lk_patch_avx2(const uint8_t *ir,const int16_t *dr,int S,int ix,int iy,const int *w,
                                                          lk_win *W,float *oA11,float *oA12,float *oA22) {
    const __m256i W01=wpair256(w[0],w[1]),W23=wpair256(w[2],w[3]);
    __m128 va=_mm_setzero_ps(),vb=_mm_setzero_ps(),vc=_mm_setzero_ps();
    float sa=0,sb=0,sc=0;
    __m256i SA0[2],SA1[2],SB[2],DA0[4],DA1[4],DB[4];
    __m128i SAt0[2],SAt1[2],SBt[2],DAt0[4],DAt1[4],DBt[4];
    row_prod256(ir+(size_t)iy*S+ix,W01,W23,SA0,SB,SAt0,SBt);
    deriv_prod256(dr+2*((size_t)iy*S+ix),W01,W23,DA0,DB,DAt0,DBt);
    for(int y=0;y<21;y+=2) {
        size_t off=(size_t)(iy+y+1)*S+ix;
        row_prod256(ir+off,W01,W23,SA1,SB,SAt1,SBt);
        deriv_prod256(dr+2*off,W01,W23,DA1,DB,DAt1,DBt);
        patch_row256(W,y,SA0,SB,SAt0,SBt,DA0,DB,DAt0,DBt,&va,&vb,&vc,&sa,&sb,&sc);
        if(y==20) break;
        off+=S;
        row_prod256(ir+off,W01,W23,SA0,SB,SAt0,SBt);
        deriv_prod256(dr+2*off,W01,W23,DA0,DB,DAt0,DBt);
        patch_row256(W,y+1,SA1,SB,SAt1,SBt,DA1,DB,DAt1,DBt,&va,&vb,&vc,&sa,&sb,&sc);
    }
    float qa[4],qb[4],qc[4];
    _mm_storeu_ps(qa,va); _mm_storeu_ps(qb,vb); _mm_storeu_ps(qc,vc);
    *oA11=(sa+reduce(qa))*(1.f/1048576);
    *oA12=(sb+reduce(qb))*(1.f/1048576);
    *oA22=(sc+reduce(qc))*(1.f/1048576);
}
__attribute__((target("avx2"))) static void lk_dot_avx2(const uint8_t *jb,int S,const int *w,const lk_win *W,float *ob1,float *ob2) {
    const __m256i W01=wpair256(w[0],w[1]),W23=wpair256(w[2],w[3]),R9=_mm256_set1_epi32(1<<8);
    const __m128i r9=_mm_set1_epi32(1<<8);
    __m128 ax=_mm_setzero_ps(),ay=_mm_setzero_ps(),s1=_mm_setzero_ps(),s2=_mm_setzero_ps();
    __m256i A0[2],A1[2],B[2];
    __m128i At0[2],At1[2],Bt[2];
    row_prod256(jb,W01,W23,A0,B,At0,Bt);
    for(int y=0;y<21;y+=2) {
        for(int half=0;half<2&&y+half<21;half++) {
            int yy=y+half;
            __m256i *A=half?A1:A0,*An=half?A0:A1; __m128i *At=half?At1:At0,*Atn=half?At0:At1;
            row_prod256(jb+(size_t)(yy+1)*S,W01,W23,An,B,Atn,Bt);
            __m256i lo=_mm256_srai_epi32(_mm256_add_epi32(_mm256_add_epi32(A[0],B[0]),R9),9);
            __m256i hi=_mm256_srai_epi32(_mm256_add_epi32(_mm256_add_epi32(A[1],B[1]),R9),9);
            __m256i d=_mm256_sub_epi16(pairv256(lo,hi),_mm256_loadu_si256((const __m256i*)(W->pp+yy*16)));
            __m256 cx=_mm256_cvtepi32_ps(_mm256_madd_epi16(d,_mm256_loadu_si256((const __m256i*)(W->gxp+yy*16))));
            __m256 cy=_mm256_cvtepi32_ps(_mm256_madd_epi16(d,_mm256_loadu_si256((const __m256i*)(W->gyp+yy*16))));
            ax=_mm_add_ps(ax,_mm256_castps256_ps128(cx)); ax=_mm_add_ps(ax,_mm256_extractf128_ps(cx,1));
            ay=_mm_add_ps(ay,_mm256_castps256_ps128(cy)); ay=_mm_add_ps(ay,_mm256_extractf128_ps(cy,1));
            {   /* columns 16..20, one product at a time in column order */
                __m128i tlo=_mm_srai_epi32(_mm_add_epi32(_mm_add_epi32(At[0],Bt[0]),r9),9);
                __m128i thi=_mm_srai_epi32(_mm_add_epi32(_mm_add_epi32(At[1],Bt[1]),r9),9);
                __m128i dt=_mm_sub_epi16(_mm_packs_epi32(tlo,thi),_mm_loadu_si128((const __m128i*)(W->pt+yy*8)));
                __m128i gx=_mm_loadu_si128((const __m128i*)(W->gxt+yy*8)),gy=_mm_loadu_si128((const __m128i*)(W->gyt+yy*8));
                __m128i xl=_mm_mullo_epi16(dt,gx),xh=_mm_mulhi_epi16(dt,gx),yl=_mm_mullo_epi16(dt,gy),yh=_mm_mulhi_epi16(dt,gy);
                __m128 px0=_mm_cvtepi32_ps(_mm_unpacklo_epi16(xl,xh)),px1=_mm_cvtepi32_ps(_mm_unpackhi_epi16(xl,xh));
                __m128 py0=_mm_cvtepi32_ps(_mm_unpacklo_epi16(yl,yh)),py1=_mm_cvtepi32_ps(_mm_unpackhi_epi16(yl,yh));
                s1=_mm_add_ss(s1,px0); s1=_mm_add_ss(s1,_mm_shuffle_ps(px0,px0,1));
                s1=_mm_add_ss(s1,_mm_shuffle_ps(px0,px0,2)); s1=_mm_add_ss(s1,_mm_shuffle_ps(px0,px0,3));
                s1=_mm_add_ss(s1,px1);
                s2=_mm_add_ss(s2,py0); s2=_mm_add_ss(s2,_mm_shuffle_ps(py0,py0,1));
                s2=_mm_add_ss(s2,_mm_shuffle_ps(py0,py0,2)); s2=_mm_add_ss(s2,_mm_shuffle_ps(py0,py0,3));
                s2=_mm_add_ss(s2,py1);
            }
        }
    }
    float X[4],Y[4];
    _mm_storeu_ps(X,ax); _mm_storeu_ps(Y,ay);
    *ob1=(_mm_cvtss_f32(s1)+((X[0]+X[2])+(X[1]+X[3])))*(1.f/1048576);
    *ob2=(_mm_cvtss_f32(s2)+((Y[0]+Y[2])+(Y[1]+Y[3])))*(1.f/1048576);
}
static int lk_have_avx2(void) {
    static int v=-1;
    if(v<0) { __builtin_cpu_init(); v=__builtin_cpu_supports("avx2")!=0; }
    return v;
}
#endif
#else
/* -------------------------------------------------------- generic C */
typedef struct { int16_t patch[441],gx[441],gy[441]; } lk_win;
static inline int win_patch_at(const lk_win *W,int y,int x) { return W->patch[y*21+x]; }
static inline void sample_row(const uint8_t *s0,const uint8_t *s1,int w0,int w1,int w2,int w3,int *o) {
    for(int x=0;x<21;x++)
        o[x]=(s0[x]*w0+s0[x+1]*w1+s1[x]*w2+s1[x+1]*w3+(1<<8))>>9;
}
static inline void lk_patch(const uint8_t *ir,const int16_t *dr,int S,int ix,int iy,const int *w,
                            lk_win *W,float *oA11,float *oA12,float *oA22) {
    float sa=0,sb=0,sc=0,qa[4]={0},qb[4]={0},qc[4]={0};
    for(int y=0;y<21;y++) {
        int off=(iy+y)*S+ix, t=y*21;
        const uint8_t *s0=ir+off,*s1=s0+S;
        const int16_t *d0=dr+2*off,*d1=d0+2*S;
        int ps[21],ddx[21],ddy[21];
        sample_row(s0,s1,w[0],w[1],w[2],w[3],ps);
        for(int x=0;x<21;x++) {
            ddx[x]=(d0[2*x]*w[0]+d0[2*x+2]*w[1]+d1[2*x]*w[2]+d1[2*x+2]*w[3]+(1<<13))>>14;
            ddy[x]=(d0[2*x+1]*w[0]+d0[2*x+3]*w[1]+d1[2*x+1]*w[2]+d1[2*x+3]*w[3]+(1<<13))>>14;
            W->patch[t+x]=(int16_t)ps[x]; W->gx[t+x]=(int16_t)ddx[x]; W->gy[t+x]=(int16_t)ddy[x];
        }
        /* 16 SIMD pixels, four lanes updated four times per row;
         * the remaining five pixels have separate scalar sums. */
        for(int x=0;x<16;x++) { int k=x&3; int dx=ddx[x],dy=ddy[x];
            qa[k]+=(float)(dx*dx); qb[k]+=(float)(dx*dy); qc[k]+=(float)(dy*dy); }
        for(int x=16;x<21;x++) { int dx=ddx[x],dy=ddy[x];
            sa+=(float)(dx*dx); sb+=(float)(dx*dy); sc+=(float)(dy*dy); }
    }
    *oA11=(sa+reduce(qa))*(1.f/1048576);
    *oA12=(sb+reduce(qb))*(1.f/1048576);
    *oA22=(sc+reduce(qc))*(1.f/1048576);
}
static inline void lk_dot(const uint8_t *jb,int S,const int *w,const lk_win *W,float *ob1,float *ob2) {
    float s1=0,s2=0,f[8]={0};
    for(int y=0;y<21;y++) {
        int diff[21];
        const uint8_t *r0=jb+(size_t)y*S;
        sample_row(r0,r0+S,w[0],w[1],w[2],w[3],diff);
        const int16_t *pr=W->patch+y*21,*gxr=W->gx+y*21,*gyr=W->gy+y*21;
        for(int x=0;x<21;x++)diff[x]-=pr[x];
        /* SSE dotprod sums integer products at x and x+4 BEFORE conversion;
         * accumulator f[2k],f[2k+1] = (X,Y) of lane k (f[0..3]=q0, f[4..7]=q1
         * of the SSE reference). */
        for(int x=0;x<16;x+=8)for(int k=0;k<4;k++) {
            int i=x+k;
            f[2*k]  +=(float)(diff[i]*gxr[i]+diff[i+4]*gxr[i+4]);
            f[2*k+1]+=(float)(diff[i]*gyr[i]+diff[i+4]*gyr[i+4]);
        }
        for(int x=16;x<21;x++) {
            s1+=(float)(diff[x]*gxr[x]); s2+=(float)(diff[x]*gyr[x]); }
    }
    *ob1=(s1+((f[0]+f[4])+(f[2]+f[6])))*(1.f/1048576);
    *ob2=(s2+((f[1]+f[5])+(f[3]+f[7])))*(1.f/1048576);
}
#endif
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
#ifdef RD_LK_AVX2
    const int avx2=lk_have_avx2();
#endif
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
            lk_win W;
            float A11,A12,A22;
#ifdef RD_LK_AVX2
            if(avx2) lk_patch_avx2(ir,dr,I->step,ix,iy,w,&W,&A11,&A12,&A22); else
#endif
            lk_patch(ir,dr,I->step,ix,iy,w,&W,&A11,&A12,&A22);
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
                float b1,b2;
#ifdef RD_LK_AVX2
                if(avx2) lk_dot_avx2(jr+ny*J->step+nx,J->step,w,&W,&b1,&b2); else
#endif
                lk_dot(jr+ny*J->step+nx,J->step,w,&W,&b1,&b2);
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
                /* sum of |int| below 441*8160 < 2^24: the reference's float
                 * accumulation is exact, so an int sum converts to the same value */
                int ei=0;
                for(int y=0;y<21;y++) {
                    const uint8_t *r0=jr+(ny+y)*J->step+nx;
                    for(int x=0;x<21;x++) {
                        int v=(r0[x]*w[0]+r0[x+1]*w[1]+r0[x+J->step]*w[2]+r0[x+J->step+1]*w[3]+(1<<8))>>9;
                        int dd=v-win_patch_at(&W,y,x);
                        ei+=dd<0?-dd:dd;
                    }
                }
                float e=(float)ei;
                err[p]=e/14112;
            }
        }
    }
    return 1;
}
