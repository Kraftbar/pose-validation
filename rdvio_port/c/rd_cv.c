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
#if defined(__SSE2__) && !defined(RD_CV_NO_SIMD)
#include <emmintrin.h>
#define RD_SSE2 1
#endif

static int reflect(int x, int n) {
    if (n == 1) return 0;
    while (x < 0 || x >= n) x = x < 0 ? -x : 2*n-x-2;
    return x;
}
static int dimensions(int w, int h) { return w > 0 && h > 0 && w <= 16384 && h <= 16384; }
static inline uint8_t byte_round(float f) {
    int v;
    if(f>=0.f&&f<4194304.f) {
        /* adding 2^23 rounds to nearest-even exactly like lrintf in the
         * default rounding mode (SSE float arithmetic, no excess precision) */
        float m=f+8388608.f;
        v=(int)(m-8388608.f);
    } else v=(int)lrintf(f);
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
        int hist[256], clipped=0;
        int x0=tx*tw;
        {   /* four interleaved sub-histograms break the store->load dependency of repeated bins */
            int hs[4][256]; memset(hs,0,sizeof hs);
            for (int y=0;y<th;y++) {
                const uint8_t *row=src+(size_t)reflect(ty*th+y,h)*w;
                if(x0+tw<=w) {
                    const uint8_t *q=row+x0; int x=0;
                    for(;x+4<=tw;x+=4) { hs[0][q[x]]++; hs[1][q[x+1]]++; hs[2][q[x+2]]++; hs[3][q[x+3]]++; }
                    for(;x<tw;x++) hs[0][q[x]]++;
                } else for(int x=0;x<tw;x++) hs[0][row[reflect(x0+x,w)]]++;
            }
            for(int i=0;i<256;i++) hist[i]=hs[0][i]+hs[1][i]+hs[2][i]+hs[3][i];
        }
        for (int i=0;i<256;i++) if(hist[i]>clip) { clipped+=hist[i]-clip; hist[i]=clip; }
        int batch=clipped/256, residual=clipped-batch*256;
        for(int i=0;i<256;i++) hist[i]+=batch;
        if(residual) { int step=256/residual; if(step<1)step=1;
            for(int i=0;i<256&&residual;i+=step,residual--) hist[i]++; }
        int sum=0;
        for(int i=0;i<256;i++) { sum+=hist[i]; lut[ty*8+tx][i]=byte_round(sum*scale); }
    }
    float iw=1.f/tw, ih=1.f/th;
    /* per-column interpolation setup is independent of y */
    float *cxa1=malloc((size_t)w*2*sizeof(float)),*cxa2;
    struct { int xs,xe,x1,x2; } *run=malloc((size_t)(w+1)*sizeof *run);
    int nruns=0;
    if(!cxa1||!run) { free(cxa1); free(run); return 0; }
    cxa2=cxa1+w;
    for(int x=0;x<w;x++) {
        float xf=x*iw-.5f; int x1=(int)floorf(xf),x2=x1+1;
        float xa=xf-x1, xa1=1.f-xa;
        if(x1<0)x1=0;
        if(x2>7)x2=7;
        cxa1[x]=xa1; cxa2[x]=xa;
        if(nruns&&run[nruns-1].x1==x1&&run[nruns-1].x2==x2) run[nruns-1].xe=x+1;
        else { run[nruns].xs=x; run[nruns].xe=x+1; run[nruns].x1=x1; run[nruns].x2=x2; nruns++; }
    }
    for(int y=0;y<h;y++) {
        float yf=y*ih-.5f; int y1=(int)floorf(yf), y2=y1+1;
        float ya=yf-y1, ya1=1.f-ya;
        if(y1<0)y1=0;
        if(y2>7)y2=7;
        const uint8_t *sr=src+(size_t)y*w; uint8_t *dr=dst+(size_t)y*w;
        /* columns come in runs sharing the same pair of LUT columns (x1,x2) */
        for(int ri=0;ri<nruns;ri++) {
            const uint8_t *A=lut[y1*8+run[ri].x1],*B=lut[y1*8+run[ri].x2],*C=lut[y2*8+run[ri].x1],*D=lut[y2*8+run[ri].x2];
            int x=run[ri].xs,xe=run[ri].xe;
#ifdef RD_SSE2
            {   /* four pixels per step; same float operations in the same order, per lane */
                const __m128 yy1=_mm_set1_ps(ya1),yy=_mm_set1_ps(ya),magic=_mm_set1_ps(8388608.f);
                for(;x+4<=xe;x+=4) {
                    int v0=sr[x],v1=sr[x+1],v2=sr[x+2],v3=sr[x+3];
                    __m128 fa=_mm_cvtepi32_ps(_mm_set_epi32(A[v3],A[v2],A[v1],A[v0]));
                    __m128 fb=_mm_cvtepi32_ps(_mm_set_epi32(B[v3],B[v2],B[v1],B[v0]));
                    __m128 fc=_mm_cvtepi32_ps(_mm_set_epi32(C[v3],C[v2],C[v1],C[v0]));
                    __m128 fd=_mm_cvtepi32_ps(_mm_set_epi32(D[v3],D[v2],D[v1],D[v0]));
                    __m128 xa1=_mm_loadu_ps(cxa1+x),xa=_mm_loadu_ps(cxa2+x);
                    __m128 top=_mm_mul_ps(_mm_add_ps(_mm_mul_ps(fa,xa1),_mm_mul_ps(fb,xa)),yy1);
                    __m128 bot=_mm_mul_ps(_mm_add_ps(_mm_mul_ps(fc,xa1),_mm_mul_ps(fd,xa)),yy);
                    __m128 r=_mm_add_ps(top,bot);
                    /* r in [0, 2^22): add/sub 2^23 rounds half-to-even like lrintf */
                    __m128i ri32=_mm_cvttps_epi32(_mm_sub_ps(_mm_add_ps(r,magic),magic));
                    int out4=_mm_cvtsi128_si32(_mm_packus_epi16(_mm_packs_epi32(ri32,ri32),_mm_setzero_si128()));
                    memcpy(dr+x,&out4,4);
                }
            }
#endif
            for(;x<xe;x++) {
                float xa1=cxa1[x],xa=cxa2[x];
                int v=sr[x];
                float r=(A[v]*xa1+B[v]*xa)*ya1
                       +(C[v]*xa1+D[v]*xa)*ya;
                dr[x]=byte_round(r);
            }
        }
    }
    free(cxa1); free(run);
    return 1;
}
/* Level buffers are recycled: allocation + page faults + zeroing of ~1.5 MB
 * per level per frame was a visible cost. The deriv border (never written,
 * must read as 0) is re-zeroed on reuse. Single-threaded use, as before. */
typedef struct pool_ent { size_t n; uint8_t *image; int16_t *deriv; struct pool_ent *next; } pool_ent;
static pool_ent *pool_head;
static int pool_count;
static int pool_take(size_t n,uint8_t **img,int16_t **der) {
    for(pool_ent **pp=&pool_head;*pp;pp=&(*pp)->next) if((*pp)->n==n) {
        pool_ent *e=*pp; *pp=e->next; *img=e->image; *der=e->deriv; free(e); pool_count--; return 1; }
    return 0;
}
static void pool_give(size_t n,uint8_t *img,int16_t *der) {
    if(!img||!der||pool_count>=16) { free(img); free(der); return; }
    pool_ent *e=malloc(sizeof *e);
    if(!e) { free(img); free(der); return; }
    e->n=n; e->image=img; e->deriv=der; e->next=pool_head; pool_head=e; pool_count++;
}
void rd_cv_free_pyramid(rd_cv_pyramid *p) {
    if(!p)return;
    for(int l=0;l<4;l++) {
        rd_cv_level *q=&p->level[l];
        if(q->image&&q->deriv) pool_give((size_t)q->step*(q->height+42),q->image,q->deriv);
        else { free(q->image); free(q->deriv); }
    }
    memset(p,0,sizeof(*p));
}
int rd_cv_build_pyramid(const uint8_t *src,int w,int h,rd_cv_pyramid *p) {
    if(!src||!p||!dimensions(w,h)||p->count)return 0;
    uint16_t *hbuf=NULL; size_t hcap=0;
    for(int l=0;l<4;l++) {
        rd_cv_level *q=&p->level[l];
        q->width=w; q->height=h; q->step=w+42;
        const int S=q->step;
        size_t n=(size_t)S*(h+42);
        if(!pool_take(n,&q->image,&q->deriv)) {
            q->image=malloc(n+32); /* slack: SIMD readers overrun a row end by < 16 bytes */ q->deriv=calloc(n+16,2*sizeof(int16_t)); /* slack for SIMD overrun */
        } else {
            /* border of deriv: rows above/below the ROI and the 21-px side strips */
            memset(q->deriv,0,(size_t)21*S*2*sizeof(int16_t));
            memset(q->deriv+2*(size_t)(21+h)*S,0,(size_t)21*S*2*sizeof(int16_t));
            for(int y=0;y<h;y++) {
                int16_t *r=q->deriv+2*((size_t)(y+21)*S);
                memset(r,0,21*2*sizeof(int16_t)); memset(r+2*(21+w),0,21*2*sizeof(int16_t));
            }
        }
        if(!q->image||!q->deriv) { free(hbuf); rd_cv_free_pyramid(p); return 0; }
        uint8_t *roi=q->image+21*S+21;
        if(!l) {
            for(int y=0;y<h;y++) memcpy(roi+(size_t)y*S,src+(size_t)y*w,(size_t)w);
        } else {
            /* Separable 1-4-6-4-1: integer sums, so the factorisation is exact. */
            rd_cv_level *a=&p->level[l-1];
            const uint8_t *r=a->image+21*a->step+21;
            int ah=a->height,as=a->step;
            size_t need=(size_t)ah*w;
            if(need>hcap) { free(hbuf); hbuf=malloc(need*sizeof *hbuf); hcap=need; if(!hbuf) { rd_cv_free_pyramid(p); return 0; } }
            /* The source level already carries its 21-px reflect-101 border, which
             * is exactly reflect(2x+i-2) for every tap the 5-tap row filter uses. */
            for(int y=0;y<ah;y++) {
                const uint8_t *s=r+(size_t)y*as; uint16_t *o=hbuf+(size_t)y*w;
                int x=0;
#ifdef RD_SSE2
                const __m128i m00ff=_mm_set1_epi16(0x00ff),six=_mm_set1_epi16(6);
                for(;x+8<=w;x+=8) {
                    __m128i u=_mm_loadu_si128((const __m128i*)(s+2*x-2)),u2=_mm_loadu_si128((const __m128i*)(s+2*x+14));
                    __m128i e0=_mm_and_si128(u,m00ff),o0=_mm_srli_epi16(u,8),e1=_mm_and_si128(u2,m00ff),o1=_mm_srli_epi16(u2,8);
                    __m128i t2=_mm_or_si128(_mm_srli_si128(e0,2),_mm_slli_si128(e1,14));
                    __m128i t3=_mm_or_si128(_mm_srli_si128(o0,2),_mm_slli_si128(o1,14));
                    __m128i t4=_mm_or_si128(_mm_srli_si128(e0,4),_mm_slli_si128(e1,12));
                    __m128i sum=_mm_add_epi16(_mm_add_epi16(e0,t4),_mm_add_epi16(_mm_slli_epi16(_mm_add_epi16(o0,t3),2),_mm_mullo_epi16(t2,six)));
                    _mm_storeu_si128((__m128i*)(o+x),sum);
                }
#endif
                for(;x<w;x++) o[x]=(uint16_t)(s[2*x-2]+4*s[2*x-1]+6*s[2*x]+4*s[2*x+1]+s[2*x+2]);
            }
            for(int y=0;y<h;y++) {
                const uint16_t *h0=hbuf+(size_t)reflect(2*y-2,ah)*w,*h1=hbuf+(size_t)reflect(2*y-1,ah)*w,
                               *h2=hbuf+(size_t)reflect(2*y,ah)*w,*h3=hbuf+(size_t)reflect(2*y+1,ah)*w,
                               *h4=hbuf+(size_t)reflect(2*y+2,ah)*w;
                uint8_t *o=roi+(size_t)y*S;
                int x=0;
#ifdef RD_SSE2
                const __m128i six=_mm_set1_epi16(6),r128=_mm_set1_epi16(128);
                for(;x+16<=w;x+=16) {
                    __m128i res[2];
                    for(int k=0;k<2;k++) {
                        int xx=x+8*k;
                        __m128i a0=_mm_loadu_si128((const __m128i*)(h0+xx)),a1=_mm_loadu_si128((const __m128i*)(h1+xx)),a2=_mm_loadu_si128((const __m128i*)(h2+xx)),
                                a3=_mm_loadu_si128((const __m128i*)(h3+xx)),a4=_mm_loadu_si128((const __m128i*)(h4+xx));
                        /* max 16*255*16+128 < 2^16: unsigned 16-bit arithmetic is exact */
                        __m128i sum=_mm_add_epi16(_mm_add_epi16(_mm_add_epi16(a0,a4),r128),_mm_add_epi16(_mm_slli_epi16(_mm_add_epi16(a1,a3),2),_mm_mullo_epi16(a2,six)));
                        res[k]=_mm_srli_epi16(sum,8);
                    }
                    _mm_storeu_si128((__m128i*)(o+x),_mm_packus_epi16(res[0],res[1]));
                }
#endif
                for(;x<w;x++) o[x]=(uint8_t)((h0[x]+4*h1[x]+6*h2[x]+4*h3[x]+h4[x]+128)>>8);
            }
        }
        /* reflect-101 border, 21 px: side strips per ROI row, then whole rows */
        {
            int lx[21],rx[21];
            for(int i=1;i<=21;i++) { lx[i-1]=reflect(-i,w); rx[i-1]=reflect(w-1+i,w); }
            for(int y=0;y<h;y++) {
                uint8_t *r=roi+(size_t)y*S;
                for(int i=1;i<=21;i++) { r[-i]=r[lx[i-1]]; r[w-1+i]=r[rx[i-1]]; }
            }
            for(int i=1;i<=21;i++) {
                memcpy(roi+(size_t)(-i)*S-21,roi+(size_t)reflect(-i,h)*S-21,(size_t)S);
                memcpy(roi+(size_t)(h-1+i)*S-21,roi+(size_t)reflect(h-1+i,h)*S-21,(size_t)S);
            }
        }
        /* Scharr: the padded image already holds the reflect-101 neighbours */
        for(int y=0;y<h;y++) {
            const uint8_t *rm=roi+(size_t)(y-1)*S,*r0=roi+(size_t)y*S,*rp=roi+(size_t)(y+1)*S;
            int16_t *d=q->deriv+2*((size_t)(y+21)*S+21);
            int x=0;
#ifdef RD_SSE2
            const __m128i Z=_mm_setzero_si128(),c3=_mm_set1_epi16(3),c10=_mm_set1_epi16(10);
#define L8(p) _mm_unpacklo_epi8(_mm_loadl_epi64((const __m128i*)(p)),Z)
            for(;x+8<=w;x+=8) {   /* |values| <= 4080: exact in int16 */
                __m128i m0=L8(rm+x-1),m1=L8(rm+x),m2=L8(rm+x+1),c0=L8(r0+x-1),c2=L8(r0+x+1),p0=L8(rp+x-1),p1=L8(rp+x),p2=L8(rp+x+1);
                __m128i dx=_mm_add_epi16(_mm_add_epi16(_mm_mullo_epi16(_mm_sub_epi16(m2,m0),c3),_mm_mullo_epi16(_mm_sub_epi16(c2,c0),c10)),_mm_mullo_epi16(_mm_sub_epi16(p2,p0),c3));
                __m128i dy=_mm_add_epi16(_mm_add_epi16(_mm_mullo_epi16(_mm_sub_epi16(p0,m0),c3),_mm_mullo_epi16(_mm_sub_epi16(p1,m1),c10)),_mm_mullo_epi16(_mm_sub_epi16(p2,m2),c3));
                _mm_storeu_si128((__m128i*)(d+2*x),_mm_unpacklo_epi16(dx,dy));
                _mm_storeu_si128((__m128i*)(d+2*x+8),_mm_unpackhi_epi16(dx,dy));
            }
#undef L8
#endif
            for(;x<w;x++) {
                int dx=3*(rm[x+1]-rm[x-1])+10*(r0[x+1]-r0[x-1])+3*(rp[x+1]-rp[x-1]);
                int dy=3*(rp[x-1]-rm[x-1])+10*(rp[x]-rm[x])+3*(rp[x+1]-rm[x+1]);
                d[2*x]=(int16_t)dx; d[2*x+1]=(int16_t)dy;
            }
        }
        p->count=l+1;
        w=(w+1)/2; h=(h+1)/2;
        if(w<=21||h<=21)break;
    }
    free(hbuf);
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
