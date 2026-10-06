/* SPDX-License-Identifier: MIT
 * Original implementation of PNG and RFC 1950/1951; no zlib/libpng code copied.
 * Grayscale/gamma arithmetic follows the observed OpenCV/libpng contract.
 * Copyright (c) 2026 pose-validation contributors. See ../reference_png/LICENSE.
 */
#include "ok_png.h"
#include <stdint.h>
#include <limits.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#ifndef OK_PNG_MAX_BYTES
#define OK_PNG_MAX_BYTES ((size_t)256 * 1024 * 1024)
#endif
#ifndef OK_PNG_MAX_PIXELS
#define OK_PNG_MAX_PIXELS ((size_t)64 * 1024 * 1024)
#endif
#ifndef OK_PNG_CHECK_CRC
#define OK_PNG_CHECK_CRC 1
#endif

static uint32_t be32(const unsigned char *p)
{ return (uint32_t)p[0]<<24 | (uint32_t)p[1]<<16 | (uint32_t)p[2]<<8 | p[3]; }
static unsigned be16(const unsigned char *p) { return (unsigned)p[0]<<8 | p[1]; }

/* RFC 1951 canonical codes. A 9-bit prefix table covers the common case;
 * longer codes use the canonical first-code/count representation. */
typedef struct {
    unsigned short fast[512], count[16], first[16], offset[16], symbol[288];
} Huff;
typedef struct {
    const unsigned char *p;
    size_t n, pos;
    uint32_t value;
    unsigned bits;
    int bad;
} Bits;
static void fill(Bits *b, unsigned n)
{
    while (b->bits < n && b->pos < b->n) {
        b->value |= (uint32_t)b->p[b->pos++] << b->bits;
        b->bits += 8;
    }
}
static unsigned take(Bits *b, unsigned n)
{
    unsigned v;
    fill(b, n);
    if (b->bits < n) { b->bad = 1; return 0; }
    v = b->value & ((1u << n)-1);
    b->value >>= n;
    b->bits -= n;
    return v;
}
static int tree(Huff *h, const unsigned char *len, unsigned n, int complete)
{
    unsigned i, k, code = 0, off = 0, next[16], total = 0;
    int left = 1;
    memset(h, 0, sizeof(*h));
    for (i = 0; i < n; ++i) {
        if (len[i] > 15) return 0;
        if (len[i]) { ++h->count[len[i]]; ++total; }
    }
    for (k = 1; k <= 15; ++k) {
        left = 2*left - h->count[k];
        if (left < 0) return 0;
        code = (code + h->count[k-1]) << 1;
        h->first[k] = (unsigned short)code;
        h->offset[k] = (unsigned short)off;
        next[k] = off;
        off += h->count[k];
    }
    if (left && (complete || (total && !(total == 1 && h->count[1] == 1)))) return 0;
    for (i = 0; i < n; ++i) if (len[i]) h->symbol[next[len[i]]++] = (unsigned short)i;
    for (k = 1; k <= 9; ++k) for (i = 0; i < h->count[k]; ++i) {
        unsigned c = h->first[k]+i, rev = 0, j;
        for (j = 0; j < k; ++j) { rev = (rev << 1) | (c & 1); c >>= 1; }
        for (j = rev; j < 512; j += 1u << k)
            h->fast[j] = (unsigned short)((k << 9) | h->symbol[h->offset[k]+i]);
    }
    return 1;
}
static int symbol(Bits *b, const Huff *h)
{
    unsigned f, code = 0, k;
    fill(b, 9);
    f = h->fast[b->value & 511];
    if (f && (f >> 9) <= b->bits) {
        b->value >>= f >> 9;
        b->bits -= f >> 9;
        return (int)(f & 511);
    }
    for (k = 1; k <= 15; ++k) {
        code = (code << 1) | take(b, 1);
        if (b->bad) return -1;
        if (code >= h->first[k] && code-h->first[k] < h->count[k])
            return h->symbol[h->offset[k]+code-h->first[k]];
    }
    b->bad = 1;
    return -1;
}
static int inflate(const unsigned char *src, size_t n, unsigned char *out, size_t cap)
{
    static const unsigned short lb[29] = {3,4,5,6,7,8,9,10,11,13,15,17,19,23,27,31,35,43,51,59,67,83,99,115,131,163,195,227,258};
    static const unsigned char le[29] = {0,0,0,0,0,0,0,0,1,1,1,1,2,2,2,2,3,3,3,3,4,4,4,4,5,5,5,5,0};
    static const unsigned short db[30] = {1,2,3,4,5,7,9,13,17,25,33,49,65,97,129,193,257,385,513,769,1025,1537,2049,3073,4097,6145,8193,12289,16385,24577};
    static const unsigned char de[30] = {0,0,0,0,1,1,2,2,3,3,4,4,5,5,6,6,7,7,8,8,9,9,10,10,11,11,12,12,13,13};
    static const unsigned char order[19] = {16,17,18,0,8,7,9,6,10,5,11,4,12,3,13,2,14,1,15};
    Bits b;
    Huff lit, dist, cl;
    size_t used = 0, i;
    unsigned final = 0;
    uint32_t a = 1, s = 0;
    if (n < 6 || (src[0]&15) != 8 || (src[0]>>4) > 7 ||
        (((unsigned)src[0]<<8) + src[1])%31 || (src[1]&32)) return 0;
    memset(&b, 0, sizeof(b)); b.p = src+2; b.n = n-6;
    while (!final && !b.bad) {
        unsigned type;
        final = take(&b, 1); type = take(&b, 2);
        if (type == 0) {
            unsigned len, inv;
            take(&b, b.bits%8);
            len = take(&b, 16); inv = take(&b, 16);
            if (b.bad || (len ^ inv) != 65535 || len > cap-used) return 0;
            for (i = 0; i < len; ++i) out[used++] = (unsigned char)take(&b, 8);
            continue;
        }
        if (type == 3) return 0;
        if (type == 1) {
            unsigned char l[288], d[32];
            for (i = 0; i < 288; ++i) l[i] = (unsigned char)(i < 144 ? 8 : i < 256 ? 9 : i < 280 ? 7 : 8);
            memset(d, 5, sizeof(d));
            if (!tree(&lit, l, 288, 0) || !tree(&dist, d, 32, 0)) return 0;
        } else {
            unsigned nl = take(&b, 5)+257, nd = take(&b, 5)+1, nc = take(&b, 4)+4;
            unsigned char c[19] = {0}, lengths[318];
            unsigned j = 0;
            if (nl > 286 || nd > 32) return 0;
            for (i = 0; i < nc; ++i) c[order[i]] = (unsigned char)take(&b, 3);
            if (b.bad || !tree(&cl, c, 19, 1)) return 0;
            while (j < nl+nd) {
                int v = symbol(&b, &cl);
                unsigned repeat, value;
                if (v < 0) return 0;
                if (v <= 15) { lengths[j++] = (unsigned char)v; continue; }
                if (v == 16) {
                    if (!j) return 0;
                    repeat = take(&b, 2)+3; value = lengths[j-1];
                } else { repeat = take(&b, v == 17 ? 3 : 7)+(v == 17 ? 3 : 11); value = 0; }
                if (b.bad || repeat > nl+nd-j) return 0;
                while (repeat--) lengths[j++] = (unsigned char)value;
            }
            if (!lengths[256] || !tree(&lit, lengths, nl, 0) || !tree(&dist, lengths+nl, nd, 0)) return 0;
        }
        for (;;) {
            int v = symbol(&b, &lit), d;
            unsigned len, distance;
            if (v < 0) return 0;
            if (v < 256) {
                if (used == cap) return 0;
                out[used++] = (unsigned char)v;
                continue;
            }
            if (v == 256) break;
            if (v > 285) return 0;
            len = lb[v-257] + take(&b, le[v-257]);
            d = symbol(&b, &dist);
            if (d < 0 || d > 29) return 0;
            distance = db[d] + take(&b, de[d]);
            if (b.bad || distance > used || distance > (1u << ((src[0]>>4)+8)) || len > cap-used) return 0;
            for (i = 0; i < len; ++i) { out[used] = out[used-distance]; ++used; }
        }
    }
    if (b.bad || used != cap || b.pos-b.bits/8 != b.n) return 0;
    /* Adler-32, modulo in bounded batches. */
    for (i = 0; i < cap;) {
        size_t end = cap-i > 5552 ? i+5552 : cap;
        for (; i < end; ++i) { a += out[i]; s += a; }
        a %= 65521; s %= 65521;
    }
    return ((s << 16) | a) == be32(src+n-4);
}

static uint32_t crc(const unsigned char *p, size_t n)
{
    uint32_t tab[256], c = UINT32_MAX;
    unsigned i, j;
    for (i = 0; i < 256; ++i) {
        uint32_t v = i;
        for (j = 0; j < 8; ++j) v = (v >> 1) ^ (0xedb88320u & (0u-(v&1)));
        tab[i] = v;
    }
    while (n--) c = tab[(c ^ *p++)&255] ^ (c >> 8);
    return c ^ UINT32_MAX;
}
static int paeth(int a, int b, int c)
{
    int p = a+b-c, da = abs(p-a), db = abs(p-b), dc = abs(p-c);
    return da <= db && da <= dc ? a : db <= dc ? b : c;
}

/* pow(x,g) for 0<=x<=1 and g>0, without libm. Range reduction plus
 * log's atanh series and exp's Taylor series; only used to build gamma LUTs.
 * No approximate per-pixel arithmetic. IEEE double LUT rounding is tested
 * against libpng; this is not a general-purpose correctly-rounded pow(). */
static double power(double x, double g)
{
    const double ln2 = 0.69314718055994530942;
    double z, zz, term, sum, y, r;
    int e = 0, k, i;
    if (x <= 0) return 0;
    if (x >= 1) return 1;
    while (x < 0.70710678118654752440) { x *= 2; --e; }
    z = (x-1)/(x+1); zz = z*z; term = z; sum = z;
    for (i = 3; i <= 31; i += 2) { term *= zz; sum += term/i; }
    y = (2*sum + e*ln2)*g;
    if (y < -750) return 0;
    k = (int)(y/ln2); r = y-k*ln2;
    term = sum = 1;
    for (i = 1; i <= 20; ++i) { term *= r/i; sum += term; }
    while (k++ < 0) sum *= 0.5;
    return sum;
}
static int significant(unsigned g) { return g < 95000 || g > 105000; }
static unsigned reciprocal(unsigned g)
{
    double v = 1e10/g + .5;
    return v <= INT32_MAX ? (unsigned)v : 0;
}
typedef struct {
    unsigned short to[2048], from[2048], same[2048];
    unsigned shift;
    int active;
} Gamma;
static void gamma_init(Gamma *t, unsigned gamma, unsigned depth, unsigned sig)
{
    unsigned screen, inv, back, count, max, i;
    t->active = 0; t->shift = 0;
    if (!gamma || gamma > INT32_MAX) return;
    screen = reciprocal(gamma);
    if (!screen || (!significant(gamma) && !significant(screen))) return;
    inv = screen; back = reciprocal(screen);
    if (!back) return;
    t->active = 1;
    if (depth == 16) {
        t->shift = sig && sig < 16 ? 16-sig : 0;
        if (t->shift < 5) t->shift = 5;
        if (t->shift > 8) t->shift = 8;
    }
    count = depth == 16 ? 1u << (16-t->shift) : 256;
    max = depth == 16 ? 65535 : 255;
    for (i = 0; i < count; ++i) {
        double x = i*(1.0/(count-1));
        t->to[i] = (unsigned short)(max*(significant(inv) ? power(x, inv*.00001) : x)+.5);
        t->from[i] = (unsigned short)(max*(significant(back) ? power(x, back*.00001) : x)+.5);
        t->same[i] = (unsigned short)i;
    }
    if (depth == 16) {
        unsigned last = 0, product = (unsigned)(gamma*1e-5*screen+.5);
        for (i = 0; i < 255; ++i) {
            unsigned bound = (unsigned)(65535*power((i*257+128)/65535.0, product*.00001)+.5);
            bound = (bound*(count-1)+32768)/65535+1;
            while (last < bound && last < count) t->same[last++] = (unsigned short)(i*257);
        }
        while (last < count) t->same[last++] = 65535;
    }
}
static unsigned gray(unsigned r, unsigned g, unsigned b, unsigned depth, const Gamma *t)
{
    unsigned v;
    if (t->active) {
        if (r == g && r == b) v = t->same[r >> t->shift];
        else {
            v = (9797u*t->to[r >> t->shift] + 19234u*t->to[g >> t->shift] + 3737u*t->to[b >> t->shift] + 16384) >> 15;
            v = t->from[v >> t->shift];
        }
    } else v = (9797u*r + 19234u*g + 3737u*b + (depth == 16 ? 16384u : 0u)) >> 15;
    return depth == 16 ? v >> 8 : v;
}
static unsigned sample(const unsigned char *p, size_t i, unsigned depth)
{
    if (depth == 8) return p[i];
    if (depth == 16) return be16(p+2*i);
    return (p[i*depth/8] >> (8-depth-(i*depth%8))) & ((1u<<depth)-1);
}
/* TIFF IFD0 orientation in PNG eXIf. OpenCV applies this after decoding. */
static uint32_t exnum(const unsigned char *p, int le, int bytes)
{
    uint32_t v = 0;
    int i;
    for (i = 0; i < bytes; ++i) v = (v << 8) | p[le ? bytes-1-i : i];
    return v;
}
static unsigned orientation(const unsigned char *p, size_t n)
{
    size_t off, i, count;
    int le;
    if (n < 8 || !((p[0]=='I' && p[1]=='I') || (p[0]=='M' && p[1]=='M'))) return 1;
    le = p[0]=='I';
    if (exnum(p+2, le, 2) != 42) return 1;
    off = exnum(p+4, le, 4);
    if (off > n-2) return 1;
    count = exnum(p+off, le, 2); off += 2;
    if (count > (n-off)/12) return 1;
    for (i = 0; i < count; ++i, off += 12) {
        if (exnum(p+off, le, 2)==274 && exnum(p+off+2, le, 2)==3 && exnum(p+off+4, le, 4)==1) {
            unsigned v = exnum(p+off+8, le, 2);
            return v >= 1 && v <= 8 ? v : 1;
        }
    }
    return 1;
}

int ok_png_decode_gray(const unsigned char *buf, size_t n, unsigned char **out, int *w, int *h)
{
    static const unsigned char signature[8] = {137,80,78,71,13,10,26,10};
    static const unsigned char passes[7][4] = {{0,0,8,8},{4,0,8,8},{0,4,4,8},{2,0,4,4},{0,2,2,4},{1,0,2,2},{0,1,1,2}};
    unsigned width=0, height=0, depth=0, type=0, interlace=0, channels=0;
    unsigned palette_n=0, gamma=0, sig=0, orient=1, srgb=0;
    unsigned char palette[768], *idat=NULL, *raw=NULL, *pixels=NULL;
    size_t pos=8, compressed=0, allocated=0, raw_n=0, npix=0, rp=0;
    int header=0, ended=0, seen_idat=0, closed_idat=0, err=OK_PNG_INVALID;
    unsigned pass;
    Gamma gt;
    if (out) *out=NULL;
    if (w) *w=0;
    if (h) *h=0;
    if (!out || !w || !h || !buf || n<8 || memcmp(buf, signature, 8)) return err;
    if (n>OK_PNG_MAX_BYTES) return OK_PNG_LIMIT;
    while (pos <= n && n-pos >= 12) {
        size_t len = be32(buf+pos);
        const unsigned char *tag=buf+pos+4, *p=buf+pos+8;
        unsigned j;
        if (len>n-pos-12 || len>INT32_MAX) goto done;
        for (j=0;j<4;++j) if (!((tag[j]>='A'&&tag[j]<='Z') || (tag[j]>='a'&&tag[j]<='z'))) goto done;
        if (tag[2]&32) goto done;
        if (OK_PNG_CHECK_CRC && crc(tag,len+4)!=be32(p+len)) goto done;
        if (!header && memcmp(tag,"IHDR",4)) goto done;
        if (!memcmp(tag,"IHDR",4)) {
            if (header || len!=13) goto done;
            header=1; width=be32(p); height=be32(p+4); depth=p[8]; type=p[9]; interlace=p[12];
            if (!width || !height || width>INT_MAX || height>INT_MAX || p[10] || p[11] || interlace>1) goto done;
            if (type==0) { channels=1; if (!(depth==1||depth==2||depth==4||depth==8||depth==16)) goto done; }
            else if (type==3) { channels=1; if (!(depth==1||depth==2||depth==4||depth==8)) goto done; }
            else if (type==2||type==4||type==6) { channels=type==2?3:type==4?2:4; if (!(depth==8||depth==16)) goto done; }
            else goto done;
            if (width>OK_PNG_MAX_PIXELS/height) { err=OK_PNG_LIMIT; goto done; }
            npix=(size_t)width*height;
        } else if (!memcmp(tag,"PLTE",4)) {
            if (seen_idat || palette_n || !len || len>768 || len%3 || type==0 || type==4) goto done;
            palette_n=(unsigned)(len/3);
            if (type==3 && palette_n>(1u<<depth)) goto done;
            memcpy(palette,p,len);
        } else if (!memcmp(tag,"IDAT",4)) {
            size_t need;
            unsigned char *next;
            if (closed_idat || (type==3&&!palette_n)) goto done;
            seen_idat=1;
            if (len>OK_PNG_MAX_BYTES-compressed) { err=OK_PNG_LIMIT; goto done; }
            need=compressed+len;
            if (need>allocated) {
                size_t cap=allocated?allocated:4096;
                while (cap<need) cap=cap>OK_PNG_MAX_BYTES/2?OK_PNG_MAX_BYTES:cap*2;
                next=(unsigned char*)realloc(idat,cap);
                if (!next) { err=OK_PNG_NOMEM; goto done; }
                idat=next; allocated=cap;
            }
            if (len) memcpy(idat+compressed,p,len);
            compressed=need;
        } else if (!memcmp(tag,"IEND",4)) {
            if (len || !seen_idat) goto done;
            ended=1; break;
        } else {
            if (!(tag[0]&32)) goto done; /* unknown critical chunk */
            if (!memcmp(tag,"gAMA",4) && !seen_idat && len==4 && !srgb) gamma=be32(p);
            if (!memcmp(tag,"sRGB",4) && !seen_idat && len==1 && p[0]<=3) { srgb=1; gamma=45455; }
            if (!memcmp(tag,"sBIT",4) && !seen_idat && (type==2||type==6) && len==channels) {
                sig=p[0]; if (p[1]>sig) sig=p[1]; if(p[2]>sig) sig=p[2];
                if (!p[0]||!p[1]||!p[2]||sig>depth) sig=0;
            }
            if (!memcmp(tag,"eXIf",4)) orient=orientation(p,len);
        }
        if (seen_idat && memcmp(tag,"IDAT",4)) closed_idat=1;
        pos+=len+12;
    }
    if (!ended) goto done;
    for (pass=0;pass<(interlace?7u:1u);++pass) {
        unsigned x0=interlace?passes[pass][0]:0, y0=interlace?passes[pass][1]:0;
        unsigned dx=interlace?passes[pass][2]:1, dy=interlace?passes[pass][3]:1;
        size_t pw=width>x0?1+(width-1-x0)/dx:0, ph=height>y0?1+(height-1-y0)/dy:0;
        size_t row;
        if (!pw||!ph) continue;
        if (pw>(SIZE_MAX-7)/(channels*depth)) { err=OK_PNG_LIMIT; goto done; }
        row=(pw*channels*depth+7)/8+1;
        if (row>(OK_PNG_MAX_BYTES-raw_n)/ph) { err=OK_PNG_LIMIT; goto done; }
        raw_n+=row*ph;
    }
    raw=(unsigned char*)malloc(raw_n); pixels=(unsigned char*)malloc(npix);
    if (!raw||!pixels) { err=OK_PNG_NOMEM; goto done; }
    if (!inflate(idat,compressed,raw,raw_n)) goto done;
    gamma_init(&gt,gamma,type==3?8:depth,sig);
    for (pass=0;pass<(interlace?7u:1u);++pass) {
        unsigned x0=interlace?passes[pass][0]:0, y0=interlace?passes[pass][1]:0;
        unsigned dx=interlace?passes[pass][2]:1, dy=interlace?passes[pass][3]:1;
        size_t pw=width>x0?1+(width-1-x0)/dx:0, ph=height>y0?1+(height-1-y0)/dy:0;
        size_t row=(pw*channels*depth+7)/8, y, x, bpp=(channels*depth+7)/8;
        unsigned char *prev=NULL;
        if (!pw||!ph) continue;
        for (y=0;y<ph;++y) {
            unsigned filter=raw[rp++];
            unsigned char *cur=raw+rp;
            if (filter>4) goto done;
            for (x=0;x<row;++x) {
                int a=x>=bpp?cur[x-bpp]:0, b=prev?prev[x]:0, c=prev&&x>=bpp?prev[x-bpp]:0;
                int add=filter==0?0:filter==1?a:filter==2?b:filter==3?(a+b)/2:paeth(a,b,c);
                cur[x]=(unsigned char)(cur[x]+add);
            }
            for (x=0;x<pw;++x) {
                unsigned v=sample(cur,x*channels,depth);
                if (type==3) {
                    const unsigned char *p;
                    if (v>=palette_n) goto done;
                    p=palette+3*v; v=gray(p[0],p[1],p[2],8,&gt);
                } else if (type==2||type==6) v=gray(v,sample(cur,x*channels+1,depth),sample(cur,x*channels+2,depth),depth,&gt);
                else if (depth==16) v>>=8;
                else if (depth<8) v=v*255/((1u<<depth)-1);
                pixels[(y0+y*dy)*(size_t)width+x0+x*dx]=(unsigned char)v;
            }
            prev=cur; rp+=row;
        }
    }
    if (orient>1) {
        unsigned char *rot=(unsigned char*)malloc(npix);
        size_t x,y,ow=orient>=5?height:width;
        if (!rot) { err=OK_PNG_NOMEM; goto done; }
        for (y=0;y<height;++y) for (x=0;x<width;++x) {
            size_t xx=x, yy=y;
            switch(orient) {
            case 2: xx=width-1-x; break;
            case 3: xx=width-1-x; yy=height-1-y; break;
            case 4: yy=height-1-y; break;
            case 5: xx=y; yy=x; break;
            case 6: xx=height-1-y; yy=x; break;
            case 7: xx=height-1-y; yy=width-1-x; break;
            case 8: xx=y; yy=width-1-x; break;
            }
            rot[yy*ow+xx]=pixels[y*(size_t)width+x];
        }
        free(pixels); pixels=rot;
        if (orient>=5) { unsigned tmp=width; width=height; height=tmp; }
    }
    *out=pixels; pixels=NULL; *w=(int)width; *h=(int)height; err=OK_PNG_OK;
done:
    free(idat); free(raw); free(pixels);
    return err;
}
int ok_png_read_gray(const char *path, unsigned char **out, int *w, int *h)
{
    FILE *f;
    long end;
    unsigned char *buf;
    size_t n;
    int err;
    if (out) *out=NULL;
    if (w) *w=0;
    if (h) *h=0;
    if (!path||!out||!w||!h) return OK_PNG_INVALID;
    f=fopen(path,"rb");
    if (!f) return OK_PNG_IO;
    if (fseek(f,0,SEEK_END) || (end=ftell(f))<0 || fseek(f,0,SEEK_SET)) { fclose(f); return OK_PNG_IO; }
    if ((unsigned long)end>OK_PNG_MAX_BYTES) { fclose(f); return OK_PNG_LIMIT; }
    n=(size_t)end; buf=(unsigned char*)malloc(n?n:1);
    if (!buf) { fclose(f); return OK_PNG_NOMEM; }
    if (fread(buf,1,n,f)!=n) { free(buf); fclose(f); return OK_PNG_IO; }
    if (fclose(f)) { free(buf); return OK_PNG_IO; }
    err=ok_png_decode_gray(buf,n,out,w,h); free(buf); return err;
}
