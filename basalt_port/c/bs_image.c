/* SPDX-License-Identifier: BSD-3-Clause
 * Basalt module M4, see bs_image.h. Ports of basalt-headers image/image_pyr.h (subsample, ManagedImagePyr), basalt
 * dataset_io_euroc.h (get_image_data) and src/utils/keypoints.cpp (detectKeypoints), BSD-3-Clause, (c) 2019 Usenko, Demmel. */
#include "bs_image.h"
#include "bs_fast.h"
#include "../../okvis_port/c/ok_png.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/* ------------------------------------------------------------------ loader */

static uint32_t be32(const unsigned char *p) { return ((uint32_t)p[0] << 24) | ((uint32_t)p[1] << 16) | ((uint32_t)p[2] << 8) | p[3]; }


/* ---- fast path for the one PNG flavour EuRoC uses (8-bit gray, no interlace, chunks IHDR / IDAT.. / IEND only, valid CRC / Adler-32).
 * Same pixels as ok_png_decode_gray for such files; anything else (or any error) returns 0 and the caller falls back to ok_png_decode_gray. */
typedef struct { unsigned short fast[1024], count[16], first[16], offset[16], symbol[288]; } fhuff;
typedef struct { const unsigned char *p; size_t n, pos; uint64_t v; unsigned bits; } fbits;

static void fb_fill(fbits *b)
{
#if defined(__BYTE_ORDER__) && defined(__ORDER_LITTLE_ENDIAN__) && __BYTE_ORDER__ == __ORDER_LITTLE_ENDIAN__
    if (b->pos + 8 <= b->n) {   /* whole 8-byte load, keep the bytes that fit (bits |= 56) */
        uint64_t w;
        memcpy(&w, b->p + b->pos, 8);
        b->v |= w << b->bits;
        b->pos += (63 - b->bits) >> 3;
        b->bits |= 56;
        return;
    }
#endif
    while (b->bits <= 56) {
        b->v |= (uint64_t)(b->pos < b->n ? b->p[b->pos] : 0) << b->bits;   /* zero padding past the end; overrun is checked at the end */
        b->pos++;
        b->bits += 8;
    }
}
static unsigned fb_take(fbits *b, unsigned n)
{
    unsigned r;
    if (b->bits < n) fb_fill(b);
    r = (unsigned)(b->v & ((1u << n) - 1u));
    b->v >>= n; b->bits -= n;
    return r;
}
static int fh_build(fhuff *h, const unsigned char *len, unsigned n)
{
    unsigned i, k, code = 0, off = 0, next[16];
    int left = 1;
    memset(h, 0, sizeof(*h));
    for (i = 0; i < n; ++i) { if (len[i] > 15) return 0; if (len[i]) ++h->count[len[i]]; }
    for (k = 1; k <= 15; ++k) {
        left = 2 * left - h->count[k];
        if (left < 0) return 0;
        code = (code + h->count[k - 1]) << 1;
        h->first[k] = (unsigned short)code; h->offset[k] = (unsigned short)off; next[k] = off;
        off += h->count[k];
    }
    for (i = 0; i < n; ++i) if (len[i]) h->symbol[next[len[i]]++] = (unsigned short)i;
    for (k = 1; k <= 10; ++k) for (i = 0; i < h->count[k]; ++i) {
        unsigned c = h->first[k] + i, rev = 0, j;
        for (j = 0; j < k; ++j) { rev = (rev << 1) | (c & 1); c >>= 1; }
        for (j = rev; j < 1024; j += 1u << k) h->fast[j] = (unsigned short)((k << 9) | h->symbol[h->offset[k] + i]);
    }
    return 1;
}
static int fh_sym(fbits *b, const fhuff *h)
{
    unsigned f, code = 0, k;
    if (b->bits < 15) fb_fill(b);
    f = h->fast[b->v & 1023];
    if (f) { b->v >>= f >> 9; b->bits -= f >> 9; return (int)(f & 511); }
    for (k = 1; k <= 15; ++k) {
        code = (code << 1) | (unsigned)((b->v >> (k - 1)) & 1);
        if (code >= h->first[k] && code - h->first[k] < h->count[k]) {
            b->v >>= k; b->bits -= k;
            return h->symbol[h->offset[k] + code - h->first[k]];
        }
    }
    return -1;
}
static int fast_inflate(const unsigned char *src, size_t n, unsigned char *out, size_t cap)
{
    static const unsigned short lb[29] = {3,4,5,6,7,8,9,10,11,13,15,17,19,23,27,31,35,43,51,59,67,83,99,115,131,163,195,227,258};
    static const unsigned char le[29] = {0,0,0,0,0,0,0,0,1,1,1,1,2,2,2,2,3,3,3,3,4,4,4,4,5,5,5,5,0};
    static const unsigned short db[30] = {1,2,3,4,5,7,9,13,17,25,33,49,65,97,129,193,257,385,513,769,1025,1537,2049,3073,4097,6145,8193,12289,16385,24577};
    static const unsigned char de[30] = {0,0,0,0,1,1,2,2,3,3,4,4,5,5,6,6,7,7,8,8,9,9,10,10,11,11,12,12,13,13};
    static const unsigned char order[19] = {16,17,18,0,8,7,9,6,10,5,11,4,12,3,13,2,14,1,15};
    fbits b;
    fhuff lit, dist, cl;
    size_t used = 0, i;
    unsigned final = 0;
    uint32_t a = 1, s = 0;
    if (n < 6 || (src[0] & 15) != 8 || (src[0] >> 4) > 7 || (((unsigned)src[0] << 8) + src[1]) % 31 || (src[1] & 32)) return 0;
    memset(&b, 0, sizeof(b)); b.p = src + 2; b.n = n - 6;
    while (!final) {
        unsigned type;
        if (b.pos > b.n + 8) return 0;
        final = fb_take(&b, 1); type = fb_take(&b, 2);
        if (type == 3) return 0;
        if (type == 0) {
            unsigned len, inv;
            fb_take(&b, b.bits % 8);
            len = fb_take(&b, 16); inv = fb_take(&b, 16);
            if ((len ^ inv) != 65535 || len > cap - used) return 0;
            b.pos -= b.bits / 8; b.v = 0; b.bits = 0;   /* give the prefetched whole bytes back, then copy the stored bytes */
            if (len > (b.pos <= b.n ? b.n - b.pos : 0)) return 0;
            memcpy(out + used, b.p + b.pos, len); used += len; b.pos += len;
            continue;
        }
        if (type == 1) {
            unsigned char l[288], d[32];
            for (i = 0; i < 288; ++i) l[i] = (unsigned char)(i < 144 ? 8 : i < 256 ? 9 : i < 280 ? 7 : 8);
            memset(d, 5, sizeof(d));
            if (!fh_build(&lit, l, 288) || !fh_build(&dist, d, 32)) return 0;
        } else {
            unsigned nl = fb_take(&b, 5) + 257, nd = fb_take(&b, 5) + 1, nc = fb_take(&b, 4) + 4, j = 0;
            unsigned char c[19] = {0}, lengths[318];
            if (nl > 286 || nd > 32) return 0;
            for (i = 0; i < nc; ++i) c[order[i]] = (unsigned char)fb_take(&b, 3);
            if (!fh_build(&cl, c, 19)) return 0;
            while (j < nl + nd) {
                int v = fh_sym(&b, &cl);
                unsigned repeat, value;
                if (v < 0) return 0;
                if (v <= 15) { lengths[j++] = (unsigned char)v; continue; }
                if (v == 16) { if (!j) return 0; repeat = fb_take(&b, 2) + 3; value = lengths[j - 1]; }
                else { repeat = fb_take(&b, v == 17 ? 3 : 7) + (v == 17 ? 3 : 11); value = 0; }
                if (repeat > nl + nd - j) return 0;
                while (repeat--) lengths[j++] = (unsigned char)value;
            }
            if (!lengths[256] || !fh_build(&lit, lengths, nl) || !fh_build(&dist, lengths + nl, nd)) return 0;
        }
        for (;;) {
            int v, d;
            unsigned len, distance;
#if defined(__BYTE_ORDER__) && defined(__ORDER_LITTLE_ENDIAN__) && __BYTE_ORDER__ == __ORDER_LITTLE_ENDIAN__
            {   /* run of literals with the bit reservoir in registers (stores through unsigned char* would otherwise force reloads of b) */
                uint64_t rv = b.v;
                unsigned rbits = b.bits;
                size_t rpos = b.pos;
                unsigned char *o = out + used, *oend = out + cap;
                const unsigned char *rp = b.p;
                const size_t rn = b.n;
                while (oend - o >= 4 && rpos + 8 <= rn) {
                    unsigned f;
                    uint64_t w;
                    memcpy(&w, rp + rpos, 8);
                    rv |= w << rbits; rpos += (63 - rbits) >> 3; rbits |= 56;
                    f = lit.fast[rv & 1023];
                    if (!f || (f & 511) >= 256) break;
                    rv >>= f >> 9; rbits -= f >> 9; *o++ = (unsigned char)f;
                    f = lit.fast[rv & 1023];
                    if (!f || (f & 511) >= 256) break;
                    rv >>= f >> 9; rbits -= f >> 9; *o++ = (unsigned char)f;
                    f = lit.fast[rv & 1023];
                    if (!f || (f & 511) >= 256) break;
                    rv >>= f >> 9; rbits -= f >> 9; *o++ = (unsigned char)f;
                }
                b.v = rv; b.bits = rbits; b.pos = rpos; used = (size_t)(o - out);
            }
#endif
            v = fh_sym(&b, &lit);
            if (v < 0) return 0;
            if (v < 256) {
                if (used == cap) return 0;
                out[used++] = (unsigned char)v;
                continue;
            }
            if (v == 256) break;
            if (v > 285) return 0;
            len = lb[v - 257] + fb_take(&b, le[v - 257]);
            d = fh_sym(&b, &dist);
            if (d < 0 || d > 29) return 0;
            distance = db[d] + fb_take(&b, de[d]);
            if (distance > used || len > cap - used) return 0;
            if (distance >= len) { memcpy(out + used, out + used - distance, len); used += len; }
            else for (i = 0; i < len; ++i) { out[used] = out[used - distance]; ++used; }
        }
    }
    /* the stream must end exactly before the Adler-32 (as ok_png: consumed bytes == n - 6 once the unread whole bytes are given back) */
    if (used != cap || b.pos - b.bits / 8 != b.n) return 0;
    for (i = 0; i < cap;) {
        size_t end = cap - i > 5552 ? i + 5552 : cap;
        for (; i < end; ++i) { a += out[i]; s += a; }
        a %= 65521; s %= 65521;
    }
    return ((s << 16) | a) == be32(src + n - 4);
}
static uint32_t fast_crc(const unsigned char *p, size_t n)
{
    static uint32_t tab[8][256];   /* slicing-by-8, same CRC-32 */
    static int init;
    uint32_t c = 0xffffffffu;
    if (!init) {
        unsigned i, j;
        for (i = 0; i < 256; ++i) { uint32_t v = i; for (j = 0; j < 8; ++j) v = (v >> 1) ^ (0xedb88320u & (0u - (v & 1))); tab[0][i] = v; }
        for (i = 0; i < 256; ++i) for (j = 1; j < 8; ++j) tab[j][i] = (tab[j - 1][i] >> 8) ^ tab[0][tab[j - 1][i] & 255];
        init = 1;
    }
    while (n >= 8) {
        uint32_t lo = c ^ ((uint32_t)p[0] | (uint32_t)p[1] << 8 | (uint32_t)p[2] << 16 | (uint32_t)p[3] << 24);
        c = tab[7][lo & 255] ^ tab[6][(lo >> 8) & 255] ^ tab[5][(lo >> 16) & 255] ^ tab[4][lo >> 24] ^
            tab[3][p[4]] ^ tab[2][p[5]] ^ tab[1][p[6]] ^ tab[0][p[7]];
        p += 8; n -= 8;
    }
    while (n--) c = tab[0][(c ^ *p++) & 255] ^ (c >> 8);
    return c ^ 0xffffffffu;
}
static int fast_png_gray(const unsigned char *buf, size_t n, unsigned char **out, int *w, int *h)
{
    size_t pos = 8, comp = 0, cap, x, y;
    unsigned width = 0, height = 0;
    int have_ihdr = 0, ended = 0, closed = 0;
    unsigned char *idat = NULL, *raw = NULL;
    *out = NULL;
    if (n > (size_t)256 * 1024 * 1024) return 0;
    idat = (unsigned char *)malloc(n);
    if (!idat) return 0;
    while (pos + 12 <= n) {
        size_t len = be32(buf + pos);
        const unsigned char *tag = buf + pos + 4, *p = buf + pos + 8;
        if (len > n - pos - 12) goto fail;
        if (fast_crc(tag, len + 4) != be32(p + len)) goto fail;
        if (!have_ihdr) {
            if (memcmp(tag, "IHDR", 4) || len != 13 || p[8] != 8 || p[9] != 0 || p[10] || p[11] || p[12]) goto fail;
            width = be32(p); height = be32(p + 4);
            if (!width || !height || width > 65536 || height > 65536) goto fail;
            have_ihdr = 1;
        } else if (!memcmp(tag, "IDAT", 4)) {
            if (closed) goto fail;
            memcpy(idat + comp, p, len); comp += len;
        } else if (!memcmp(tag, "IEND", 4)) {
            if (len || !comp) goto fail;
            ended = 1; break;
        } else goto fail;
        if (comp && memcmp(tag, "IDAT", 4)) closed = 1;
        pos += len + 12;
    }
    if (!ended) goto fail;
    cap = ((size_t)width + 1) * height;
    raw = (unsigned char *)malloc(cap);
    if (!raw || !fast_inflate(idat, comp, raw, cap)) goto fail;
    {   /* unfilter (bpp = 1) in place into a packed width x height image */
        unsigned char *pix = (unsigned char *)malloc((size_t)width * height);
        if (!pix) goto fail;
        for (y = 0; y < height; ++y) {
            const unsigned char *src = raw + y * ((size_t)width + 1);
            unsigned char *cur = pix + y * (size_t)width;
            const unsigned char *prev = y ? cur - width : NULL;
            unsigned filter = src[0];
            src++;
            if (filter > 4) { free(pix); goto fail; }
            if (filter == 0 || !prev) {
                if (filter == 0 || filter == 2) memcpy(cur, src, width);
                else if (filter == 1 || filter == 4) { cur[0] = src[0]; for (x = 1; x < width; ++x) cur[x] = (unsigned char)(src[x] + cur[x - 1]); }
                else { cur[0] = src[0]; for (x = 1; x < width; ++x) cur[x] = (unsigned char)(src[x] + (cur[x - 1] >> 1)); }
            } else if (filter == 1) {
                cur[0] = src[0]; for (x = 1; x < width; ++x) cur[x] = (unsigned char)(src[x] + cur[x - 1]);
            } else if (filter == 2) {
                for (x = 0; x < width; ++x) cur[x] = (unsigned char)(src[x] + prev[x]);
            } else if (filter == 3) {
                int a = (unsigned char)(src[0] + (prev[0] >> 1));   /* a = left pixel kept in a register */
                cur[0] = (unsigned char)a;
                for (x = 1; x < width; ++x) { a = (unsigned char)(src[x] + ((a + prev[x]) >> 1)); cur[x] = (unsigned char)a; }
            } else {
                int a = (unsigned char)(src[0] + prev[0]);   /* a = c = 0: paeth picks b */
                cur[0] = (unsigned char)a;
                for (x = 1; x < width; ++x) {
                    const int b_ = prev[x], c_ = prev[x - 1], d = b_ - c_, sd = d >> 31, pa = (d ^ sd) - sd;
                    const int t = a - c_, st = t >> 31, pb = (t ^ st) - st, u = t + d, su = u >> 31, pc = (u ^ su) - su;   /* p - a = d, p - b = t, p - c = t + d */
                    const int m1 = -(int)((pa <= pb) & (pa <= pc)), m2 = -(int)(pb <= pc);   /* branch-free select: a, else b, else c */
                    a = (unsigned char)(src[x] + ((a & m1) | (~m1 & ((b_ & m2) | (c_ & ~m2)))));
                    cur[x] = (unsigned char)a;
                }
            }
        }
        free(idat); free(raw);
        *out = pix; *w = (int)width; *h = (int)height;
        return 1;
    }
fail:
    free(idat); free(raw);
    return 0;
}

int bs_image_decode_euroc(const unsigned char *buf, size_t n, uint16_t **out, int *w, int *h)
{
    static const unsigned char sig[8] = {137, 80, 78, 71, 13, 10, 26, 10};
    size_t pos = 8;
    int got_ihdr = 0, rc, x, y;
    unsigned char *g = NULL;
    uint16_t *o;
    *out = NULL; *w = *h = 0;
    if (n < 33 || memcmp(buf, sig, 8) != 0) return BS_IMG_DECODE;
    while (pos + 12 <= n) {
        uint32_t len = be32(buf + pos);
        const unsigned char *t = buf + pos + 4;
        if ((size_t)len > n - pos - 12) break;
        if (!memcmp(t, "IHDR", 4)) {
            if (len != 13) return BS_IMG_DECODE;
            if (t[12] != 8 || t[13] != 0) return BS_IMG_UNSUPPORTED;   /* bit depth 8, colour type 0 (gray) */
            got_ihdr = 1;
        } else if (!memcmp(t, "gAMA", 4) || !memcmp(t, "sRGB", 4) || !memcmp(t, "iCCP", 4) || !memcmp(t, "sBIT", 4) || !memcmp(t, "tRNS", 4)) {
            return BS_IMG_UNSUPPORTED;
        } else if (!memcmp(t, "IDAT", 4)) break;
        pos += 12 + (size_t)len;
    }
    if (!got_ihdr) return BS_IMG_DECODE;
    if (!fast_png_gray(buf, n, &g, w, h)) {
        rc = ok_png_decode_gray(buf, n, &g, w, h);
        if (rc != OK_PNG_OK) return rc == OK_PNG_NOMEM ? BS_IMG_NOMEM : BS_IMG_DECODE;
    }
    o = (uint16_t *)malloc((size_t)*w * (size_t)*h * sizeof(uint16_t));
    if (!o) { free(g); *w = *h = 0; return BS_IMG_NOMEM; }
    for (y = 0; y < *h; y++)
        for (x = 0; x < *w; x++) o[(size_t)y * (size_t)*w + x] = (uint16_t)((int)g[(size_t)y * (size_t)*w + x] << 8);
    free(g);
    *out = o;
    return BS_IMG_OK;
}

int bs_image_load_euroc(const char *path, uint16_t **out, int *w, int *h)
{
    FILE *f = fopen(path, "rb");
    unsigned char *b;
    long sz;
    int rc;
    *out = NULL; *w = *h = 0;
    if (!f) return BS_IMG_IO;
    if (fseek(f, 0, SEEK_END) != 0 || (sz = ftell(f)) < 0 || fseek(f, 0, SEEK_SET) != 0) { fclose(f); return BS_IMG_IO; }
    b = (unsigned char *)malloc((size_t)sz + 1);
    if (!b) { fclose(f); return BS_IMG_NOMEM; }
    if (fread(b, 1, (size_t)sz, f) != (size_t)sz) { free(b); fclose(f); return BS_IMG_IO; }
    fclose(f);
    rc = bs_image_decode_euroc(b, (size_t)sz, out, w, h);
    free(b);
    return rc;
}

/* ----------------------------------------------------------------- pyramid */

static int iabs(int a) { return a < 0 ? -a : a; }
static int border101(int x, int h) { return h - 1 - iabs(h - 1 - x); }

void bs_subsample(const uint16_t *src, int sp, int w, int h, uint16_t *dst, int dp, int sub_w, int sub_h)
{
    static const int k[5] = {1, 4, 6, 4, 1};
    int r, c;
    /* tmp(r, c) of basalt is a (sub_h x w) image stored transposed: element (x = r, y = c) at tmp[c * sub_h + r] */
    int *tmp = (int *)malloc((size_t)sub_h * (size_t)w * sizeof(int));
    if (!tmp) abort();
    for (r = 0; r < sub_h; r++) {
        const uint16_t *m2 = src + (size_t)iabs(2 * r - 2) * (size_t)sp;
        const uint16_t *m1 = src + (size_t)iabs(2 * r - 1) * (size_t)sp;
        const uint16_t *r0 = src + (size_t)(2 * r) * (size_t)sp;
        const uint16_t *p1 = src + (size_t)border101(2 * r + 1, h) * (size_t)sp;
        const uint16_t *p2 = src + (size_t)border101(2 * r + 2, h) * (size_t)sp;
        for (c = 0; c < w; c++)
            tmp[(size_t)c * (size_t)sub_h + (size_t)r] = k[0] * (int)m2[c] + k[1] * (int)m1[c] + k[2] * (int)r0[c] + k[3] * (int)p1[c] + k[4] * (int)p2[c];
    }
    for (c = 0; c < sub_w; c++) {
        const int *m2 = tmp + (size_t)iabs(2 * c - 2) * (size_t)sub_h;
        const int *m1 = tmp + (size_t)iabs(2 * c - 1) * (size_t)sub_h;
        const int *r0 = tmp + (size_t)(2 * c) * (size_t)sub_h;
        const int *p1 = tmp + (size_t)border101(2 * c + 1, w) * (size_t)sub_h;
        const int *p2 = tmp + (size_t)border101(2 * c + 2, w) * (size_t)sub_h;
        for (r = 0; r < sub_h; r++) {
            int v = k[0] * m2[r] + k[1] * m1[r] + k[2] * r0[r] + k[3] * p1[r] + k[4] * p2[r];
            dst[(size_t)r * (size_t)dp + (size_t)c] = (uint16_t)((v + (1 << 7)) >> 8);
        }
    }
    free(tmp);
}

static uint16_t *lvl_ptr(const bs_pyr *p, int l, int *w, int *h)
{
    size_t x = (l == 0) ? 0 : (size_t)p->orig_w;
    size_t y = (l <= 1) ? 0 : (size_t)(p->h - (p->h >> (l - 1)));
    *w = p->orig_w >> l;
    *h = p->h >> l;
    return p->data + y * (size_t)p->pitch + x;
}

const uint16_t *bs_pyr_lvl(const bs_pyr *p, int l, int *w, int *h) { return lvl_ptr(p, l, w, h); }

int bs_pyr_set(bs_pyr *p, const uint16_t *img, int w, int h, int levels)
{
    int i, y;
    free(p->data);
    p->data = NULL;
    p->orig_w = w; p->h = h; p->levels = levels; p->pitch = w + w / 2;
    p->data = (uint16_t *)calloc((size_t)p->pitch * (size_t)h, sizeof(uint16_t));
    if (!p->data) return BS_IMG_NOMEM;
    for (y = 0; y < h; y++) memcpy(p->data + (size_t)y * (size_t)p->pitch, img + (size_t)y * (size_t)w, (size_t)w * sizeof(uint16_t));
    for (i = 0; i < levels; i++) {
        int sw, sh, dw, dh;
        const uint16_t *s = lvl_ptr(p, i, &sw, &sh);
        uint16_t *d = lvl_ptr(p, i + 1, &dw, &dh);
        bs_subsample(s, p->pitch, sw, sh, d, p->pitch, dw, dh);
    }
    return BS_IMG_OK;
}

void bs_pyr_free(bs_pyr *p) { free(p->data); memset(p, 0, sizeof(*p)); }

/* ---------------------------------------------------------------- keypoints */

#define EDGE_THRESHOLD 19

int bs_detect_keypoints(const uint16_t *img, int pitch, int w, int h, int grid, int num_points_cell,
                        const double *cur, int n_cur, double *out, int max_out)
{
    const size_t P = (size_t)grid;
    int nout = 0, i;
    if (grid < 1 || w < grid || h < grid) return -2;
    const size_t x_start = ((size_t)w % P) / 2;
    const size_t x_stop = x_start + P * ((size_t)w / P - 1);
    const size_t y_start = ((size_t)h % P) / 2;
    const size_t y_stop = y_start + P * ((size_t)h / P - 1);
    const size_t ncx = (size_t)w / P + 1, ncy = (size_t)h / P + 1;
    int *cells = (int *)calloc(ncx * ncy, sizeof(int));
    uint8_t *sub = (uint8_t *)malloc(P * P);
    bs_fast_kp *kps = (bs_fast_kp *)malloc(P * P * sizeof(bs_fast_kp));
    if (!cells || !sub || !kps) { free(cells); free(sub); free(kps); return -3; }

    for (i = 0; i < n_cur; i++) {
        double px = cur[2 * i], py = cur[2 * i + 1];
        if (px >= (double)x_start && py >= (double)y_start && px < (double)(x_stop + P) && py < (double)(y_stop + P)) {
            int x = (int)((px - (double)x_start) / (double)grid);
            int y = (int)((py - (double)y_start) / (double)grid);
            cells[(size_t)y * ncx + (size_t)x] += 1;
        }
    }

    for (size_t x = x_start; x <= x_stop; x += P) {
        for (size_t y = y_start; y <= y_stop; y += P) {
            if (cells[((y - y_start) / P) * ncx + (x - x_start) / P] > 0) continue;
            for (size_t yy = 0; yy < P; yy++)
                for (size_t xx = 0; xx < P; xx++) sub[yy * P + xx] = (uint8_t)(img[(y + yy) * (size_t)pitch + x + xx] >> 8);

            int points_added = 0, threshold = 40;
            while (points_added < num_points_cell && threshold >= 5) {
                int n = bs_fast9_16(sub, grid, grid, grid, threshold, 1, kps, grid * grid);
                if (n < 0) { free(cells); free(sub); free(kps); return -3; }
                bs_fast_sort_response_desc(kps, (size_t)n);
                for (int k = 0; k < n && points_added < num_points_cell; k++) {
                    float fx = (float)x + kps[k].x, fy = (float)y + kps[k].y;
                    const float border = (float)EDGE_THRESHOLD;
                    /* Image::InBounds(float, float, float): border <= x && x < (w - border - 1) && ... (all float) */
                    if (border <= fx && fx < ((float)w - border - 1.0f) && border <= fy && fy < ((float)h - border - 1.0f)) {
                        if (nout >= max_out) { free(cells); free(sub); free(kps); return -1; }
                        out[2 * nout] = (double)fx;
                        out[2 * nout + 1] = (double)fy;
                        nout++;
                        points_added++;
                    }
                }
                threshold /= 2;
            }
        }
    }
    free(cells); free(sub); free(kps);
    return nout;
}
