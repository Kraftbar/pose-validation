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
    rc = ok_png_decode_gray(buf, n, &g, w, h);
    if (rc != OK_PNG_OK) return rc == OK_PNG_NOMEM ? BS_IMG_NOMEM : BS_IMG_DECODE;
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
