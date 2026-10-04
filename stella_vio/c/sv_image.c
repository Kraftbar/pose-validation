/* SV_PORT_SOURCES: sv_image.c
 * Modified from OpenCV to C99 scalar specializations; fastAtan2 is Apache-2.0.
 * See ../NOTICE and ../LICENSES/ for all upstream notices.
 * SPDX-License-Identifier: Apache-2.0 AND BSD-3-Clause
 *
 * Ports of OpenCV 4.6.0 modules/imgproc/src/resize.cpp (INTER_LINEAR 8U
 * fixed-point path), modules/imgproc/src/smooth.dispatch.cpp
 * (getGaussianKernelBitExact / getGaussianKernelFixedPoint_ED formulas,
 * reimplemented with plain IEEE double instead of OpenCV's softdouble --
 * see note in sv_image.h) and its ufixedpoint16-based GaussianBlurFixedPoint
 * separable convolution (reimplemented with plain int32, which is exactly
 * what that fixed-point type does under the hood: Q8 x Q8 -> Q16 with
 * round-to-nearest at the end), and modules/core/src/mathfuncs_core.simd.hpp
 * (fastAtan2 scalar path, atan_f32).
 *
 * -----------------------------------------------------------------------
 *                           License Agreement
 *                For Open Source Computer Vision Library
 *                        (3-clause BSD License)
 *
 * Copyright (C) 2000-2020, Intel Corporation, all rights reserved.
 * Copyright (C) 2009-2011, Willow Garage Inc., all rights reserved.
 * Copyright (C) 2013, OpenCV Foundation, all rights reserved.
 * Third party copyrights are property of their respective owners.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *   * Redistributions of source code must retain the above copyright notice,
 *     this list of conditions and the following disclaimer.
 *
 *   * Redistributions in binary form must reproduce the above copyright
 *     notice, this list of conditions and the following disclaimer in the
 *     documentation and/or other materials provided with the distribution.
 *
 *   * Neither the names of the copyright holders nor the names of the
 *     contributors may be used to endorse or promote products derived from
 *     this software without specific prior written permission.
 *
 * This software is provided by the copyright holders and contributors "as
 * is" and any express or implied warranties, including, but not limited to,
 * the implied warranties of merchantability and fitness for a particular
 * purpose are disclaimed. In no event shall copyright holders or
 * contributors be liable for any direct, indirect, incidental, special,
 * exemplary, or consequential damages (including, but not limited to,
 * procurement of substitute goods or services; loss of use, data, or
 * profits; or business interruption) however caused and on any theory of
 * liability, whether in contract, strict liability, or tort (including
 * negligence or otherwise) arising in any way out of the use of this
 * software, even if advised of the possibility of such damage.
 * -----------------------------------------------------------------------
 */
#include "sv_image.h"
#include <math.h>
#include <stdlib.h>
#include <string.h>

static int sv_cv_floor_f(float v) {
    int i = (int)v;
    return i - (i > v);
}

static long long sv_cv_round_d(double v) {
    return (long long)nearbyint(v);
}

static short sv_saturate_i16(int v) {
    if (v < -32768) return -32768;
    if (v > 32767) return 32767;
    return (short)v;
}

/* ---- resize: INTER_LINEAR, CV_8UC1, matches modules/imgproc/src/resize.cpp
 * hal::resize()'s fixpt path (HResizeLinear / VResizeLinear /
 * FixedPtCast<int,uchar,22>). ---- */

static int sv_clip(int x, int a, int b) {
    return x >= a ? (x < b ? x : b - 1) : a;
}

void sv_resize_linear_u8(const uint8_t* src, int src_step, int src_w, int src_h,
                          uint8_t* dst, int dst_w, int dst_h) {
    const int INTER_RESIZE_COEF_BITS = 11;
    const int INTER_RESIZE_COEF_SCALE = 1 << INTER_RESIZE_COEF_BITS; /* 2048 */
    const int ksize2 = 1;

    double inv_scale_x = (double)dst_w / (double)src_w;
    double inv_scale_y = (double)dst_h / (double)src_h;
    double scale_x = 1.0 / inv_scale_x;
    double scale_y = 1.0 / inv_scale_y;

    int* xofs = (int*)malloc(sizeof(int) * (size_t)dst_w);
    short* ialpha = (short*)malloc(sizeof(short) * (size_t)dst_w * 2);
    int* yofs = (int*)malloc(sizeof(int) * (size_t)dst_h);
    short* ibeta = (short*)malloc(sizeof(short) * (size_t)dst_h * 2);

    int xmax = dst_w;
    int dx, dy;
    for (dx = 0; dx < dst_w; dx++) {
        float fx = (float)((dx + 0.5) * scale_x - 0.5);
        int sx = sv_cv_floor_f(fx);
        fx -= (float)sx;
        if (sx < ksize2 - 1) {
            if (sx < 0) { fx = 0.f; sx = 0; }
        }
        if (sx + ksize2 >= src_w) {
            xmax = xmax < dx ? xmax : dx;
            if (sx >= src_w - 1) { fx = 0.f; sx = src_w - 1; }
        }
        xofs[dx] = sx;
        float cbuf0 = 1.f - fx, cbuf1 = fx;
        ialpha[dx * 2 + 0] = sv_saturate_i16(sv_cv_round_f(cbuf0 * (float)INTER_RESIZE_COEF_SCALE));
        ialpha[dx * 2 + 1] = sv_saturate_i16(sv_cv_round_f(cbuf1 * (float)INTER_RESIZE_COEF_SCALE));
    }
    for (dy = 0; dy < dst_h; dy++) {
        float fy = (float)((dy + 0.5) * scale_y - 0.5);
        int sy = sv_cv_floor_f(fy);
        fy -= (float)sy;
        if (sy < ksize2 - 1) {
            if (sy < 0) { fy = 0.f; sy = 0; }
        }
        if (sy + ksize2 >= src_h) {
            if (sy >= src_h - 1) { fy = 0.f; sy = src_h - 1; }
        }
        yofs[dy] = sy;
        float cbuf0 = 1.f - fy, cbuf1 = fy;
        ibeta[dy * 2 + 0] = sv_saturate_i16(sv_cv_round_f(cbuf0 * (float)INTER_RESIZE_COEF_SCALE));
        ibeta[dy * 2 + 1] = sv_saturate_i16(sv_cv_round_f(cbuf1 * (float)INTER_RESIZE_COEF_SCALE));
    }

    for (dy = 0; dy < dst_h; dy++) {
        int sy0 = yofs[dy];
        int sy_k0 = sv_clip(sy0 + 0, 0, src_h);
        int sy_k1 = sv_clip(sy0 + 1, 0, src_h);
        const uint8_t* S0 = src + (size_t)sy_k0 * src_step;
        const uint8_t* S1 = src + (size_t)sy_k1 * src_step;
        short b0 = ibeta[dy * 2 + 0], b1 = ibeta[dy * 2 + 1];
        uint8_t* D = dst + (size_t)dy * dst_w;

        /* The CV_8U full specialization of VResizeLinear (resize.cpp,
         * VResizeLinear<uchar,int,short,FixedPtCast<...>,VResizeLinearVec_32s8u>)
         * does NOT use the generic FixedPtCast((a*b0+b*b1+DELTA)>>22)
         * formula anywhere, including its own scalar tail loop -- every
         * column, SIMD-covered or not, uses the lossy
         * uchar(((b0*(S0>>4))>>16) + ((b1*(S1>>4))>>16) + 2)>>2) formula
         * (an extra >>4 truncation before the multiply that the generic
         * template doesn't have), with a plain narrowing (uchar) cast --
         * not saturate_cast. Reproduced verbatim, uniformly, no boundary
         * split needed. The horizontal results (row0 / row1 of HResizeLinear)
         * are formed per column right where the vertical formula consumes them. */
        for (dx = 0; dx < xmax; dx++) {
            int sx = xofs[dx];
            int h0 = S0[sx] * ialpha[dx * 2 + 0] + S0[sx + 1] * ialpha[dx * 2 + 1];
            int h1 = S1[sx] * ialpha[dx * 2 + 0] + S1[sx + 1] * ialpha[dx * 2 + 1];
            int m0 = ((int)b0 * (h0 >> 4)) >> 16;
            int m1 = ((int)b1 * (h1 >> 4)) >> 16;
            D[dx] = (uint8_t)((m0 + m1 + 2) >> 2);
        }
        for (; dx < dst_w; dx++) {
            int sx = xofs[dx];
            int h0 = S0[sx] * INTER_RESIZE_COEF_SCALE;
            int h1 = S1[sx] * INTER_RESIZE_COEF_SCALE;
            int m0 = ((int)b0 * (h0 >> 4)) >> 16;
            int m1 = ((int)b1 * (h1 >> 4)) >> 16;
            D[dx] = (uint8_t)((m0 + m1 + 2) >> 2);
        }
    }

    free(xofs); free(ialpha); free(yofs); free(ibeta);
}

/* ---- GaussianBlur, ksize=7, sigma=2, BORDER_REFLECT_101, CV_8UC1 ---- */

static void sv_gaussian_kernel7(double sigma, double w[7]) {
    /* getGaussianKernelBitExact, n=7, sigma>0 branch (softdouble ->
     * plain double; see sv_image.h note on why this is expected to match
     * OpenCV's softdouble exp() bit-for-bit for these small, well-scaled
     * arguments). */
    double sigmaX = sigma;
    double scale2X = -0.5 * 0.25 / (sigmaX * sigmaX);
    int n2_ = 3; /* (7-1)/2 */
    double values[3];
    double sum = 0.0;
    int i, x;
    for (i = 0, x = 1 - 7; i < n2_; i++, x += 2) {
        double t = exp((double)(x * x) * scale2X);
        values[i] = t;
        sum += t;
    }
    sum *= 2.0;
    sum += 1.0; /* center, x=0 -> exp(0)=1 */

    double mul1 = 1.0 / sum;
    for (i = 0; i < n2_; i++) {
        double t = values[i] * mul1;
        w[i] = t;
        w[6 - i] = t;
    }
    w[3] = 1.0 * mul1;
}

static void sv_gaussian_kernel7_fixed(const double w[7], int fixed[7]) {
    /* getGaussianKernelFixedPoint_ED, fractionBits=8 (ufixedpoint16). */
    const long long fractionMultiplier = 256;
    int n2_ = 3; /* n/2 for odd n=7 */
    double err = 0.0;
    long long sum = 0;
    int i;
    for (i = 0; i < n2_; i++) {
        double adj_v = w[i] * (double)fractionMultiplier + err;
        long long v0 = sv_cv_round_d(adj_v);
        err = adj_v - (double)v0;
        fixed[i] = (int)v0;
        fixed[6 - i] = (int)v0;
        sum += v0;
    }
    sum *= 2;
    long long v_center = fractionMultiplier - sum;
    fixed[3] = (int)v_center;
}

static int sv_reflect101(int idx, int n) {
    if (n == 1) return 0;
    while (idx < 0 || idx >= n) {
        if (idx < 0) idx = -idx;
        if (idx >= n) idx = 2 * (n - 1) - idx;
    }
    return idx;
}

void sv_gaussian_blur7_u8(const uint8_t* src, int step, int w, int h, uint8_t* dst) {
    double kd[7];
    int kfix[7];
    sv_gaussian_kernel7(2.0, kd);
    sv_gaussian_kernel7_fixed(kd, kfix);
    const int k0 = kfix[0], k1 = kfix[1], k2 = kfix[2], k3 = kfix[3];

    /* Everything below is exact integer arithmetic, so the order of the sums
     * does not matter. Kernel weights are >= 0 and sum to 256 (Q8). */

    /* Horizontal pass -> Q8 intermediate (<= 255*256, fits uint16). Each row is
     * first copied into a padded line (3 BORDER_REFLECT_101 pixels each side)
     * so the inner loop needs no border tests. */
    uint16_t* rowbuf = (uint16_t*)malloc(sizeof(uint16_t) * (size_t)w * (size_t)h);
    uint8_t* line = (uint8_t*)malloc((size_t)w + 6);
    int x, y;
    for (y = 0; y < h; y++) {
        const uint8_t* srow = src + (size_t)y * step;
        uint16_t* orow = rowbuf + (size_t)y * w;
        for (x = 0; x < w; x++) line[x + 3] = srow[x];
        for (x = 1; x <= 3; x++) {
            line[3 - x] = srow[sv_reflect101(-x, w)];
            line[w + 2 + x] = srow[sv_reflect101(w - 1 + x, w)];
        }
        for (x = 0; x < w; x++) {
            const uint8_t* p = line + x;
            orow[x] = (uint16_t)(k0 * (p[0] + p[6]) + k1 * (p[1] + p[5]) + k2 * (p[2] + p[4]) + k3 * p[3]);
        }
    }
    free(line);

    /* Vertical pass: Q8 * Q8 -> Q16, round to nearest (the sum is <= 255 * 65536 so
     * the result never needs saturation). */
    for (y = 0; y < h; y++) {
        uint8_t* drow = dst + (size_t)y * w;
        const uint16_t* r0 = rowbuf + (size_t)sv_reflect101(y - 3, h) * w;
        const uint16_t* r1 = rowbuf + (size_t)sv_reflect101(y - 2, h) * w;
        const uint16_t* r2 = rowbuf + (size_t)sv_reflect101(y - 1, h) * w;
        const uint16_t* r3 = rowbuf + (size_t)y * w;
        const uint16_t* r4 = rowbuf + (size_t)sv_reflect101(y + 1, h) * w;
        const uint16_t* r5 = rowbuf + (size_t)sv_reflect101(y + 2, h) * w;
        const uint16_t* r6 = rowbuf + (size_t)sv_reflect101(y + 3, h) * w;
        for (x = 0; x < w; x++) {
            int acc = k0 * (r0[x] + r6[x]) + k1 * (r1[x] + r5[x]) + k2 * (r2[x] + r4[x]) + k3 * r3[x];
            drow[x] = (uint8_t)((acc + (1 << 15)) >> 16);
        }
    }

    free(rowbuf);
}

/* ---- fastAtan2: modules/core/src/mathfuncs_core.simd.hpp atan_f32 ---- */

#define SV_CV_PI 3.14159265358979311600 /* CV_PI, matches OpenCV's own definition */

float sv_fast_atan2(float y, float x) {
    static const float atan2_p1 = 0.9997878412794807f * (float)(180.0 / SV_CV_PI);
    static const float atan2_p3 = -0.3258083974640975f * (float)(180.0 / SV_CV_PI);
    static const float atan2_p5 = 0.1555786518463281f * (float)(180.0 / SV_CV_PI);
    static const float atan2_p7 = -0.04432655554792128f * (float)(180.0 / SV_CV_PI);
    const float eps = 2.2204460492503131e-16f; /* (float)DBL_EPSILON */

    float ax = fabsf(x), ay = fabsf(y);
    float a, c, c2;
    if (ax >= ay) {
        c = ay / (ax + eps);
        c2 = c * c;
        a = (((atan2_p7 * c2 + atan2_p5) * c2 + atan2_p3) * c2 + atan2_p1) * c;
    }
    else {
        c = ax / (ay + eps);
        c2 = c * c;
        a = 90.f - (((atan2_p7 * c2 + atan2_p5) * c2 + atan2_p3) * c2 + atan2_p1) * c;
    }
    if (x < 0) a = 180.f - a;
    if (y < 0) a = 360.f - a;
    return a;
}
