/* SV_PORT_SOURCES: sv_fast.c
 * SPDX-License-Identifier: BSD-3-Clause
 *
 * Port of OpenCV 4.6.0 modules/features2d/src/fast.cpp (FAST_t<16>, scalar
 * path) and fast_score.cpp (makeOffsets, cornerScore<16>).
 *
 * -----------------------------------------------------------------------
 * This is FAST corner detector, contributed to OpenCV by the author,
 * Edward Rosten. Below is the original copyright and the references.
 *
 * Copyright (c) 2006, 2008 Edward Rosten
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 *     *Redistributions of source code must retain the above copyright
 *      notice, this list of conditions and the following disclaimer.
 *
 *     *Redistributions in binary form must reproduce the above copyright
 *      notice, this list of conditions and the following disclaimer in the
 *      documentation and/or other materials provided with the distribution.
 *
 *     *Neither the name of the University of Cambridge nor the names of
 *      its contributors may be used to endorse or promote products derived
 *      from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 * A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
 * PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
 * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
 * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 * The rest (OpenCV's FAST_t<16>/cornerScore<16>/makeOffsets wrapper code
 * around the above algorithm) is BSD-3, "Open Source Computer Vision
 * Library" license, reproduced in sv_undistort.c / README notes for this
 * directory.
 * -----------------------------------------------------------------------
 */
#include "sv_fast.h"
#include <stdlib.h>

static void sv_make_offsets16(int pixel[25], int row_stride) {
    static const int offsets16[16][2] = {
        {0, 3}, {1, 3}, {2, 2}, {3, 1}, {3, 0}, {3, -1}, {2, -2}, {1, -3}, {0, -3}, {-1, -3}, {-2, -2}, {-3, -1}, {-3, 0}, {-3, 1}, {-2, 2}, {-1, 3}};
    int k;
    for (k = 0; k < 16; k++) {
        pixel[k] = offsets16[k][0] + offsets16[k][1] * row_stride;
    }
    for (; k < 25; k++) {
        pixel[k] = pixel[k - 16];
    }
}

static int sv_min_i(int a, int b) { return a < b ? a : b; }
static int sv_max_i(int a, int b) { return a > b ? a : b; }

/* cornerScore<16>: modules/features2d/src/fast_score.cpp, non-SIMD path
 * (bit-identical to the SIMD path -- pure int16 min/max, no rounding). */
/* `dark` = 1 when the corner's run is the dark one (d > threshold), 0 for the bright one. Only one side can beat
 * the threshold (two arcs of 9 on the 16-ring always share a pixel, which cannot be both darker and brighter than
 * v by more than threshold), so the loop of the other side is a no-op: the a-loop leaves a0 = threshold when there
 * is no dark run, and once a0 > threshold exists the b-loop cannot lower b0 below -a0. Skipping it is exact. */
static int sv_corner_score16(const uint8_t* ptr, const int pixel[25], int threshold, int dark) {
    const int N = 25;
    int k, v = ptr[0];
    short d[25];
    for (k = 0; k < N; k++) {
        d[k] = (short)(v - ptr[pixel[k]]);
    }

    int a0 = threshold;
    for (k = 0; dark && k < 16; k += 2) {
        int a = sv_min_i((int)d[k + 1], (int)d[k + 2]);
        a = sv_min_i(a, (int)d[k + 3]);
        if (a <= a0) {
            continue;
        }
        a = sv_min_i(a, (int)d[k + 4]);
        a = sv_min_i(a, (int)d[k + 5]);
        a = sv_min_i(a, (int)d[k + 6]);
        a = sv_min_i(a, (int)d[k + 7]);
        a = sv_min_i(a, (int)d[k + 8]);
        a0 = sv_max_i(a0, sv_min_i(a, (int)d[k]));
        a0 = sv_max_i(a0, sv_min_i(a, (int)d[k + 9]));
    }

    int b0 = -a0;
    for (k = 0; !dark && k < 16; k += 2) {
        int b = sv_max_i((int)d[k + 1], (int)d[k + 2]);
        b = sv_max_i(b, (int)d[k + 3]);
        b = sv_max_i(b, (int)d[k + 4]);
        b = sv_max_i(b, (int)d[k + 5]);
        if (b >= b0) {
            continue;
        }
        b = sv_max_i(b, (int)d[k + 6]);
        b = sv_max_i(b, (int)d[k + 7]);
        b = sv_max_i(b, (int)d[k + 8]);
        b0 = sv_min_i(b0, sv_max_i(b, (int)d[k]));
        b0 = sv_min_i(b0, sv_max_i(b, (int)d[k + 9]));
    }

    return -b0 - 1;
}

/* 1 iff the 16-bit circular mask has a cyclic run of >= 9 set bits */
static int sv_has_run9(unsigned int m) {
    unsigned int r = m | (m << 16); /* unrolled twice: runs across bit 15 -> 0 become plain runs */
    r &= r >> 1; /* run >= 2 starts here */
    r &= r >> 2; /* >= 4 */
    r &= r >> 4; /* >= 8 */
    r &= (m | (m << 16)) >> 8; /* >= 9 */
    return r != 0;
}

int sv_fast_detect(const uint8_t* img, int step, int cols, int rows,
                    int threshold, sv_keypoint* out_keypts, int cap) {
    int pixel[25];
    int i, j, k;
    int n_out = 0;

    sv_make_offsets16(pixel, step);

    if (threshold < 0) threshold = 0;
    if (threshold > 255) threshold = 255;

    if (rows < 7 || cols < 7) {
        return 0;
    }

    /* K = 8 (patternSize / 2): a corner needs more than K consecutive circle pixels; see sv_has_run9 */

    unsigned char threshold_tab[512];
    for (i = -255; i <= 255; i++) {
        threshold_tab[i + 255] = (unsigned char)(i < -threshold ? 1 : i > threshold ? 2 : 0);
    }

    unsigned char* buf[3];
    int* cpbuf_storage[3]; /* cpbuf[idx] = cpbuf_storage[idx]+1, so [-1] is valid */
    for (i = 0; i < 3; i++) {
        buf[i] = (unsigned char*)calloc((size_t)cols, 1);
        cpbuf_storage[i] = (int*)calloc((size_t)cols + 1, sizeof(int));
    }
    int* cpbuf[3];
    for (i = 0; i < 3; i++) {
        cpbuf[i] = cpbuf_storage[i] + 1;
    }

    for (i = 3; i < rows - 2; i++) {
        const uint8_t* ptr = img + (size_t)i * step + 3;
        unsigned char* curr = buf[(i - 3) % 3];
        int* cornerpos = cpbuf[(i - 3) % 3];
        int ncorners = 0;
        for (j = 0; j < cols; j++) curr[j] = 0;

        if (i < rows - 3) {
            for (j = 3; j < cols - 3; j++, ptr++) {
                int v = ptr[0];
                const unsigned char* tab = threshold_tab - v + 255;
                int d = tab[ptr[pixel[0]]] | tab[ptr[pixel[8]]];
                if (d == 0) continue;
                d &= tab[ptr[pixel[2]]] | tab[ptr[pixel[10]]];
                d &= tab[ptr[pixel[4]]] | tab[ptr[pixel[12]]];
                d &= tab[ptr[pixel[6]]] | tab[ptr[pixel[14]]];
                if (d == 0) continue;

                /* The antipodal tests above (and the remaining odd ones, which are implied) are necessary
                 * conditions only; the decision "a run of more than K = 8 consecutive
                 * pixels of the 25-long unrolled circle is darker than v - threshold (or brighter than
                 * v + threshold)" is the same as "a cyclic run of >= 9 set bits in the 16-bit mask of the 16
                 * circle pixels" (a run of >= 9 on the 16-ring always shows up as >= 9 consecutive entries of the
                 * unrolled sequence), and a dark and a bright run cannot coexist (18 > 16 pixels). */
                {
                    int vt = v - threshold, vb = v + threshold;
                    unsigned int dark = 0, bright = 0;
#define SV_FAST_MASK(k) \
    { int x = ptr[pixel[k]]; dark |= (unsigned int)(x < vt) << (k); bright |= (unsigned int)(x > vb) << (k); }
                    SV_FAST_MASK(0) SV_FAST_MASK(1) SV_FAST_MASK(2) SV_FAST_MASK(3)
                    SV_FAST_MASK(4) SV_FAST_MASK(5) SV_FAST_MASK(6) SV_FAST_MASK(7)
                    SV_FAST_MASK(8) SV_FAST_MASK(9) SV_FAST_MASK(10) SV_FAST_MASK(11)
                    SV_FAST_MASK(12) SV_FAST_MASK(13) SV_FAST_MASK(14) SV_FAST_MASK(15)
#undef SV_FAST_MASK
                    const int is_dark = sv_has_run9(dark);
                    if (is_dark || sv_has_run9(bright)) {
                        cornerpos[ncorners++] = j;
                        curr[j] = (unsigned char)sv_corner_score16(ptr, pixel, threshold, is_dark);
                    }
                }
            }
        }

        cornerpos[-1] = ncorners;

        if (i == 3) continue;

        const unsigned char* prev = buf[(i - 4 + 3) % 3];
        const unsigned char* pprev = buf[(i - 5 + 3) % 3];
        int* prev_cornerpos = cpbuf[(i - 4 + 3) % 3];
        int prev_ncorners = prev_cornerpos[-1];

        for (k = 0; k < prev_ncorners; k++) {
            j = prev_cornerpos[k];
            int score = prev[j];
            if (score > prev[j + 1] && score > prev[j - 1] &&
                score > pprev[j - 1] && score > pprev[j] && score > pprev[j + 1] &&
                score > curr[j - 1] && score > curr[j] && score > curr[j + 1]) {
                if (n_out < cap) {
                    out_keypts[n_out].x = (float)j;
                    out_keypts[n_out].y = (float)(i - 1);
                    out_keypts[n_out].size = 7.f;
                    out_keypts[n_out].angle = -1.f;
                    out_keypts[n_out].response = (float)score;
                    out_keypts[n_out].octave = 0;
                }
                n_out++;
            }
        }
    }

    for (i = 0; i < 3; i++) {
        free(buf[i]);
        free(cpbuf_storage[i]);
    }

    return n_out;
}
