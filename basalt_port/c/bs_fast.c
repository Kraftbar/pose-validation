/* SPDX-License-Identifier: BSD-3-Clause AND Apache-2.0 AND BSD-2-Clause
 * See bs_fast.h. Adapted from OpenCV 4.6.0 features2d fast.cpp / fast_score.cpp / fast.avx2.cpp (Apache-2.0).
 *
 * Original FAST copyright (OpenCV fast.cpp):
 *   Copyright (c) 2006, 2008 Edward Rosten. All rights reserved.
 *   Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following
 *   conditions are met: *Redistributions of source code must retain the above copyright notice, this list of conditions and the
 *   following disclaimer. *Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the
 *   following disclaimer in the documentation and/or other materials provided with the distribution. *Neither the name of
 *   the University of Cambridge nor the names of its contributors may be used to endorse or promote products derived from this
 *   software without specific prior written permission.
 *   THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING,
 *   BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT
 *   SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 *   DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 *   INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE
 *   OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 * OpenCV parts: Copyright (C) 2000-2008 Intel Corporation, (C) 2008-2012 Willow Garage, (C) 2013 OpenCV Foundation, Apache-2.0
 * (basalt_port/reference_cv/LICENSE-OpenCV-Apache-2.0). This file is a modified C99 adaptation, not an upstream file.
 */
#include "bs_fast.h"
#include <stdlib.h>
#include <string.h>

/* ---------------------------------------------------------------- FAST 9-16 */

static const int OFF16[16][2] = {
    {0, 3}, {1, 3}, {2, 2}, {3, 1}, {3, 0}, {3, -1}, {2, -2}, {1, -3},
    {0, -3}, {-1, -3}, {-2, -2}, {-3, -1}, {-3, 0}, {-3, 1}, {-2, 2}, {-1, 3}};

/* score = (uchar)(max(max_arc min d, -min_arc max d) - 1) over the 16 arcs of 9 contiguous ring pixels, d = v - ring (fast_score.cpp) */
static int corner_score(const uint8_t *ptr, const int *pixel)
{
    short d[25];
    int v = ptr[0], k;
    for (k = 0; k < 25; k++) d[k] = (short)(v - ptr[pixel[k]]);
    int q0 = -1000, q1 = 1000;
    for (k = 0; k < 16; k++) {
        int a = d[k], b = d[k], m;
        for (m = 1; m <= 8; m++) {
            if (d[k + m] < a) a = d[k + m];
            if (d[k + m] > b) b = d[k + m];
        }
        if (a > q0) q0 = a;
        if (b < q1) q1 = b;
    }
    if (-q1 > q0) q0 = -q1;
    return q0 - 1;
}

/* a pixel is a corner iff 9 contiguous of the 25 wrapped ring entries are all > v + t, or all < v - t (the scalar path of fast.cpp) */
static int is_corner(const uint8_t *ptr, const int *pixel, int threshold)
{
    int v = ptr[0], k, c0 = 0, c1 = 0;
    int hi = v + threshold, lo = v - threshold;
    for (k = 0; k < 25; k++) {
        int x = ptr[pixel[k]];
        if (x > hi) { if (++c0 > 8) return 1; } else c0 = 0;
        if (x < lo) { if (++c1 > 8) return 1; } else c1 = 0;
    }
    return 0;
}

int bs_fast9_16(const uint8_t *img, int w, int h, int step, int threshold, int nonmax, bs_fast_kp *out, int cap)
{
    int pixel[25], i, j, k, nk = 0, ret = 0;
    if (!img || w < 1 || h < 1 || step < w || threshold < 0 || threshold > 127 || (cap > 0 && !out)) return -2;
    for (k = 0; k < 16; k++) pixel[k] = OFF16[k][0] + OFF16[k][1] * step;
    for (; k < 25; k++) pixel[k] = pixel[k - 16];

    uint8_t *buf[3];
    int *cp[3];
    uint8_t *bmem = (uint8_t *)calloc((size_t)3 * (size_t)w, 1);
    int *cmem = (int *)calloc((size_t)3 * ((size_t)w + 1), sizeof(int));
    if (!bmem || !cmem) { free(bmem); free(cmem); return -3; }
    for (k = 0; k < 3; k++) { buf[k] = bmem + (size_t)k * (size_t)w; cp[k] = cmem + (size_t)k * ((size_t)w + 1); }

    for (i = 3; i < h - 2; i++) {
        const uint8_t *ptr = img + (size_t)i * (size_t)step + 3;
        uint8_t *curr = buf[(i - 3) % 3];
        int *cornerpos = cp[(i - 3) % 3] + 1;
        int ncorners = 0;
        memset(curr, 0, (size_t)w);
        if (i < h - 3) {
            for (j = 3; j < w - 3; j++, ptr++) {
                if (is_corner(ptr, pixel, threshold)) {
                    cornerpos[ncorners++] = j;
                    if (nonmax) curr[j] = (uint8_t)corner_score(ptr, pixel);
                }
            }
        }
        cornerpos[-1] = ncorners;
        if (i == 3) continue;

        const uint8_t *prev = buf[(i - 4 + 3) % 3];
        const uint8_t *pprev = buf[(i - 5 + 3) % 3];
        cornerpos = cp[(i - 4 + 3) % 3] + 1;
        ncorners = cornerpos[-1];
        for (k = 0; k < ncorners; k++) {
            j = cornerpos[k];
            int score = prev[j];
            if (!nonmax || (score > prev[j + 1] && score > prev[j - 1] && score > pprev[j - 1] && score > pprev[j] && score > pprev[j + 1] &&
                            score > curr[j - 1] && score > curr[j] && score > curr[j + 1])) {
                if (nk < cap) { out[nk].x = (float)j; out[nk].y = (float)(i - 1); out[nk].response = (float)score; }
                nk++;
            }
        }
    }
    free(bmem); free(cmem);
    ret = nk > cap ? -1 : nk;
    return ret;
}

/* ------------------------------------------------------- std::sort (introsort) */
/* Written from the introsort description (Musser) with the libstdc++ parameters (cutoff 16, median of three moved to the front,
 * unguarded Hoare partition, depth limit 2 floor(log2 n), heap-sort fallback, final insertion sort), no libstdc++ text copied;
 * its tie order is verified against std::sort in bs_image_test.cc (statement "sort"). */
typedef bs_fast_kp kp_t;
#define GT(a, b) ((a)->response > (b)->response)

static void kswap(kp_t *a, kp_t *b) { kp_t t = *a; *a = *b; *b = t; }

static void adjust_heap(kp_t *first, long hole, long len, kp_t value)
{
    const long top = hole;
    long child = hole;
    while (child < (len - 1) / 2) {
        child = 2 * (child + 1);
        if (GT(&first[child], &first[child - 1])) child--;
        first[hole] = first[child];
        hole = child;
    }
    if ((len & 1) == 0 && child == (len - 2) / 2) {
        child = 2 * (child + 1);
        first[hole] = first[child - 1];
        hole = child - 1;
    }
    long parent = (hole - 1) / 2;
    while (hole > top && GT(&first[parent], &value)) {
        first[hole] = first[parent];
        hole = parent;
        parent = (hole - 1) / 2;
    }
    first[hole] = value;
}

static void heap_sort(kp_t *first, kp_t *last)
{
    long len = (long)(last - first), parent;
    if (len < 2) return;
    parent = (len - 2) / 2;
    for (;;) {
        adjust_heap(first, parent, len, first[parent]);
        if (parent == 0) break;
        parent--;
    }
    while (last - first > 1) {
        --last;
        kp_t value = *last;
        *last = *first;
        adjust_heap(first, 0, (long)(last - first), value);
    }
}

static void move_median_to_first(kp_t *result, kp_t *a, kp_t *b, kp_t *c)
{
    if (GT(a, b)) {
        if (GT(b, c)) kswap(result, b);
        else if (GT(a, c)) kswap(result, c);
        else kswap(result, a);
    } else if (GT(a, c)) kswap(result, a);
    else if (GT(b, c)) kswap(result, c);
    else kswap(result, b);
}

static kp_t *unguarded_partition(kp_t *first, kp_t *last, kp_t *pivot)
{
    for (;;) {
        while (GT(first, pivot)) ++first;
        --last;
        while (GT(pivot, last)) --last;
        if (!(first < last)) return first;
        kswap(first, last);
        ++first;
    }
}

static void unguarded_linear_insert(kp_t *last)
{
    kp_t val = *last;
    kp_t *next = last - 1;
    while (GT(&val, next)) { *last = *next; last = next; --next; }
    *last = val;
}

static void insertion_sort(kp_t *first, kp_t *last)
{
    kp_t *i;
    if (first == last) return;
    for (i = first + 1; i != last; ++i) {
        if (GT(i, first)) {
            kp_t val = *i;
            memmove(first + 1, first, (size_t)(i - first) * sizeof(kp_t));
            *first = val;
        } else unguarded_linear_insert(i);
    }
}

static void introsort_loop(kp_t *first, kp_t *last, long depth_limit)
{
    while (last - first > 16) {
        if (depth_limit == 0) { heap_sort(first, last); return; }
        --depth_limit;
        kp_t *mid = first + (last - first) / 2;
        move_median_to_first(first, first + 1, mid, last - 1);
        kp_t *cut = unguarded_partition(first + 1, last, first);
        introsort_loop(cut, last, depth_limit);
        last = cut;
    }
}

void bs_fast_sort_response_desc(bs_fast_kp *v, size_t n)
{
    long lg = 0;
    size_t m = n;
    kp_t *first = v, *last = v + n, *i;
    if (n < 2) return;
    while (m > 1) { m >>= 1; lg++; }
    introsort_loop(first, last, lg * 2);
    if (last - first > 16) {
        insertion_sort(first, first + 16);
        for (i = first + 16; i != last; ++i) unguarded_linear_insert(i);
    } else insertion_sort(first, last);
}

/* test hook: the same sort with an explicit introsort depth limit (std::__introsort_loop(first, last, depth, comp) equivalent),
 * so that the heap-sort fallback can be exercised by the oracle without an adversarial input */
void bs_fast_sort_depth_test(bs_fast_kp *v, size_t n, long depth)
{
    kp_t *first = v, *last = v + n, *i;
    if (n < 2) return;
    introsort_loop(first, last, depth);
    if (last - first > 16) {
        insertion_sort(first, first + 16);
        for (i = first + 16; i != last; ++i) unguarded_linear_insert(i);
    } else insertion_sort(first, last);
}
