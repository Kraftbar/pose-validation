/* SPDX-License-Identifier: BSD-2-Clause */
/* BSD 2-Clause License
 * Copyright (c) 2019, National Institute of Advanced Industrial Science
 * and Technology (AIST), All rights reserved.
 * Copyright (c) 2022, stella-cv, All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.

 */

/* stella_vslam e445b545 match/bow_tree.cc (the two non-triangulation
 * methods), match/base.h ORB distance, util/angle.cc. */
#include "sv_match_bow.h"
#include <math.h>
#include <stdlib.h>
#include <string.h>
#include <limits.h>

static unsigned distance(const uint8_t *a, const uint8_t *b)
{
    unsigned d = 0;
    for (unsigned i = 0; i < 32; ++i) {
        unsigned v = a[i] ^ b[i];
        v = v - ((v >> 1) & 0x55u);
        v = (v & 0x33u) + ((v >> 2) & 0x33u);
        d += (v + (v >> 4)) & 0x0fu;
    }
    return d;
}

static float angle_diff(float a, float b)
{
    float d = a - b;
    /* Upstream literals are double: preserve widening on wrap. */
    if (d <= -180.0) d = (float)(d + 360.0);
    if (d > 180.0) d = (float)(d - 360.0);
    return d;
}

static int valid(const sv_match_bow_view *v)
{
    if (!v || !v->features || (v->count && (!v->keypoints || !v->descriptors))
        || (v->features->count && !v->features->nodes)
        || v->count > INT_MAX) return 0;
    uint8_t *seen = v->count ? calloc(v->count, 1) : NULL;
    if (v->count && !seen) return 0;
    int ok = 1;
    for (uint32_t n = 0; n < v->features->count && ok; ++n) {
        const sv_bow_feat_node *node = &v->features->nodes[n];
        if ((n && v->features->nodes[n-1].node_id >= node->node_id)
            || (node->count && !node->kp_indices)) { ok = 0; break; }
        for (uint32_t j = 0; j < node->count; ++j) {
            uint32_t i = node->kp_indices[j];
            if (i >= v->count || seen[i] || !isfinite(v->keypoints[i].angle)) {
                ok = 0; break;
            }
            seen[i] = 1;
        }
    }
    free(seen);
    return ok;
}

static int live(const sv_match_bow_view *v, uint32_t i)
{
    return v->landmarks && v->landmarks[i] && (!v->erased || !v->erased[i]);
}

static int match(const sv_match_bow_view *a, const sv_match_bow_view *b,
                 float ratio, int orientation, int keyframes,
                 uint64_t *out, uint32_t *num)
{
    if (!num || !isfinite(ratio) || ratio < 0 || !valid(a) || !valid(b)) return -1;
    uint32_t length = keyframes ? a->count : b->count;
    if (length && !out) return -1;
    uint64_t *result = length ? calloc(length, sizeof(*result)) : NULL;
    uint8_t *used = b->count ? calloc(b->count, 1) : NULL;
    if ((length && !result) || (b->count && !used)) { free(result); free(used); return -1; }
    uint32_t count = 0, x = 0, y = 0;
    while (x < a->features->count && y < b->features->count) {
        const sv_bow_feat_node *an = &a->features->nodes[x];
        const sv_bow_feat_node *bn = &b->features->nodes[y];
        if (an->node_id < bn->node_id) { ++x; continue; }
        if (an->node_id > bn->node_id) { ++y; continue; }
        for (uint32_t ai = 0; ai < an->count; ++ai) {
            uint32_t i = an->kp_indices[ai];
            if (!live(a, i)) continue;
            unsigned best = 256, second = 256;
            int best_j = -1;
            for (uint32_t bj = 0; bj < bn->count; ++bj) {
                uint32_t j = bn->kp_indices[bj];
                if (used[j] || (keyframes && !live(b, j))) continue;
                if (orientation && fabsf(angle_diff(a->keypoints[i].angle,
                                                    b->keypoints[j].angle)) > 30.0) continue;
                unsigned d = distance(a->descriptors + (size_t)i*32,
                                      b->descriptors + (size_t)j*32);
                if (d < best) { second = best; best = d; best_j = (int)j; }
                else if (d < second) second = d;
            }
            if (best > 50 || ratio * second < (float)best) continue;
            if (keyframes) result[i] = b->landmarks[best_j];
            else result[best_j] = a->landmarks[i];
            used[best_j] = 1;
            ++count;
        }
        ++x; ++y;
    }
    if (length) memcpy(out, result, (size_t)length * sizeof(*out));
    *num = count;
    free(result); free(used);
    return 0;
}

int sv_match_bow_frame(const sv_match_bow_view *a, const sv_match_bow_view *b,
                       float ratio, int orientation, uint64_t *out, uint32_t *num)
{ return match(a, b, ratio, orientation, 0, out, num); }

int sv_match_bow_keyframes(const sv_match_bow_view *a, const sv_match_bow_view *b,
                           float ratio, int orientation, uint64_t *out, uint32_t *num)
{ return match(a, b, ratio, orientation, 1, out, num); }
