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


/* stella_vslam e445b545: match/robust.cc (frame/keyframe path),
 * module/frame_tracker.cc::robust_match_based_track. */
#include "sv_track.h"
#include "sv_solve_essential_ransac.h"
#include <math.h>
#include <stdlib.h>
#include <string.h>

int sv_tr_robust_match_based_track(const sv_tr_config* cfg, const sv_tr_map* map, sv_tr_frame* curr,
                                   const sv_tr_frame* last, const sv_tr_kf* ref_kf) {
    unsigned n = curr->obs->num_kp, count = 0, valid = 0;
    int ok = 0;
    int* pairs = malloc((n ? n : 1) * sizeof(*pairs));
    double* a = malloc((n ? n : 1) * 3 * sizeof(*a));
    double* b = malloc((n ? n : 1) * 3 * sizeof(*b));
    unsigned char* mask = malloc(n ? n : 1);
    unsigned char* outlier = malloc(n ? n : 1);
    if (!pairs || !a || !b || !mask || !outlier) goto done;
    if (sv_tr_obs_ensure_bearings(curr->obs, cfg)
        || sv_tr_obs_ensure_bearings(ref_kf->obs, cfg)) goto done;
    for (unsigned i = 0; i < n; ++i) pairs[i] = -1;
    /* Keyframe order decides ownership; accepted pairs are emitted in
     * current-frame order, exactly as robust::brute_force_match does. */
    for (unsigned j = 0; j < ref_kf->obs->num_kp; ++j) {
        if (!sv_tr_map_lm(map, ref_kf->lm[j])) continue;
        unsigned best = 256, second = 256;
        int best_i = -1;
        for (unsigned i = 0; i < n; ++i) {
            if (pairs[i] >= 0) continue;
            if (fabsf(sv_tr_angle_diff(curr->obs->kp[i].angle,
                                      ref_kf->obs->kp[j].angle)) > 30.0) continue;
            unsigned d = sv_tr_hamming(ref_kf->obs->desc + (size_t)j * 32,
                                       curr->obs->desc + (size_t)i * 32);
            if (d < best) { second = best; best = d; best_i = (int)i; }
            else if (d < second) second = d;
        }
        if (best > 50 || best_i < 0 || 0.8f * second < (float)best) continue;
        pairs[best_i] = (int)j;
    }
    for (unsigned i = 0; i < n; ++i) {
        if (pairs[i] < 0) continue;
        memcpy(a + 3 * count, curr->obs->bearings + 3 * i, 3 * sizeof(double));
        memcpy(b + 3 * count, ref_kf->obs->bearings + 3 * pairs[i], 3 * sizeof(double));
        ++count;
    }
    sv_essential_result result;
    if (sv_essential_ransac(a, b, count, 1000, 1, 5, NULL, &result, mask, NULL, NULL)
        || !result.valid || result.inliers < cfg->num_matches_thr) goto done;
    count = 0;
    for (unsigned i = 0; i < n; ++i) {
        curr->lm[i] = SV_TR_NONE;
        if (pairs[i] >= 0 && mask[count++]) curr->lm[i] = ref_kf->lm[pairs[i]];
    }
    sv_tr_frame_set_pose_cw(curr, last->pose_cw);
    double optimized[16];
    sv_tr_optimize_pose(cfg, map, curr, optimized, outlier);
    sv_tr_frame_set_pose_cw(curr, optimized);
    for (unsigned i = 0; i < n; ++i) {
        if (curr->lm[i] < 0) continue;
        if (outlier[i]) curr->lm[i] = SV_TR_NONE;
        else ++valid;
    }
    ok = valid >= cfg->num_matches_thr;
done:
    free(pairs); free(a); free(b); free(mask); free(outlier);
    return ok;
}
