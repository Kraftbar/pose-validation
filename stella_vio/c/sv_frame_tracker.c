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


/* stella_vslam e445b545: match/projection.cc (match_current_and_last_frames),
 * module/frame_tracker.cc, optimize/pose_optimizer_g2o.cc (frame overload:
 * edge construction only; the optimizer itself is sv_g2o_pose_optimizer.c),
 * match/bow_tree.h glue for sv_match_bow.c (Codex's leaf, used read-only). */
#include "sv_track.h"
#include "sv_eigen_mat4.h"
#include "sv_match_bow.h"
#include <math.h>
#include <stdlib.h>
#include <string.h>

#define HAMMING_DIST_THR_HIGH 100u
#define MAX_HAMMING_DIST 256u

unsigned int sv_tr_match_current_and_last_frames(const sv_tr_config* cfg, const sv_tr_map* map,
                                                 sv_tr_frame* curr, const sv_tr_frame* last, float margin) {
    unsigned int num_matches = 0;
    unsigned int idx_last;
    unsigned int* indices = (unsigned int*)malloc((curr->obs->num_kp ? curr->obs->num_kp : 1) * sizeof(unsigned int));
    if (!indices) {
        return 0;
    }
    /* Monocular: assume_forward / assume_backward are both false, so the
     * curr->last translation (trans_lc) is never consulted. */
    for (idx_last = 0; idx_last < last->obs->num_kp; ++idx_last) {
        const sv_tr_lm* lm;
        double reproj[2];
        float x_right;
        unsigned int last_scale_level, n_idx, k;
        int min_level, max_level;
        unsigned int best_hamm_dist = MAX_HAMMING_DIST;
        int best_idx = -1;

        if (last->lm[idx_last] < 0) {
            continue;
        }
        lm = sv_tr_map_lm(map, last->lm[idx_last]);
        if (!lm) { /* will_be_erased() */
            continue;
        }
        if (!sv_tr_reproject_to_image(cfg, curr->rot_cw, curr->trans_cw, lm->pos_w, reproj, &x_right)) {
            continue;
        }

        last_scale_level = (unsigned int)last->obs->kp[idx_last].octave;
        min_level = (int)last_scale_level - 1;
        if (min_level < 0) {
            min_level = 0;
        }
        max_level = (int)(last_scale_level + 1 < cfg->num_levels - 1 ? last_scale_level + 1 : cfg->num_levels - 1);
        n_idx = sv_frame_get_keypoints_in_cell(&curr->obs->grid, curr->obs->kp, (float)reproj[0], (float)reproj[1],
                                               margin * cfg->scale_factors[last_scale_level],
                                               min_level, max_level, indices, curr->obs->num_kp);
        if (n_idx == 0) {
            continue;
        }

        for (k = 0; k < n_idx; ++k) {
            const unsigned int curr_idx = indices[k];
            unsigned int hamm_dist;
            if (curr->lm[curr_idx] >= 0) { /* curr_lm && curr_lm->has_observation() */
                continue;
            }
            if (fabsf(sv_tr_angle_diff(last->obs->kp[idx_last].angle, curr->obs->kp[curr_idx].angle)) > 30.0) {
                continue;
            }
            hamm_dist = sv_tr_hamming(lm->desc, curr->obs->desc + (size_t)curr_idx * SV_TR_DESC_BYTES);
            if (hamm_dist < best_hamm_dist) {
                best_hamm_dist = hamm_dist;
                best_idx = (int)curr_idx;
            }
        }

        if (HAMMING_DIST_THR_HIGH < best_hamm_dist) {
            continue;
        }
        curr->lm[best_idx] = (int)lm->id;
        ++num_matches;
    }
    free(indices);
    return num_matches;
}

/* ---- pose_optimizer_g2o::optimize(frame, ...) edge construction ---- */

static void pose_to_se3(const double pose_cw[16], sv_se3* out) {
    double rot[9];
    int r, c;
    for (c = 0; c < 3; ++c) {
        for (r = 0; r < 3; ++r) {
            rot[c * 3 + r] = pose_cw[c * 4 + r];
        }
    }
    sv_quat_from_mat3(rot, &out->q);
    out->t[0] = pose_cw[3 * 4 + 0];
    out->t[1] = pose_cw[3 * 4 + 1];
    out->t[2] = pose_cw[3 * 4 + 2];
    /* util::converter::to_g2o_SE3 == SE3Quat{rot, trans}, whose constructor
     * calls normalizeRotation() (HANDOVER module 4b closure item 2). */
    sv_se3_normalize_rotation(out);
}

static void se3_to_pose(const sv_se3* pose, double out[16]) {
    double rot[9];
    int r, c;
    sv_quat_to_mat3(&pose->q, rot);
    for (c = 0; c < 3; ++c) {
        for (r = 0; r < 3; ++r) {
            out[c * 4 + r] = rot[c * 3 + r];
        }
    }
    out[3 * 4 + 0] = pose->t[0];
    out[3 * 4 + 1] = pose->t[1];
    out[3 * 4 + 2] = pose->t[2];
    out[0 * 4 + 3] = 0.0;
    out[1 * 4 + 3] = 0.0;
    out[2 * 4 + 3] = 0.0;
    out[3 * 4 + 3] = 1.0;
}

unsigned int sv_tr_optimize_pose(const sv_tr_config* cfg, const sv_tr_map* map, const sv_tr_frame* curr,
                                 double pose_out[16], unsigned char* outlier) {
    const unsigned int num_kp = curr->obs->num_kp;
    sv_pose_opt_edge* edges = (sv_pose_opt_edge*)malloc((num_kp ? num_kp : 1) * sizeof(sv_pose_opt_edge));
    unsigned int* edge_idx = (unsigned int*)malloc((num_kp ? num_kp : 1) * sizeof(unsigned int));
    unsigned int idx, n = 0, i, num_valid;
    sv_se3 pose;

    memset(outlier, 0, num_kp);
    for (idx = 0; idx < num_kp; ++idx) {
        const sv_tr_lm* lm;
        sv_pose_opt_edge* e;
        if (curr->lm[idx] < 0) {
            continue;
        }
        lm = sv_tr_map_lm(map, curr->lm[idx]);
        if (!lm) {
            continue;
        }
        e = &edges[n];
        edge_idx[n] = idx;
        e->pos_w[0] = lm->pos_w[0];
        e->pos_w[1] = lm->pos_w[1];
        e->pos_w[2] = lm->pos_w[2];
        e->obs[0] = (double)curr->obs->kp[idx].x;
        e->obs[1] = (double)curr->obs->kp[idx].y;
        e->inv_sigma_sq = (double)cfg->inv_level_sigma_sq[curr->obs->kp[idx].octave];
        e->fx = cfg->fx;
        e->fy = cfg->fy;
        e->cx = cfg->cx;
        e->cy = cfg->cy;
        e->level = 0;
        ++n;
    }
    pose_to_se3(curr->pose_cw, &pose);
    num_valid = sv_pose_optimizer_optimize(&pose, edges, (int)n, &cfg->pose_opt);
    if (n < 5) {
        memcpy(pose_out, curr->pose_cw, 16 * sizeof(double));
    } else {
        se3_to_pose(&pose, pose_out);
        for (i = 0; i < n; ++i) {
            outlier[edge_idx[i]] = (unsigned char)(edges[i].level != 0);
        }
    }
    free(edges);
    free(edge_idx);
    return num_valid;
}

/* frame_tracker::discard_outliers */
static unsigned int discard_outliers(const unsigned char* outlier, sv_tr_frame* curr) {
    unsigned int num_valid = 0, idx;
    for (idx = 0; idx < curr->obs->num_kp; ++idx) {
        if (curr->lm[idx] < 0) {
            continue;
        }
        if (outlier[idx]) {
            curr->lm[idx] = SV_TR_NONE;
        } else {
            ++num_valid;
        }
    }
    return num_valid;
}

static void erase_landmarks(sv_tr_frame* f) {
    unsigned int i;
    for (i = 0; i < f->obs->num_kp; ++i) {
        f->lm[i] = SV_TR_NONE;
    }
}

/* Common tail of the three trackers: pose optimization + discard. */
static int optimize_and_discard(const sv_tr_config* cfg, const sv_tr_map* map, sv_tr_frame* curr) {
    unsigned char* outlier = (unsigned char*)malloc(curr->obs->num_kp ? curr->obs->num_kp : 1);
    double optimized[16];
    unsigned int num_valid;
    sv_tr_optimize_pose(cfg, map, curr, optimized, outlier);
    sv_tr_frame_set_pose_cw(curr, optimized);
    num_valid = discard_outliers(outlier, curr);
    free(outlier);
    return num_valid >= cfg->num_matches_thr;
}

int sv_tr_motion_based_track(const sv_tr_config* cfg, const sv_tr_map* map, sv_tr_frame* curr,
                             const sv_tr_frame* last, const double velocity[16]) {
    double pose[16];
    unsigned int num_matches;
    /* curr_frm.set_pose_cw(velocity * last_frm.get_pose_cw()); */
    sv_mat4_mul(velocity, last->pose_cw, pose);
    sv_tr_frame_set_pose_cw(curr, pose);

    erase_landmarks(curr);
    num_matches = sv_tr_match_current_and_last_frames(cfg, map, curr, last, cfg->margin_last_frame_projection);
    if (num_matches < cfg->num_matches_thr) {
        erase_landmarks(curr);
        num_matches = sv_tr_match_current_and_last_frames(cfg, map, curr, last, 2 * cfg->margin_last_frame_projection);
    }
    if (num_matches < cfg->num_matches_thr) {
        return 0;
    }
    return optimize_and_discard(cfg, map, curr);
}

int sv_tr_bow_match_based_track(const sv_tr_config* cfg, const sv_tr_map* map, sv_tr_frame* curr,
                                const sv_tr_frame* last, const sv_tr_kf* ref_kf) {
    sv_match_bow_view kv, fv;
    uint64_t* tokens;
    uint64_t* matched;
    uint32_t count = 0;
    unsigned int i;
    int rc;

    if (sv_tr_obs_ensure_bow(ref_kf->obs, cfg) != 0 || sv_tr_obs_ensure_bow(curr->obs, cfg) != 0) {
        return 0;
    }
    tokens = (uint64_t*)calloc(ref_kf->obs->num_kp ? ref_kf->obs->num_kp : 1, sizeof(uint64_t));
    matched = (uint64_t*)calloc(curr->obs->num_kp ? curr->obs->num_kp : 1, sizeof(uint64_t));
    for (i = 0; i < ref_kf->obs->num_kp; ++i) {
        tokens[i] = (ref_kf->lm[i] >= 0 && sv_tr_map_lm(map, ref_kf->lm[i])) ? (uint64_t)ref_kf->lm[i] + 1u : 0u;
    }
    memset(&kv, 0, sizeof(kv));
    memset(&fv, 0, sizeof(fv));
    kv.count = ref_kf->obs->num_kp;
    kv.keypoints = ref_kf->obs->kp;
    kv.descriptors = ref_kf->obs->desc;
    kv.features = &ref_kf->obs->bow_feat;
    kv.landmarks = tokens;
    fv.count = curr->obs->num_kp;
    fv.keypoints = curr->obs->kp;
    fv.descriptors = curr->obs->desc;
    fv.features = &curr->obs->bow_feat;
    /* bow_tree bow_matcher(0.7, true); match_frame_and_keyframe(ref_keyfrm, curr_frm, ...) */
    rc = sv_match_bow_frame(&kv, &fv, 0.7f, 1, matched, &count);
    free(tokens);
    if (rc != 0 || count < cfg->num_matches_thr) {
        free(matched);
        return 0;
    }
    /* curr_frm.set_landmarks(matched_lms_in_curr) */
    for (i = 0; i < curr->obs->num_kp; ++i) {
        curr->lm[i] = matched[i] ? (int)(matched[i] - 1u) : SV_TR_NONE;
    }
    free(matched);
    /* The initial value is the pose of the previous frame */
    sv_tr_frame_set_pose_cw(curr, last->pose_cw);
    return optimize_and_discard(cfg, map, curr);
}
