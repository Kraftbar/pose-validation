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


/* stella_vslam e445b545: module/keyframe_inserter.cc (create_new_keyframe,
 * insert_new_keyframe), data/keyframe.cc (keyframe(id, frm), update_landmarks),
 * data/landmark.cc (add_observation, compute_mean_normal,
 * compute_orb_scale_variance, update_mean_normal_and_obs_scale_variance,
 * compute_descriptor -- selection delegated to Codex's read-only
 * sv_landmark_select_descriptor leaf, exactly as sv_map.c does). */
#include "sv_track.h"
#include "sv_landmark_descriptor.h"
#include "sv_linalg.h"
#include <stdlib.h>
#include <string.h>

static void vec3_normalize(const double v[3], double out[3]) {
    const double n = sv_vec3_norm(v); /* normalized(): x / sqrt(L(x.x)) */
    out[0] = v[0] / n;
    out[1] = v[1] / n;
    out[2] = v[2] / n;
}

/* landmark::update_mean_normal_and_obs_scale_variance() */
int sv_tr_lm_update_mean_normal_and_obs_scale_variance(const sv_tr_config* cfg, const sv_tr_map* map, sv_tr_lm* lm) {
    double mean_normal[3] = {0.0, 0.0, 0.0}, out[3];
    unsigned int i;
    int ref_idx = -1;
    const sv_tr_kf* ref = NULL;
    double vec[3], dist;
    float scale_factor;
    int octave;

    for (i = 0; i < lm->num_obs; ++i) { /* observations_t: ascending keyframe id */
        const sv_tr_kf* kf = ((unsigned int)lm->obs_kf[i] < map->kf_cap) ? map->kfs[lm->obs_kf[i]] : NULL;
        double normal[3], nn[3];
        if (!kf) {
            return -1;
        }
        normal[0] = lm->pos_w[0] - kf->trans_wc[0];
        normal[1] = lm->pos_w[1] - kf->trans_wc[1];
        normal[2] = lm->pos_w[2] - kf->trans_wc[2];
        vec3_normalize(normal, nn);
        mean_normal[0] = mean_normal[0] + nn[0];
        mean_normal[1] = mean_normal[1] + nn[1];
        mean_normal[2] = mean_normal[2] + nn[2];
        if ((int)lm->obs_kf[i] == lm->ref_kf) {
            ref = kf;
            ref_idx = (int)lm->obs_idx[i];
        }
    }
    vec3_normalize(mean_normal, out);
    if (!ref) {
        return -1;
    }
    lm->mean_normal[0] = out[0];
    lm->mean_normal[1] = out[1];
    lm->mean_normal[2] = out[2];

    /* compute_orb_scale_variance */
    vec[0] = lm->pos_w[0] - ref->trans_wc[0];
    vec[1] = lm->pos_w[1] - ref->trans_wc[1];
    vec[2] = lm->pos_w[2] - ref->trans_wc[2];
    dist = sv_vec3_norm(vec);
    octave = ref->obs->kp[ref_idx].octave;
    scale_factor = cfg->scale_factors[octave];
    lm->max_valid_dist = (float)(dist * scale_factor);
    lm->min_valid_dist = lm->max_valid_dist * cfg->inv_scale_factors[cfg->num_levels - 1];
    return 0;
}

/* landmark::compute_descriptor() */
int sv_tr_lm_compute_descriptor(const sv_tr_map* map, sv_tr_lm* lm) {
    sv_descriptor_observation* obs = (sv_descriptor_observation*)malloc((lm->num_obs ? lm->num_obs : 1) * sizeof(*obs));
    sv_landmark_descriptor_result res;
    unsigned int i;
    int rc;
    for (i = 0; i < lm->num_obs; ++i) {
        const sv_tr_kf* kf = ((unsigned int)lm->obs_kf[i] < map->kf_cap) ? map->kfs[lm->obs_kf[i]] : NULL;
        if (!kf) {
            free(obs);
            return -1;
        }
        obs[i].keyframe_id = lm->obs_kf[i];
        obs[i].descriptor = kf->obs->desc + (size_t)lm->obs_idx[i] * SV_TR_DESC_BYTES;
        obs[i].erased = 0;
    }
    rc = sv_landmark_select_descriptor(obs, lm->num_obs, &res, NULL);
    free(obs);
    if (rc != 0) {
        return -1;
    }
    memcpy(lm->desc, res.descriptor, SV_TR_DESC_BYTES);
    return 0;
}

int sv_tr_create_new_keyframe(const sv_tr_config* cfg, sv_tr_map* map, const sv_tr_frame* curr,
                              unsigned int new_id, double timestamp, sv_tr_kf* kf) {
    unsigned int idx;
    memset(kf, 0, sizeof(*kf));
    kf->id = new_id;
    kf->alive = 1;
    kf->timestamp = timestamp;
    kf->obs = curr->obs;
    kf->parent = SV_TR_NONE;
    kf->lm = (int*)malloc((curr->obs->num_kp ? curr->obs->num_kp : 1) * sizeof(int));
    memcpy(kf->lm, curr->lm, curr->obs->num_kp * sizeof(int));
    /* keyframe(id, frm): set_pose_cw(frm.get_pose_cw()) */
    sv_tr_kf_set_pose_cw(kf, curr->pose_cw);
    map->kfs[new_id] = kf;

    /* keyframe::update_landmarks() */
    for (idx = 0; idx < kf->obs->num_kp; ++idx) {
        sv_tr_lm* lm;
        if (kf->lm[idx] < 0 || !sv_tr_map_lm(map, kf->lm[idx])) {
            continue;
        }
        lm = map->lms[kf->lm[idx]];
        /* add_observation: appended (the new keyframe has the highest id) */
        lm->obs_kf = (unsigned int*)realloc(lm->obs_kf, (lm->num_obs + 1) * sizeof(unsigned int));
        lm->obs_idx = (unsigned int*)realloc(lm->obs_idx, (lm->num_obs + 1) * sizeof(unsigned int));
        lm->obs_kf[lm->num_obs] = new_id;
        lm->obs_idx[lm->num_obs] = idx;
        lm->num_obs += 1;
        if (sv_tr_lm_update_mean_normal_and_obs_scale_variance(cfg, map, lm) != 0) {
            return -1;
        }
        if (sv_tr_lm_compute_descriptor(map, lm) != 0) {
            return -1;
        }
    }
    return 0;
}
