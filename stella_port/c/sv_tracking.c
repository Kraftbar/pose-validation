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


/* stella_vslam e445b545: tracking_module.cc (feed_frame, track,
 * track_current_frame, update_motion_model, update_last_frame,
 * optimize_current_frame_with_local_map, update_local_map,
 * search_local_landmarks, new_keyframe_is_needed), data/frame.cc (can_observe),
 * data/landmark.{h,cc} (is_inside_in_orb_scale, predict_scale_level),
 * match/projection.cc (match_frame_and_landmarks),
 * module/keyframe_inserter.cc (new_keyframe_is_needed). */
#include "sv_track.h"
#include "sv_eigen_mat4.h"
#include "sv_linalg.h"
#include <math.h>
#include <stdlib.h>
#include <string.h>

#define HAMMING_DIST_THR_HIGH 100u
#define MAX_HAMMING_DIST 256u

void sv_tracker_init(sv_tracker* t, const sv_tr_config* cfg) {
    memset(t, 0, sizeof(*t));
    t->cfg = cfg;
    t->local.nearest_covisibility = SV_TR_NONE;
}

void sv_tracker_free(sv_tracker* t) {
    sv_tr_local_map_free(&t->local);
    if (t->last_frm_valid) {
        sv_tr_frame_free(&t->last_frm);
    }
    sv_tr_frame_free(&t->curr_frm);
}

/* landmark::predict_scale_level(cam_to_lm_dist(float), num_scale_levels(float), log_scale_factor(float)) */
static unsigned int predict_scale_level(const sv_tr_lm* lm, float cam_to_lm_dist, float num_scale_levels, float log_scale_factor) {
    const float ratio = lm->max_valid_dist / cam_to_lm_dist;
    const int pred_scale_level = (int)ceilf(logf(ratio) / log_scale_factor);
    if (pred_scale_level < 0) {
        return 0;
    }
    else if (num_scale_levels <= (float)(unsigned int)pred_scale_level) {
        return (unsigned int)(num_scale_levels - 1);
    }
    return (unsigned int)pred_scale_level;
}

/* data::frame::can_observe(lm, ray_cos_thr, reproj, x_right, pred_scale_level) */
static int can_observe(const sv_tr_config* cfg, const sv_tr_frame* f, const sv_tr_lm* lm, float ray_cos_thr,
                       double reproj[2], unsigned int* pred_scale_level) {
    double cam_to_lm_vec[3], cam_to_lm_dist, ray_cos;
    float x_right, dist_f, max_dist, min_dist;
    const double margin_far = 1.3;
    const double margin_near = 1.0 / margin_far;
    if (!sv_tr_reproject_to_image(cfg, f->rot_cw, f->trans_cw, lm->pos_w, reproj, &x_right)) {
        return 0;
    }
    cam_to_lm_vec[0] = lm->pos_w[0] - f->trans_wc[0];
    cam_to_lm_vec[1] = lm->pos_w[1] - f->trans_wc[1];
    cam_to_lm_vec[2] = lm->pos_w[2] - f->trans_wc[2];
    cam_to_lm_dist = sv_vec3_norm(cam_to_lm_vec);
    dist_f = (float)cam_to_lm_dist;
    /* is_inside_in_orb_scale(const float dist, const float margin_far, const float margin_near) */
    max_dist = (float)margin_far * lm->max_valid_dist;
    min_dist = (float)margin_near * lm->min_valid_dist;
    if (!(min_dist <= dist_f && dist_f <= max_dist)) {
        return 0;
    }
    ray_cos = sv_vec3_dot(cam_to_lm_vec, lm->mean_normal) / cam_to_lm_dist;
    if (ray_cos < (double)ray_cos_thr) {
        return 0;
    }
    *pred_scale_level = predict_scale_level(lm, dist_f, (float)cfg->num_levels, cfg->log_scale_factor);
    return 1;
}

/* tracking_module::update_last_frame */
static int update_last_frame(sv_tracker* t, const sv_tr_map* map) {
    double pose[16];
    const sv_tr_kf* ref;
    if (t->last_frm.ref_kf < 0) {
        return 0;
    }
    ref = sv_tr_map_kf_any(map, t->last_frm.ref_kf);
    if (!ref) {
        return -1;
    }
    sv_mat4_mul(t->last_cam_pose_from_ref_keyfrm, ref->pose_cw, pose);
    sv_tr_frame_set_pose_cw(&t->last_frm, pose);
    return 0;
}

/* tracking_module::track_current_frame */
static int track_current_frame(sv_tracker* t, const sv_tr_map* map) {
    const sv_tr_config* cfg = t->cfg;
    sv_tr_frame* curr = &t->curr_frm;
    const sv_tr_kf* ref = sv_tr_map_kf_any(map, curr->ref_kf);
    int succeeded = 0;
    t->path = SV_TR_PATH_NONE;
    if (t->twist_valid && !t->force_skip_motion) {
        succeeded = sv_tr_motion_based_track(cfg, map, curr, &t->last_frm, t->twist);
        if (succeeded) {
            t->path = SV_TR_PATH_MOTION;
        }
    }
    if (!succeeded && ref && !t->force_skip_bow) {
        succeeded = sv_tr_bow_match_based_track(cfg, map, curr, &t->last_frm, ref);
        if (succeeded) {
            t->path = SV_TR_PATH_BOW;
        }
    }
    if (!succeeded && ref) {
        succeeded = sv_tr_robust_match_based_track(cfg, map, curr, &t->last_frm, ref);
        if (succeeded) {
            t->path = SV_TR_PATH_ROBUST;
        }
    }
    return succeeded;
}

/* tracking_module::update_local_map */
static int update_local_map(sv_tracker* t, const sv_tr_map* map) {
    sv_tr_frame* curr = &t->curr_frm;
    unsigned int idx;
    /* clean landmark associations */
    for (idx = 0; idx < curr->obs->num_kp; ++idx) {
        if (curr->lm[idx] < 0) {
            continue;
        }
        if (!sv_tr_map_lm(map, curr->lm[idx])) { /* will_be_erased() */
            curr->lm[idx] = SV_TR_NONE;
        }
    }
    t->local.n_lms = 0;
    t->local.n_kfs = 0;
    if (!sv_tr_acquire_local_map(t->cfg, map, curr->lm, curr->obs->num_kp, &t->local)) {
        return 0;
    }
    if (t->local.nearest_covisibility != SV_TR_NONE) {
        curr->ref_kf = t->local.nearest_covisibility;
    }
    return 1;
}

/* tracking_module::search_local_landmarks + match::projection::match_frame_and_landmarks(0.8) */
static int search_local_landmarks(sv_tracker* t, sv_tr_map* map) {
    const sv_tr_config* cfg = t->cfg;
    sv_tr_frame* curr = &t->curr_frm;
    const unsigned int lm_cap = map->lm_cap ? map->lm_cap : 1;
    unsigned char* in_curr = (unsigned char*)calloc(lm_cap, 1);
    unsigned char* cand = (unsigned char*)calloc(lm_cap, 1);
    double* reproj = (double*)calloc((size_t)lm_cap * 2, sizeof(double));
    unsigned int* scale = (unsigned int*)calloc(lm_cap, sizeof(unsigned int));
    unsigned int* indices = (unsigned int*)malloc((curr->obs->num_kp ? curr->obs->num_kp : 1) * sizeof(unsigned int));
    unsigned int idx, i;
    int found_proj_candidate = 0;
    float margin;
    const float lowe_ratio = 0.8f;

    for (idx = 0; idx < curr->obs->num_kp; ++idx) {
        sv_tr_lm* lm;
        if (curr->lm[idx] < 0 || !sv_tr_map_lm(map, curr->lm[idx])) {
            continue;
        }
        lm = map->lms[curr->lm[idx]];
        /* cannot be reprojected: already observed in the current frame */
        in_curr[lm->id] = 1;
        lm->num_observable += 1;
    }

    for (i = 0; i < t->local.n_lms; ++i) {
        sv_tr_lm* lm = map->lms[t->local.lms[i]];
        double rp[2];
        unsigned int psl = 0;
        if (in_curr[lm->id]) {
            continue;
        }
        if (!lm->alive) {
            continue;
        }
        if (can_observe(cfg, curr, lm, 0.5f, rp, &psl)) {
            reproj[2 * lm->id + 0] = rp[0];
            reproj[2 * lm->id + 1] = rp[1];
            scale[lm->id] = psl;
            cand[lm->id] = 1;
            lm->num_observable += 1;
            found_proj_candidate = 1;
        }
    }
    if (!found_proj_candidate) {
        free(in_curr); free(cand); free(reproj); free(scale); free(indices);
        return 0;
    }

    margin = (curr->id < t->last_reloc_frm_id + 2) ? cfg->margin_local_map_projection_unstable
                                                    : cfg->margin_local_map_projection;
    for (i = 0; i < t->local.n_lms; ++i) {
        const sv_tr_lm* local_lm = map->lms[t->local.lms[i]];
        unsigned int pred_scale_level, n_idx, k;
        int min_level, max_level, best_scale_level = -1, second_best_scale_level = -1, best_idx = -1;
        unsigned int best_hamm_dist = MAX_HAMMING_DIST, second_best_hamm_dist = MAX_HAMMING_DIST;
        if (!cand[local_lm->id]) {
            continue;
        }
        if (!local_lm->alive) {
            continue;
        }
        pred_scale_level = scale[local_lm->id];
        min_level = (int)pred_scale_level - 1;
        if (min_level < 0) {
            min_level = 0;
        }
        max_level = (int)(pred_scale_level < cfg->num_levels - 1 - 1u ? pred_scale_level + 1 : cfg->num_levels - 1);
        n_idx = sv_frame_get_keypoints_in_cell(&curr->obs->grid, curr->obs->kp,
                                               (float)reproj[2 * local_lm->id + 0], (float)reproj[2 * local_lm->id + 1],
                                               margin * cfg->scale_factors[pred_scale_level],
                                               min_level, max_level, indices, curr->obs->num_kp);
        if (n_idx == 0) {
            continue;
        }
        for (k = 0; k < n_idx; ++k) {
            const unsigned int cidx = indices[k];
            unsigned int dist;
            if (curr->lm[cidx] >= 0) { /* lm && lm->has_observation() */
                continue;
            }
            dist = sv_tr_hamming(local_lm->desc, curr->obs->desc + (size_t)cidx * SV_TR_DESC_BYTES);
            if (dist < best_hamm_dist) {
                second_best_hamm_dist = best_hamm_dist;
                best_hamm_dist = dist;
                second_best_scale_level = best_scale_level;
                best_scale_level = curr->obs->kp[cidx].octave;
                best_idx = (int)cidx;
            }
            else if (dist < second_best_hamm_dist) {
                second_best_scale_level = curr->obs->kp[cidx].octave;
                second_best_hamm_dist = dist;
            }
        }
        if (best_hamm_dist <= HAMMING_DIST_THR_HIGH) {
            /* Lowe's ratio test */
            if (best_scale_level == second_best_scale_level && (float)best_hamm_dist > lowe_ratio * (float)second_best_hamm_dist) {
                continue;
            }
            curr->lm[best_idx] = (int)local_lm->id;
        }
    }
    free(in_curr); free(cand); free(reproj); free(scale); free(indices);
    return 1;
}

/* tracking_module::optimize_current_frame_with_local_map */
static int optimize_current_frame_with_local_map(sv_tracker* t, sv_tr_map* map, unsigned int min_num_obs_thr) {
    sv_tr_frame* curr = &t->curr_frm;
    unsigned char* outlier = (unsigned char*)malloc(curr->obs->num_kp ? curr->obs->num_kp : 1);
    double optimized[16];
    unsigned int idx;
    const unsigned int num_tracked_lms_thr = 20;

    t->optimize_ran = 1;
    sv_tr_optimize_pose(t->cfg, map, curr, optimized, outlier);
    sv_tr_frame_set_pose_cw(curr, optimized);
    for (idx = 0; idx < curr->obs->num_kp; ++idx) {
        if (outlier[idx] && curr->lm[idx] >= 0) {
            curr->lm[idx] = SV_TR_NONE;
        }
    }
    free(outlier);

    t->num_tracked_lms = 0;
    t->num_reliable_lms = 0;
    for (idx = 0; idx < curr->obs->num_kp; ++idx) {
        sv_tr_lm* lm;
        if (curr->lm[idx] < 0 || !sv_tr_map_lm(map, curr->lm[idx])) {
            continue;
        }
        lm = map->lms[curr->lm[idx]];
        if (0 < min_num_obs_thr) {
            if (min_num_obs_thr <= lm->num_obs) {
                ++t->num_reliable_lms;
            }
        }
        ++t->num_tracked_lms;
        lm->num_observed += 1;
    }

    /* if recently relocalized, use the more strict threshold */
    if (curr->timestamp < t->last_reloc_frm_timestamp + 1.0 && t->num_tracked_lms < 2 * num_tracked_lms_thr) {
        return 0;
    }
    if (t->num_tracked_lms < num_tracked_lms_thr) {
        return 0;
    }
    return 1;
}

/* tracking_module::update_motion_model */
static void update_motion_model(sv_tracker* t) {
    if (t->last_frm.pose_valid) {
        double last_wc[16];
        int r, c;
        for (c = 0; c < 16; ++c) {
            last_wc[c] = (c % 5 == 0) ? 1.0 : 0.0; /* Identity */
        }
        for (c = 0; c < 3; ++c) {
            for (r = 0; r < 3; ++r) {
                last_wc[c * 4 + r] = t->last_frm.rot_wc[c * 3 + r];
            }
        }
        for (r = 0; r < 3; ++r) {
            last_wc[3 * 4 + r] = t->last_frm.trans_wc[r];
        }
        t->twist_valid = 1;
        sv_mat4_mul(t->curr_frm.pose_cw, last_wc, t->twist);
    }
    else {
        t->twist_valid = 0;
        for (int c = 0; c < 16; ++c) {
            t->twist[c] = (c % 5 == 0) ? 1.0 : 0.0;
        }
    }
}

/* module::keyframe_inserter::new_keyframe_is_needed (mapper never paused /
 * skipping local BA in the synchronous reference; only ref_kf is read). */
static int new_keyframe_is_needed(sv_tracker* t, const sv_tr_map* map, unsigned int min_num_obs_thr) {
    const sv_tr_config* cfg = t->cfg;
    const sv_tr_frame* curr = &t->curr_frm;
    sv_tr_kf_decision* d = &t->decision;
    const sv_tr_kf* ref = sv_tr_map_kf_any(map, curr->ref_kf);
    unsigned int num_reliable_lms_ref = 0, i;
    double diff[3];
    const int have_last = map->last_inserted_kf != SV_TR_NONE;

    memset(d, 0, sizeof(*d));
    if (t->mapper_paused) { /* mapper_->is_paused() || pause_is_requested(): last_decision_ = {}, paused, verdict false */
        d->mapper_paused_or_pausing = 1;
        return 0;
    }
    d->min_interval_elapsed = 1;
    d->min_distance_traveled = 1;
    d->distance_traveled = -1.0f;

    if (ref) {
        /* keyframe::get_num_tracked_landmarks(min_num_obs_thr) */
        for (i = 0; i < ref->obs->num_kp; ++i) {
            const sv_tr_lm* lm;
            if (ref->lm[i] < 0) {
                continue;
            }
            lm = sv_tr_map_lm(map, ref->lm[i]);
            if (!lm) {
                continue;
            }
            if (0 < min_num_obs_thr) {
                if (min_num_obs_thr <= lm->num_obs) {
                    ++num_reliable_lms_ref;
                }
            }
            else {
                ++num_reliable_lms_ref;
            }
        }
    }
    d->num_reliable_lms_ref = num_reliable_lms_ref;
    d->num_reliable_lms = t->num_reliable_lms;
    d->num_tracked_lms = t->num_tracked_lms;

    d->enough_keyfrms = map->num_keyframes > 5;

    if (cfg->max_interval > 0.0) {
        d->max_interval_elapsed = have_last && (map->last_inserted_timestamp + cfg->max_interval <= curr->timestamp);
    }
    d->min_interval_elapsed = 1;
    if (cfg->min_interval > 0.0) {
        d->min_interval_elapsed = !have_last || (map->last_inserted_timestamp + cfg->min_interval <= curr->timestamp);
    }
    if (have_last) {
        diff[0] = map->last_inserted_trans_wc[0] - curr->trans_wc[0];
        diff[1] = map->last_inserted_trans_wc[1] - curr->trans_wc[1];
        diff[2] = map->last_inserted_trans_wc[2] - curr->trans_wc[2];
        d->distance_traveled = (float)sv_vec3_norm(diff);
    }
    d->max_distance_traveled = 0;
    if (cfg->max_distance > 0.0) {
        d->max_distance_traveled = have_last && (d->distance_traveled > cfg->max_distance);
    }
    d->min_distance_traveled = 1;
    if (cfg->min_distance > 0.0) {
        d->min_distance_traveled = !have_last || (d->distance_traveled > cfg->min_distance);
    }
    d->view_changed = 0;
    if (cfg->lms_ratio_thr_view_changed > 0.0) {
        d->view_changed = (double)d->num_reliable_lms < (double)num_reliable_lms_ref * cfg->lms_ratio_thr_view_changed;
    }
    d->not_enough_lms = d->num_reliable_lms < cfg->enough_lms_thr;
    d->tracking_is_unstable = d->num_tracked_lms < 15;
    d->almost_all_lms_are_tracked = 0;
    if (cfg->lms_ratio_thr_almost_all_lms_are_tracked > 0.0) {
        d->almost_all_lms_are_tracked =
            (double)d->num_reliable_lms > (double)num_reliable_lms_ref * cfg->lms_ratio_thr_almost_all_lms_are_tracked;
    }
    d->mapper_is_skipping_localBA = 0;
    d->verdict = (d->max_interval_elapsed || d->max_distance_traveled || d->view_changed || d->not_enough_lms) &&
                 (!d->enough_keyfrms || (d->min_interval_elapsed && d->min_distance_traveled)) &&
                 !d->tracking_is_unstable && !d->almost_all_lms_are_tracked && !d->mapper_is_skipping_localBA;
    return d->verdict;
}

int sv_tracker_track(sv_tracker* t, sv_tr_map* map, const sv_tr_frame* input) {
    const sv_tr_config* cfg = t->cfg;
    const unsigned int min_num_obs_thr = (3 <= map->num_keyframes) ? 3 : 2;
    int succeeded = 0;
    sv_tr_frame fresh;

    /* curr_frm_ = curr_frm (a freshly extracted frame: no pose, no landmarks) */
    sv_tr_frame_free(&t->curr_frm);
    sv_tr_frame_init(&fresh, input->id, input->timestamp, input->obs);
    t->curr_frm = fresh;

    t->succeeded = 0;
    t->path = SV_TR_PATH_NONE;
    t->initial_pose_valid = 0;
    t->num_tracked_lms = 0;
    t->num_reliable_lms = 0;
    t->optimize_ran = 0;
    t->local.n_kfs = 0;
    t->local.n_lms = 0;
    t->decision_evaluated = 0;
    memset(&t->decision, 0, sizeof(t->decision));

    /* track(): tracking state Tracking, or Lost with a relocalization hook (sv_system) */
    if ((t->tracking_state != 1 && !(t->tracking_state == 2 && t->reloc_hook)) || !t->last_frm_valid) {
        return 0;
    }
    if (update_last_frame(t, map) != 0) {
        return 0;
    }
    t->curr_frm.ref_kf = t->last_frm.ref_kf;

    if (t->tracking_state == 1) {
        succeeded = track_current_frame(t, map);
    }
    else {
        /* bow_db_ && enable_auto_relocalization_: relocalizer_.relocalize(bow_db_, curr_frm_) */
        succeeded = t->reloc_hook(t->reloc_user, t, map);
        if (succeeded) {
            t->path = SV_TR_PATH_RELOC_AUTO;
        }
    }
    if (succeeded && t->curr_frm.pose_valid) {
        memcpy(t->initial_pose, t->curr_frm.pose_cw, sizeof(t->initial_pose));
        t->initial_pose_valid = 1;
    }
    if (succeeded) {
        /* track_local_map(...) (fixed_keyframe_id_threshold == 0) */
        succeeded = update_local_map(t, map);
        if (succeeded) {
            succeeded = search_local_landmarks(t, map);
        }
        if (succeeded) {
            succeeded = optimize_current_frame_with_local_map(t, map, min_num_obs_thr);
        }
    }
    if (succeeded) {
        update_motion_model(t);
    }
    t->succeeded = succeeded;

    /* feed_frame(): if (succeeded && !stopped && new_keyframe_is_needed(...)) */
    if (succeeded) {
        if (t->curr_frm.timestamp < t->last_reloc_frm_timestamp + 1.0) {
            t->decision_evaluated = 0; /* keyframe_inserter not consulted */
        }
        else {
            new_keyframe_is_needed(t, map, min_num_obs_thr);
            t->decision_evaluated = 1;
        }
    }
    (void)cfg;
    return succeeded;
}

int sv_tracker_finish_frame(sv_tracker* t, const sv_tr_map* map, int inserted_kf_id) {
    sv_tr_frame* curr = &t->curr_frm;
    if (inserted_kf_id >= 0) {
        curr->ref_kf = inserted_kf_id; /* insert_new_keyframe: curr_frm.ref_keyfrm_ = new keyframe */
    }
    if (t->succeeded) {
        t->tracking_state = 1;
    }
    else if (t->tracking_state == 1) {
        t->tracking_state = 2; /* Lost (reset()/relocalization are not ported) */
    }
    if (curr->pose_valid) {
        const sv_tr_kf* ref = sv_tr_map_kf_any(map, curr->ref_kf);
        if (ref) {
            /* last_cam_pose_from_ref_keyfrm_ = curr_frm_.get_pose_cw() * curr_frm_.ref_keyfrm_->get_pose_wc(); */
            sv_mat4_mul(curr->pose_cw, ref->pose_wc, t->last_cam_pose_from_ref_keyfrm);
        }
    }
    if (t->last_frm_valid) {
        sv_tr_frame_free(&t->last_frm);
    }
    memset(&t->last_frm, 0, sizeof(t->last_frm));
    if (sv_tr_frame_copy(&t->last_frm, curr) != 0) {
        return -1;
    }
    t->last_frm_valid = 1;
    return 0;
}
