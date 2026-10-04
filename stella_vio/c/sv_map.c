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

/* Port of data/keyframe.cc, data/landmark.cc, data/graph_node.cc (spanning
 * tree subset) and module/initializer.cc::create_map_for_monocular /
 * ::scale_map (e445b545) -- see sv_map.h. Mat33/Vec3 evaluation order via
 * sv_linalg.h (MPL-2.0, Eigen-derived; unchanged here, only called). */
#include "sv_map.h"
#include "sv_linalg.h"
#include "sv_landmark_descriptor.h"
#include <string.h>
#include <math.h>
#include <stdlib.h>

/* ---- column-major 4x4 <-> 3x3 block helpers ---------------------------- */
#define M4(m, r, c) (m)[(c) * 4 + (r)]

static void mat4_get_rot(const double m[16], double rot9[9]) {
    int r, c;
    for (c = 0; c < 3; ++c)
        for (r = 0; r < 3; ++r)
            rot9[c * 3 + r] = M4(m, r, c);
}
static void mat4_get_trans(const double m[16], double t[3]) {
    t[0] = M4(m, 0, 3);
    t[1] = M4(m, 1, 3);
    t[2] = M4(m, 2, 3);
}
static void mat4_set_identity(double m[16]) {
    int i;
    for (i = 0; i < 16; ++i) m[i] = 0.0;
    M4(m, 0, 0) = 1.0;
    M4(m, 1, 1) = 1.0;
    M4(m, 2, 2) = 1.0;
    M4(m, 3, 3) = 1.0;
}
static void mat4_set_rot(double m[16], const double rot9[9]) {
    int r, c;
    for (c = 0; c < 3; ++c)
        for (r = 0; r < 3; ++r)
            M4(m, r, c) = rot9[c * 3 + r];
}
static void mat4_set_trans(double m[16], const double t[3]) {
    M4(m, 0, 3) = t[0];
    M4(m, 1, 3) = t[1];
    M4(m, 2, 3) = t[2];
}

/* ---- data::keyframe::set_pose_cw ---------------------------------------
 * pose_wc_.block<3,3> = rot_cw.transpose() (materialized into a named
 * Mat33_t before use -> plain-matrix mixed rule for any later product,
 * per HANDOVER.md); trans_wc_ = -rot_wc * trans_cw (Matrix3d*Vector3d,
 * mixed rows-0-1-L/row-2-R rule, via sv_mat3_mulv). */
static void keyframe_set_pose_cw(sv_map_keyframe* kf, const double pose_cw[16]) {
    double rot_cw[9], trans_cw[3], rot_wc[9], trans_wc[3];
    int i;
    for (i = 0; i < 16; ++i) kf->pose_cw[i] = pose_cw[i];
    mat4_get_rot(pose_cw, rot_cw);
    mat4_get_trans(pose_cw, trans_cw);
    sv_mat3_transpose(rot_cw, rot_wc);
    sv_mat3_mulv(rot_wc, trans_cw, trans_wc);
    trans_wc[0] = -trans_wc[0];
    trans_wc[1] = -trans_wc[1];
    trans_wc[2] = -trans_wc[2];
    mat4_set_identity(kf->pose_wc);
    mat4_set_rot(kf->pose_wc, rot_wc);
    mat4_set_trans(kf->pose_wc, trans_wc);
    kf->trans_wc[0] = trans_wc[0];
    kf->trans_wc[1] = trans_wc[1];
    kf->trans_wc[2] = trans_wc[2];
}

void sv_map_keyframe_set_pose_cw(sv_map_keyframe* kf, const double pose_cw_colmajor[16]) {
    keyframe_set_pose_cw(kf, pose_cw_colmajor);
}

void sv_map_orb_params_init(sv_map_orb_params* p, float scale_factor, int num_levels) {
    int level;
    p->scale_factor = scale_factor;
    p->num_levels = num_levels;
    p->scale_factors[0] = 1.0f;
    p->inv_scale_factors[0] = 1.0f;
    for (level = 1; level < num_levels; ++level) {
        p->scale_factors[level] = scale_factor * p->scale_factors[level - 1];
        p->inv_scale_factors[level] = (1.0f / scale_factor) * p->inv_scale_factors[level - 1];
    }
}

void sv_map_keyframe_init(sv_map_keyframe* kf, unsigned int id,
                          const double pose_cw_colmajor[16],
                          const sv_keypoint* keypts, const uint8_t* descriptors,
                          unsigned int num_keypts,
                          const sv_map_orb_params* orb_params) {
    memset(kf, 0, sizeof(*kf));
    kf->id = id;
    kf->keypts = keypts;
    kf->descriptors = descriptors;
    kf->num_keypts = num_keypts;
    kf->orb_params = orb_params;
    kf->spanning_parent_id = -1;
    kf->spanning_root_id = -1;
    keyframe_set_pose_cw(kf, pose_cw_colmajor);
}

/* ---- data::landmark::compute_mean_normal --------------------------------
 * mean_normal = sum_i normalized(pos_w - trans_wc_i); result normalized
 * again. normalized() = x / sqrt(L(x.x)) (division elementwise, per
 * HANDOVER.md); the subtraction/addition/division themselves are plain
 * elementwise ops (no reduction => no evaluation-order ambiguity). */
static void vec3_normalize(const double v[3], double out[3]) {
    double n = sv_vec3_norm(v);
    out[0] = v[0] / n;
    out[1] = v[1] / n;
    out[2] = v[2] / n;
}

static void landmark_compute_mean_normal(const sv_map_landmark* lm,
                                         const sv_map_keyframe* kfs[2],
                                         double mean_normal[3]) {
    unsigned int i;
    mean_normal[0] = mean_normal[1] = mean_normal[2] = 0.0;
    for (i = 0; i < lm->num_observations; ++i) {
        const sv_map_keyframe* kf = kfs[i];
        double normal[3], nn[3];
        normal[0] = lm->pos_w[0] - kf->trans_wc[0];
        normal[1] = lm->pos_w[1] - kf->trans_wc[1];
        normal[2] = lm->pos_w[2] - kf->trans_wc[2];
        vec3_normalize(normal, nn);
        mean_normal[0] += nn[0];
        mean_normal[1] += nn[1];
        mean_normal[2] += nn[2];
    }
    {
        double out[3];
        vec3_normalize(mean_normal, out);
        mean_normal[0] = out[0];
        mean_normal[1] = out[1];
        mean_normal[2] = out[2];
    }
}

/* ---- data::landmark::compute_orb_scale_variance -------------------------
 * max_valid_dist = ||pos_w - ref_keyfrm.trans_wc|| (double) * scale_factor
 * (float) -> narrowed to float; min_valid_dist = max_valid_dist (float) *
 * inv_scale_factors[num_levels-1] (float), a true single-precision
 * multiply (both operands already float). */
static void landmark_compute_orb_scale_variance(const sv_map_landmark* lm,
                                                 const sv_map_keyframe* ref_kf,
                                                 unsigned int ref_idx,
                                                 float* max_valid_dist, float* min_valid_dist) {
    double vec[3];
    double dist;
    int octave;
    float scale_factor;
    vec[0] = lm->pos_w[0] - ref_kf->trans_wc[0];
    vec[1] = lm->pos_w[1] - ref_kf->trans_wc[1];
    vec[2] = lm->pos_w[2] - ref_kf->trans_wc[2];
    dist = sv_vec3_norm(vec);
    octave = ref_kf->keypts[ref_idx].octave;
    scale_factor = ref_kf->orb_params->scale_factors[octave];
    *max_valid_dist = (float)(dist * (double)scale_factor);
    *min_valid_dist = (*max_valid_dist) * ref_kf->orb_params->inv_scale_factors[ref_kf->orb_params->num_levels - 1];
}

/* Finds the observation (of at most 2) belonging to a given keyframe id;
 * returns its idx, or -1. */
static int landmark_obs_idx_in(const sv_map_landmark* lm, unsigned int kf_id) {
    unsigned int i;
    for (i = 0; i < lm->num_observations; ++i) {
        if (lm->observations[i].keyframe_id == kf_id) return (int)lm->observations[i].idx;
    }
    return -1;
}

static void landmark_update_mean_normal_and_scale(sv_map_landmark* lm,
                                                   const sv_map_keyframe* init_kf,
                                                   const sv_map_keyframe* curr_kf) {
    const sv_map_keyframe* kfs[2];
    const sv_map_keyframe* ref_kf;
    int ref_idx;
    unsigned int i;
    for (i = 0; i < lm->num_observations; ++i) {
        kfs[i] = (lm->observations[i].keyframe_id == init_kf->id) ? init_kf : curr_kf;
    }
    landmark_compute_mean_normal(lm, kfs, lm->mean_normal);

    ref_kf = (lm->ref_keyfrm_id == init_kf->id) ? init_kf : curr_kf;
    ref_idx = landmark_obs_idx_in(lm, ref_kf->id);
    landmark_compute_orb_scale_variance(lm, ref_kf, (unsigned int)ref_idx,
                                        &lm->max_valid_dist, &lm->min_valid_dist);
}

/* ---- data::landmark::compute_descriptor ---------------------------------
 * Delegates selection to Codex's sv_landmark_select_descriptor
 * (sv_landmark_descriptor.h) -- see that header for the exact tie rule
 * (increasing keyframe id wins; matches this landmark's at-most-2-entry
 * observation map, which is already id-ordered by construction below). */
static void landmark_compute_descriptor(sv_map_landmark* lm,
                                        const sv_map_keyframe* init_kf,
                                        const sv_map_keyframe* curr_kf) {
    sv_descriptor_observation obs[8];
    sv_landmark_descriptor_result res;
    unsigned int i;
    for (i = 0; i < lm->num_observations; ++i) {
        const sv_map_keyframe* kf = (lm->observations[i].keyframe_id == init_kf->id) ? init_kf : curr_kf;
        obs[i].keyframe_id = lm->observations[i].keyframe_id;
        obs[i].descriptor = kf->descriptors + (size_t)lm->observations[i].idx * 32;
        obs[i].erased = 0;
    }
    sv_landmark_select_descriptor(obs, lm->num_observations, &res, NULL);
    memcpy(lm->descriptor, res.descriptor, 32);
}

/* connect_to_keyframe: keyframe::add_landmark then landmark::add_observation.
 * add_observation asserts an id-ordered observations_t (std::map keyed by
 * id_less<weak_ptr<keyframe>>) -- module 4a only ever has 2 observations
 * per landmark (init_idx then curr_idx, ids 0 then 1), so appending in
 * call order already keeps the array id-ordered; no separate sort needed. */
static void landmark_connect_to_keyframe(sv_map_landmark* lm, unsigned int kf_id, unsigned int idx) {
    lm->observations[lm->num_observations].keyframe_id = kf_id;
    lm->observations[lm->num_observations].idx = idx;
    lm->num_observations++;
}

unsigned int sv_map_build_pre_ba(
    unsigned int ref_frame_id, unsigned int cur_frame_id,
    const sv_keypoint* ref_keypts, const uint8_t* ref_descriptors, unsigned int num_kp_ref,
    const sv_keypoint* cur_keypts, const uint8_t* cur_descriptors, unsigned int num_kp_cur,
    const sv_map_orb_params* orb_params,
    const double rot_ref_to_cur[9], const double trans_ref_to_cur[3],
    const int* init_matches, const unsigned char* is_triangulated,
    const double* triangulated_pts,
    sv_map_landmark* landmarks_out,
    sv_map_init_map* map) {
    unsigned int i;
    unsigned int next_landmark_id = 0;
    double identity[16], cur_pose[16];
    (void)ref_frame_id;
    (void)cur_frame_id;

    mat4_set_identity(identity);
    sv_map_keyframe_init(&map->init_keyfrm, 0, identity, ref_keypts, ref_descriptors, num_kp_ref, orb_params);

    mat4_set_identity(cur_pose);
    mat4_set_rot(cur_pose, rot_ref_to_cur);
    mat4_set_trans(cur_pose, trans_ref_to_cur);
    sv_map_keyframe_init(&map->curr_keyfrm, 1, cur_pose, cur_keypts, cur_descriptors, num_kp_cur, orb_params);

    /* spanning tree: curr's parent = init; init's child = curr; both roots = init. */
    map->curr_keyfrm.spanning_parent_id = 0;
    map->init_keyfrm.spanning_root_id = 0;
    map->curr_keyfrm.spanning_root_id = 0;
    map->init_keyfrm.spanning_children[0] = 1;
    map->init_keyfrm.num_spanning_children = 1;

    map->landmarks = landmarks_out;
    map->num_landmarks = 0;

    for (i = 0; i < num_kp_ref; ++i) {
        int curr_idx = init_matches[i];
        sv_map_landmark* lm;
        if (curr_idx < 0) continue;
        if (!is_triangulated[i]) continue; /* module::initializer's own
                                             * invalidation loop is folded
                                             * into this check: matches
                                             * lacking a triangulated
                                             * point never reach here. */

        lm = &map->landmarks[map->num_landmarks];
        memset(lm, 0, sizeof(*lm));
        lm->id = next_landmark_id++;
        lm->first_keyfrm_id = map->curr_keyfrm.id; /* landmark ctor: first_keyfrm_id_ = ref_keyfrm->id_,
                                                     * and ref_keyfrm passed to the ctor is curr_keyfrm --
                                                     * see module/initializer.cc create_map_for_monocular. */
        lm->pos_w[0] = triangulated_pts[(size_t)i * 3 + 0];
        lm->pos_w[1] = triangulated_pts[(size_t)i * 3 + 1];
        lm->pos_w[2] = triangulated_pts[(size_t)i * 3 + 2];
        lm->ref_keyfrm_id = map->curr_keyfrm.id;
        lm->num_observable = 1;
        lm->num_observed = 1;

        /* connect_to_keyframe(init_keyfrm, init_idx) then
         * connect_to_keyframe(curr_keyfrm, curr_idx), in that exact
         * program order (module/initializer.cc). */
        landmark_connect_to_keyframe(lm, map->init_keyfrm.id, i);
        landmark_connect_to_keyframe(lm, map->curr_keyfrm.id, (unsigned int)curr_idx);

        landmark_compute_descriptor(lm, &map->init_keyfrm, &map->curr_keyfrm);
        landmark_update_mean_normal_and_scale(lm, &map->init_keyfrm, &map->curr_keyfrm);

        map->num_landmarks++;
    }

    map->median_scale = 0.0f;
    map->inv_median_scale = 0.0;
    map->applied_scale = 0.0;
    map->reset_wrong_init = 0;
    return map->num_landmarks;
}

/* ---- data::keyframe::compute_median_depth(abs=true) ---------------------
 * depth_i = (rot_cw.row(2) . pos_w)(double) + (float)pose_cw(2,3), abs'd
 * in double, narrowed to float on push_back; sort ascending; pick
 * depths[(n-1)/2] -- iterated over init_keyfrm's OWN landmarks_ array,
 * which for module 4a is exactly every landmark (all are observed by
 * init_keyfrm; see keyframe::get_landmarks()/compute_median_depth). */
static int cmp_float(const void* a, const void* b) {
    float fa = *(const float*)a, fb = *(const float*)b;
    return (fa > fb) - (fa < fb);
}

static float keyframe_compute_median_depth_abs(const sv_map_keyframe* kf,
                                                const sv_map_landmark* lms, unsigned int n) {
    float* depths = (float*)malloc(sizeof(float) * (n ? n : 1));
    double rot_row2[3];
    float trans_cw_z;
    unsigned int i, count = 0;
    float result;
    rot_row2[0] = M4(kf->pose_cw, 2, 0);
    rot_row2[1] = M4(kf->pose_cw, 2, 1);
    rot_row2[2] = M4(kf->pose_cw, 2, 2);
    trans_cw_z = (float)M4(kf->pose_cw, 2, 3);
    for (i = 0; i < n; ++i) {
        double pos_c_z = sv_vec3_dot(rot_row2, lms[i].pos_w) + (double)trans_cw_z;
        depths[count++] = (float)fabs(pos_c_z);
    }
    qsort(depths, count, sizeof(float), cmp_float);
    result = depths[(count - 1) / 2];
    free(depths);
    return result;
}

/* module::initializer::scale_map(): curr_keyfrm's translation *= scale;
 * every landmark's pos_w *= scale, then update_mean_normal_and_obs_scale_
 * variance() again (private orchestration method -- reimplemented here as
 * a plain sequence of calls to the same public setters/recomputation this
 * file already ports, no new math beyond scalar multiplication). */
void sv_map_apply_post_ba(sv_map_init_map* map,
                          unsigned int min_num_triangulated_pts,
                          double scaling_factor) {
    unsigned int i;
    unsigned int num_tracked;

    for (i = 0; i < map->num_landmarks; ++i) {
        landmark_update_mean_normal_and_scale(&map->landmarks[i], &map->init_keyfrm, &map->curr_keyfrm);
    }

    map->median_scale = keyframe_compute_median_depth_abs(&map->init_keyfrm, map->landmarks, map->num_landmarks);
    map->inv_median_scale = 1.0 / (double)map->median_scale;

    /* curr_keyfrm->get_num_tracked_landmarks(1): min_num_obs_thr=1 (>0)
     * branch -- every module-4a landmark has num_observations_==2 (both
     * observing keyframes are non-stereo, so add_observation's stereo
     * branch never fires) or ==1 for stereo dummies (n/a here), always
     * >=1, so every landmark counts; will_be_erased is always false at
     * this point. */
    num_tracked = map->num_landmarks;
    map->reset_wrong_init = (num_tracked < min_num_triangulated_pts) && (map->median_scale < 0.0f);
    if (map->reset_wrong_init) {
        return;
    }

    map->applied_scale = map->inv_median_scale * scaling_factor;

    {
        double trans[3];
        mat4_get_trans(map->curr_keyfrm.pose_cw, trans);
        trans[0] *= map->applied_scale;
        trans[1] *= map->applied_scale;
        trans[2] *= map->applied_scale;
        {
            double new_pose[16];
            memcpy(new_pose, map->curr_keyfrm.pose_cw, sizeof(new_pose));
            mat4_set_trans(new_pose, trans);
            keyframe_set_pose_cw(&map->curr_keyfrm, new_pose);
        }
    }

    for (i = 0; i < map->num_landmarks; ++i) {
        sv_map_landmark* lm = &map->landmarks[i];
        lm->pos_w[0] *= map->applied_scale;
        lm->pos_w[1] *= map->applied_scale;
        lm->pos_w[2] *= map->applied_scale;
        landmark_update_mean_normal_and_scale(lm, &map->init_keyfrm, &map->curr_keyfrm);
    }
}
