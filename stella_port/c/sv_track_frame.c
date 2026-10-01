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

/* stella_vslam e445b545: data/frame.cc, data/keyframe.cc (set_pose_cw),
 * data/landmark.h (predict helpers live in sv_tracking.c), camera/perspective.cc
 * (reproject_to_image), feature/orb_params.cc, util/angle.cc, match/base.h.
 * Common data-model plumbing of the module-5 tracking port (see sv_track.h). */
#include "sv_track.h"
#include "sv_linalg.h"
#include <math.h>
#include <stdlib.h>
#include <string.h>

/* feature::orb_params(name, scale_factor, num_levels, ...) tables. */
void sv_tr_config_init(sv_tr_config* cfg, double fx, double fy, double cx, double cy,
                       const sv_image_bounds* bounds, const sv_bow_vocab* vocab) {
    unsigned int level;
    float scale_factor_at_level;
    memset(cfg, 0, sizeof(*cfg));
    cfg->fx = fx;
    cfg->fy = fy;
    cfg->cx = cx;
    cfg->cy = cy;
    cfg->bounds = *bounds;
    cfg->num_grid_cols = 64;
    cfg->num_grid_rows = 48;
    cfg->scale_factor = 1.2f;
    cfg->num_levels = 8;
    cfg->log_scale_factor = logf(cfg->scale_factor);
    cfg->scale_factors[0] = 1.0f;
    cfg->inv_scale_factors[0] = 1.0f;
    for (level = 1; level < cfg->num_levels; ++level) {
        cfg->scale_factors[level] = cfg->scale_factor * cfg->scale_factors[level - 1];
        cfg->inv_scale_factors[level] = (1.0f / cfg->scale_factor) * cfg->inv_scale_factors[level - 1];
    }
    scale_factor_at_level = 1.0f;
    cfg->inv_level_sigma_sq[0] = 1.0f;
    for (level = 1; level < cfg->num_levels; ++level) {
        scale_factor_at_level = cfg->scale_factor * scale_factor_at_level;
        cfg->inv_level_sigma_sq[level] = 1.0f / (scale_factor_at_level * scale_factor_at_level);
    }

    cfg->num_matches_thr = 10;
    cfg->margin_last_frame_projection = 20.0f;
    cfg->margin_local_map_projection = 5.0f;
    cfg->margin_local_map_projection_unstable = 20.0f;
    cfg->max_num_local_keyfrms = 60;
    cfg->pose_opt.num_trials_robust = 2;
    cfg->pose_opt.num_trials = 2;
    cfg->pose_opt.num_each_iter = 10;

    cfg->max_interval = 1.0;
    cfg->min_interval = 0.1;
    cfg->max_distance = -1.0;
    cfg->min_distance = -1.0;
    cfg->lms_ratio_thr_almost_all_lms_are_tracked = 0.9;
    cfg->lms_ratio_thr_view_changed = 0.5;
    cfg->enough_lms_thr = 100;
    cfg->vocab = vocab;
}

int sv_tr_obs_init(sv_tr_obs* o, const sv_tr_config* cfg, const sv_keypoint* kp,
                   const uint8_t* desc, unsigned int num_kp) {
    memset(o, 0, sizeof(*o));
    o->num_kp = num_kp;
    o->kp = kp;
    o->desc = desc;
    return sv_frame_build_grid(kp, num_kp, &cfg->bounds, cfg->num_grid_cols, cfg->num_grid_rows, &o->grid);
}

void sv_tr_obs_free(sv_tr_obs* o) {
    sv_frame_grid_free(&o->grid);
    if (o->bow_ready) {
        sv_bow_feat_vector_free(&o->bow_feat);
    }
    free(o->bearings);
    memset(o, 0, sizeof(*o));
}

int sv_tr_obs_ensure_bow(sv_tr_obs* o, const sv_tr_config* cfg) {
    sv_bow_vector vec;
    if (o->bow_ready) {
        return 0;
    }
    memset(&vec, 0, sizeof(vec));
    memset(&o->bow_feat, 0, sizeof(o->bow_feat));
    if (sv_bow_transform(cfg->vocab, o->desc, o->num_kp, 4, &vec, &o->bow_feat) != 0) {
        return -1;
    }
    sv_bow_vector_free(&vec);
    o->bow_ready = 1;
    return 0;
}

/* camera::perspective::convert_point_to_bearing (per keypoint):
 *   x = (pt.x - cx_) / fx_;  y = (pt.y - cy_) / fy_;  l2 = sqrt(x*x + y*y + 1.0);
 *   bearing = {x / l2, y / l2, 1.0 / l2}   (float pt widened to double) */
int sv_tr_obs_ensure_bearings(sv_tr_obs* o, const sv_tr_config* cfg) {
    unsigned int i;
    if (o->bearings) {
        return 0;
    }
    o->bearings = (double*)malloc((o->num_kp ? o->num_kp : 1) * 3 * sizeof(double));
    if (!o->bearings) {
        return -1;
    }
    for (i = 0; i < o->num_kp; ++i) {
        const double x_normalized = ((double)o->kp[i].x - cfg->cx) / cfg->fx;
        const double y_normalized = ((double)o->kp[i].y - cfg->cy) / cfg->fy;
        const double l2_norm = sqrt(x_normalized * x_normalized + y_normalized * y_normalized + 1.0);
        o->bearings[3 * i + 0] = x_normalized / l2_norm;
        o->bearings[3 * i + 1] = y_normalized / l2_norm;
        o->bearings[3 * i + 2] = 1.0 / l2_norm;
    }
    return 0;
}

void sv_tr_frame_init(sv_tr_frame* f, unsigned int id, double timestamp, sv_tr_obs* obs) {
    unsigned int i;
    memset(f, 0, sizeof(*f));
    f->id = id;
    f->timestamp = timestamp;
    f->obs = obs;
    f->ref_kf = SV_TR_NONE;
    f->lm = (int*)malloc((obs->num_kp ? obs->num_kp : 1) * sizeof(int));
    for (i = 0; i < obs->num_kp; ++i) {
        f->lm[i] = SV_TR_NONE;
    }
}

void sv_tr_frame_free(sv_tr_frame* f) {
    free(f->lm);
    memset(f, 0, sizeof(*f));
}

int sv_tr_frame_copy(sv_tr_frame* dst, const sv_tr_frame* src) {
    int* lm = (int*)malloc((src->obs->num_kp ? src->obs->num_kp : 1) * sizeof(int));
    if (!lm) {
        return -1;
    }
    memcpy(lm, src->lm, src->obs->num_kp * sizeof(int));
    free(dst->lm);
    *dst = *src;
    dst->lm = lm;
    return 0;
}

/* data::frame::set_pose_cw:
 *   rot_cw_ = pose_cw_.block<3,3>(0,0);  rot_wc_ = rot_cw_.transpose();
 *   trans_cw_ = pose_cw_.block<3,1>(0,3);
 *   trans_wc_ = -rot_cw_.transpose() * trans_cw_;   (lazy transpose: all rows
 *   left-associative, then negated -- sv_linalg.h / HANDOVER Eigen rules) */
void sv_tr_frame_set_pose_cw(sv_tr_frame* f, const double pose_cw[16]) {
    int r, c;
    double t[3];
    f->pose_valid = 1;
    memcpy(f->pose_cw, pose_cw, 16 * sizeof(double));
    for (c = 0; c < 3; ++c) {
        for (r = 0; r < 3; ++r) {
            f->rot_cw[c * 3 + r] = pose_cw[c * 4 + r];
        }
    }
    sv_mat3_transpose(f->rot_cw, f->rot_wc);
    for (r = 0; r < 3; ++r) {
        f->trans_cw[r] = pose_cw[3 * 4 + r];
    }
    sv_mat3_mulv_lhs_transposed(f->rot_cw, f->trans_cw, t);
    f->trans_wc[0] = -t[0];
    f->trans_wc[1] = -t[1];
    f->trans_wc[2] = -t[2];
}

/* data::keyframe::set_pose_cw:
 *   rot_wc = rot_cw.transpose();  (materialized Mat33_t)
 *   trans_wc_ = -rot_wc * trans_cw;  (plain matrix: rows 0-1 L, row 2 R; negated)
 *   pose_wc_ = Identity; block<3,3> = rot_wc; block<3,1> = trans_wc_ */
void sv_tr_kf_set_pose_cw(sv_tr_kf* k, const double pose_cw[16]) {
    int r, c;
    double rot_cw[9], rot_wc[9], trans_cw[3], t[3];
    memcpy(k->pose_cw, pose_cw, 16 * sizeof(double));
    for (c = 0; c < 3; ++c) {
        for (r = 0; r < 3; ++r) {
            rot_cw[c * 3 + r] = pose_cw[c * 4 + r];
        }
    }
    for (r = 0; r < 3; ++r) {
        trans_cw[r] = pose_cw[3 * 4 + r];
    }
    sv_mat3_transpose(rot_cw, rot_wc);
    sv_mat3_mulv(rot_wc, trans_cw, t);
    k->trans_wc[0] = -t[0];
    k->trans_wc[1] = -t[1];
    k->trans_wc[2] = -t[2];
    for (c = 0; c < 16; ++c) {
        k->pose_wc[c] = 0.0;
    }
    for (c = 0; c < 3; ++c) {
        for (r = 0; r < 3; ++r) {
            k->pose_wc[c * 4 + r] = rot_wc[c * 3 + r];
        }
    }
    for (r = 0; r < 3; ++r) {
        k->pose_wc[3 * 4 + r] = k->trans_wc[r];
    }
    k->pose_wc[0 * 4 + 3] = 0.0;
    k->pose_wc[1 * 4 + 3] = 0.0;
    k->pose_wc[2 * 4 + 3] = 0.0;
    k->pose_wc[3 * 4 + 3] = 1.0;
}

const sv_tr_lm* sv_tr_map_lm(const sv_tr_map* m, int id) {
    if (id < 0 || (unsigned int)id >= m->lm_cap) {
        return NULL;
    }
    return m->lms[id] && m->lms[id]->alive ? m->lms[id] : NULL;
}

const sv_tr_kf* sv_tr_map_kf(const sv_tr_map* m, int id) {
    if (id < 0 || (unsigned int)id >= m->kf_cap) {
        return NULL;
    }
    return m->kfs[id] && m->kfs[id]->alive ? m->kfs[id] : NULL;
}

const sv_tr_kf* sv_tr_map_kf_any(const sv_tr_map* m, int id) {
    const sv_tr_kf* k = sv_tr_map_kf(m, id);
    if (k) {
        return k;
    }
    if (m->kf_pool && id >= 0 && (unsigned int)id < m->kf_pool_cap && m->kf_pool[id] && m->kf_pool[id]->obs) {
        return m->kf_pool[id];
    }
    return NULL;
}

/* match::compute_descriptor_distance (ORB, 32 bytes): plain popcount of the
 * xor; upstream's SWAR is bit-exact equal. */
unsigned int sv_tr_hamming(const uint8_t* a, const uint8_t* b) {
    unsigned int d = 0, i;
    for (i = 0; i < SV_TR_DESC_BYTES; ++i) {
        unsigned int v = (unsigned int)(a[i] ^ b[i]);
        v = v - ((v >> 1) & 0x55u);
        v = (v & 0x33u) + ((v >> 2) & 0x33u);
        d += (v + (v >> 4)) & 0x0fu;
    }
    return d;
}

/* util::angle::diff (float in/out; the wrap constants are double literals). */
float sv_tr_angle_diff(float a, float b) {
    float ret = a - b;
    if (ret <= -180.0) {
        ret = (float)(ret + 360.0);
    }
    if (ret > 180.0) {
        ret = (float)(ret - 360.0);
    }
    return ret;
}

/* camera::perspective::reproject_to_image:
 *   pos_c = rot_cw * pos_w + trans_cw;  if (pos_c(2) <= 0) return false;
 *   z_inv = 1.0 / pos_c(2);
 *   reproj(0) = fx_ * pos_c(0) * z_inv + cx_;  reproj(1) = fy_ * pos_c(1) * z_inv + cy_;
 *   return img_bounds_.min_x_ < reproj(0) < max_x_ && min_y_ < reproj(1) < max_y_ */
int sv_tr_reproject_to_image(const sv_tr_config* cfg, const double rot_cw[9], const double trans_cw[3],
                             const double pos_w[3], double reproj[2], float* x_right) {
    double pos_c[3], z_inv;
    sv_mat3_mulv(rot_cw, pos_w, pos_c);
    pos_c[0] = pos_c[0] + trans_cw[0];
    pos_c[1] = pos_c[1] + trans_cw[1];
    pos_c[2] = pos_c[2] + trans_cw[2];
    if (pos_c[2] <= 0.0) {
        return 0;
    }
    z_inv = 1.0 / pos_c[2];
    reproj[0] = cfg->fx * pos_c[0] * z_inv + cfg->cx;
    reproj[1] = cfg->fy * pos_c[1] * z_inv + cfg->cy;
    *x_right = (float)reproj[0]; /* monocular: focal_x_baseline_ == 0, value unused */
    return (cfg->bounds.min_x < reproj[0] && reproj[0] < cfg->bounds.max_x &&
            cfg->bounds.min_y < reproj[1] && reproj[1] < cfg->bounds.max_y);
}
