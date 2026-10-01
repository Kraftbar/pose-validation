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

/* stella_vslam e445b545: match/bow_tree.cc (match_for_triangulation),
 * match/base.h (check_epipolar_constraint), match/fuse.cc (detect_duplication),
 * camera/perspective.cc (reproject_to_bearing), solve/essential_solver.cc
 * (create_E_21), solve/triangulator.h (Mat44 pose overload),
 * module/two_view_triangulator.{h,cc}, data/keyframe.cc (compute_median_depth).
 * Matching / geometry half of the module-6 mapping port (see sv_mapping.h). */
#include "sv_mapping.h"
#include "sv_eigen_svd.h"
#include "sv_linalg.h"
#include <math.h>
#include <stdlib.h>
#include <string.h>

#define HAMMING_DIST_THR_LOW 50u
#define MAX_HAMMING_DIST 256u

static void pose_rot(const double pose[16], double rot[9]) {
    int r, c;
    for (c = 0; c < 3; ++c) {
        for (r = 0; r < 3; ++r) {
            rot[c * 3 + r] = pose[c * 4 + r];
        }
    }
}

static void pose_trans(const double pose[16], double t[3]) {
    t[0] = pose[3 * 4 + 0];
    t[1] = pose[3 * 4 + 1];
    t[2] = pose[3 * 4 + 2];
}

/* solve::essential_solver::create_E_21:
 *   rot_21 = rot_2w * rot_1w.transpose();  trans_21 = -rot_21 * trans_1w + trans_2w;
 *   E = skew(trans_21) * rot_21 */
void sv_map_create_E_21(const double rot_1w[9], const double trans_1w[3], const double rot_2w[9],
                        const double trans_2w[3], double E[9]) {
    double r1t[9], r21[9], mt[3], t21[3], skew[9];
    sv_mat3_transpose(rot_1w, r1t);
    sv_mat3_mul(rot_2w, r1t, r21);
    sv_mat3_mulv(r21, trans_1w, mt);
    t21[0] = -mt[0] + trans_2w[0];
    t21[1] = -mt[1] + trans_2w[1];
    t21[2] = -mt[2] + trans_2w[2];
    /* util::converter::to_skew_symmetric_mat (column-major) */
    skew[0] = 0.0;     skew[3] = -t21[2]; skew[6] = t21[1];
    skew[1] = t21[2];  skew[4] = 0.0;     skew[7] = -t21[0];
    skew[2] = -t21[1]; skew[5] = t21[0];  skew[8] = 0.0;
    sv_mat3_mul(skew, r21, E);
}

/* solve::triangulator::triangulate(bearing_1, bearing_2, Mat44 cam_pose_1, Mat44 cam_pose_2):
 *   A.row(0) = b1(0) * P1.row(2) - b1(2) * P1.row(0);  A.row(1) = b1(1) * P1.row(2) - b1(2) * P1.row(1);
 *   A.row(2) = b2(0) * P2.row(2) - b2(2) * P2.row(0);  A.row(3) = b2(1) * P2.row(2) - b2(2) * P2.row(1);
 *   V = JacobiSVD<Mat44>(A, FullU|FullV).matrixV();  return V.block<3,1>(0,3) / V(3,3) */
void sv_map_triangulate_poses(const double b1[3], const double b2[3], const double pose1[16],
                              const double pose2[16], double pos[3]) {
    double A[16], U[16], V[16], sv[4];
    int j;
#define P1(r, c) pose1[(c) * 4 + (r)]
#define P2(r, c) pose2[(c) * 4 + (r)]
    for (j = 0; j < 4; ++j) {
        A[j * 4 + 0] = b1[0] * P1(2, j) - b1[2] * P1(0, j);
        A[j * 4 + 1] = b1[1] * P1(2, j) - b1[2] * P1(1, j);
        A[j * 4 + 2] = b2[0] * P2(2, j) - b2[2] * P2(0, j);
        A[j * 4 + 3] = b2[1] * P2(2, j) - b2[2] * P2(1, j);
    }
#undef P1
#undef P2
    sv_eigen_jacobisvd_4x4(A, U, V, sv);
    pos[0] = V[3 * 4 + 0] / V[3 * 4 + 3];
    pos[1] = V[3 * 4 + 1] / V[3 * 4 + 3];
    pos[2] = V[3 * 4 + 2] / V[3 * 4 + 3];
}

static int cmp_float(const void* a, const void* b) {
    float fa = *(const float*)a, fb = *(const float*)b;
    return (fa > fb) - (fa < fb);
}

/* keyframe::compute_median_depth(abs): rot_cw_z_row (Vec3 copy) . pos_w + (float)trans_cw_z,
 * pushed to a float vector, sorted, depths[(n - 1) / 2]. */
float sv_map_kf_median_depth(const sv_tr_map* map, const sv_tr_kf* kf, int use_abs) {
    float* depths = (float*)malloc(sizeof(float) * (kf->obs->num_kp ? kf->obs->num_kp : 1));
    double rot_row2[3];
    float trans_cw_z, result;
    unsigned int i, count = 0;
    rot_row2[0] = kf->pose_cw[0 * 4 + 2];
    rot_row2[1] = kf->pose_cw[1 * 4 + 2];
    rot_row2[2] = kf->pose_cw[2 * 4 + 2];
    trans_cw_z = (float)kf->pose_cw[3 * 4 + 2];
    for (i = 0; i < kf->obs->num_kp; ++i) {
        const sv_tr_lm* lm;
        double pos_c_z;
        if (kf->lm[i] < 0) {
            continue;
        }
        lm = map->lms[kf->lm[i]];
        if (!lm) {
            continue;
        }
        pos_c_z = sv_vec3_dot(rot_row2, lm->pos_w) + (double)trans_cw_z;
        depths[count++] = (float)(use_abs ? fabs(pos_c_z) : pos_c_z);
    }
    qsort(depths, count, sizeof(float), cmp_float);
    result = count ? depths[(count - 1) / 2] : 0.0f;
    free(depths);
    return result;
}

/* first node index whose node_id >= key (std::map::lower_bound) */
static unsigned int node_lower_bound(const sv_bow_feat_vector* fv, unsigned int from, uint32_t key) {
    unsigned int lo = from, hi = fv->count;
    while (lo < hi) {
        const unsigned int mid = lo + (hi - lo) / 2;
        if (fv->nodes[mid].node_id < key) {
            lo = mid + 1;
        } else {
            hi = mid;
        }
    }
    return lo;
}

/* match/base.h check_epipolar_constraint; `thr` is the float product
 * residual_rad_thr * scale_factor (the call site swaps the two float
 * arguments; the product is commutative). */
static int check_epipolar_constraint(const double bearing_1[3], const double bearing_2[3], const double E_12[9],
                                     float thr) {
    double epiplane_in_1[3], q, cos_residual, residual_rad;
    sv_mat3_mulv(E_12, bearing_2, epiplane_in_1);
    q = sv_vec3_dot(epiplane_in_1, bearing_1) / sv_vec3_norm(epiplane_in_1);
    /* std::min(1.0, std::max(-1.0, q)): max(a,b) = a < b ? b : a; min(a,b) = b < a ? b : a */
    cos_residual = (-1.0 < q) ? q : -1.0;
    cos_residual = (cos_residual < 1.0) ? cos_residual : 1.0;
    residual_rad = fabs(3.14159265358979323846 / 2.0 - acos(cos_residual));
    return residual_rad < (double)thr;
}

unsigned int sv_map_match_for_triangulation(const sv_tr_config* cfg, sv_tr_kf* kf1, sv_tr_kf* kf2, const double E_12[9],
                                            float residual_rad_thr, unsigned int (**pairs_out)[2]) {
    const float lowe_ratio = 0.95f;
    unsigned int num_matches = 0;
    double cam_center_1[3], rot_2w[9], trans_2w[3], epi[3];
    int valid_epiplane = 0;
    unsigned char* matched2;
    int* matched_indices_2_in_1;
    unsigned int n1 = kf1->obs->num_kp, n2 = kf2->obs->num_kp;
    unsigned int i1 = 0, i2 = 0, k;
    const sv_bow_feat_vector *f1, *f2;
    unsigned int (*pairs)[2];

    sv_tr_obs_ensure_bow(kf1->obs, cfg);
    sv_tr_obs_ensure_bow(kf2->obs, cfg);
    sv_tr_obs_ensure_bearings(kf1->obs, cfg);
    sv_tr_obs_ensure_bearings(kf2->obs, cfg);
    f1 = &kf1->obs->bow_feat;
    f2 = &kf2->obs->bow_feat;

    cam_center_1[0] = kf1->trans_wc[0];
    cam_center_1[1] = kf1->trans_wc[1];
    cam_center_1[2] = kf1->trans_wc[2];
    pose_rot(kf2->pose_cw, rot_2w);
    pose_trans(kf2->pose_cw, trans_2w);
    /* perspective::reproject_to_bearing */
    {
        double z_inv, x, y, sq;
        sv_mat3_mulv(rot_2w, cam_center_1, epi);
        epi[0] = epi[0] + trans_2w[0];
        epi[1] = epi[1] + trans_2w[1];
        epi[2] = epi[2] + trans_2w[2];
        if (epi[2] <= 0.0) {
            valid_epiplane = 0;
        }
        else {
            z_inv = 1.0 / epi[2];
            x = cfg->fx * epi[0] * z_inv + cfg->cx;
            y = cfg->fy * epi[1] * z_inv + cfg->cy;
            sq = sv_vec3_dot(epi, epi); /* normalize(): z = squaredNorm(); if (z > 0) *this /= sqrt(z) */
            if (sq > 0.0) {
                const double s = sqrt(sq);
                epi[0] = epi[0] / s;
                epi[1] = epi[1] / s;
                epi[2] = epi[2] / s;
            }
            valid_epiplane = ((double)cfg->bounds.min_x < x && x < (double)cfg->bounds.max_x
                              && (double)cfg->bounds.min_y < y && y < (double)cfg->bounds.max_y);
        }
    }

    matched2 = (unsigned char*)calloc(n2 ? n2 : 1, 1);
    matched_indices_2_in_1 = (int*)malloc((n1 ? n1 : 1) * sizeof(int));
    for (k = 0; k < n1; ++k) {
        matched_indices_2_in_1[k] = -1;
    }

    while (i1 < f1->count && i2 < f2->count) {
        const sv_bow_feat_node* a = &f1->nodes[i1];
        const sv_bow_feat_node* b = &f2->nodes[i2];
        if (a->node_id == b->node_id) {
            unsigned int p, q;
            for (p = 0; p < a->count; ++p) {
                const unsigned int idx_1 = a->kp_indices[p];
                unsigned int best_hamm_dist, second_best_hamm_dist;
                int best_idx_2;
                if (kf1->lm[idx_1] >= 0) {
                    continue;
                }
                best_hamm_dist = HAMMING_DIST_THR_LOW;
                best_idx_2 = -1;
                second_best_hamm_dist = MAX_HAMMING_DIST;
                for (q = 0; q < b->count; ++q) {
                    const unsigned int idx_2 = b->kp_indices[q];
                    unsigned int hamm_dist;
                    if (kf2->lm[idx_2] >= 0) {
                        continue;
                    }
                    if (matched2[idx_2]) {
                        continue;
                    }
                    hamm_dist = sv_tr_hamming(kf1->obs->desc + (size_t)idx_1 * SV_TR_DESC_BYTES,
                                              kf2->obs->desc + (size_t)idx_2 * SV_TR_DESC_BYTES);
                    if (HAMMING_DIST_THR_LOW < hamm_dist || best_hamm_dist < hamm_dist) {
                        continue;
                    }
                    if (valid_epiplane) {
                        const double cos_dist = sv_vec3_dot(epi, kf2->obs->bearings + 3 * (size_t)idx_2);
                        const double cos_dist_thr = 0.99862953475;
                        if (cos_dist_thr < cos_dist) {
                            continue;
                        }
                    }
                    {
                        const float thr = cfg->scale_factors[kf1->obs->kp[idx_1].octave] * residual_rad_thr;
                        if (check_epipolar_constraint(kf1->obs->bearings + 3 * (size_t)idx_1,
                                                      kf2->obs->bearings + 3 * (size_t)idx_2, E_12, thr)) {
                            if (hamm_dist < best_hamm_dist) {
                                second_best_hamm_dist = best_hamm_dist;
                                best_hamm_dist = hamm_dist;
                                best_idx_2 = (int)idx_2;
                            }
                            else if (hamm_dist < second_best_hamm_dist) {
                                second_best_hamm_dist = hamm_dist;
                            }
                        }
                    }
                }
                if (best_idx_2 < 0) {
                    continue;
                }
                if (lowe_ratio * (float)second_best_hamm_dist < (float)best_hamm_dist) {
                    continue;
                }
                matched2[best_idx_2] = 1;
                matched_indices_2_in_1[idx_1] = best_idx_2;
                ++num_matches;
            }
            ++i1;
            ++i2;
        }
        else if (a->node_id < b->node_id) {
            i1 = node_lower_bound(f1, i1, b->node_id);
        }
        else {
            i2 = node_lower_bound(f2, i2, a->node_id);
        }
    }

    pairs = (unsigned int(*)[2])malloc((num_matches ? num_matches : 1) * sizeof(unsigned int[2]));
    {
        unsigned int n = 0;
        for (k = 0; k < n1; ++k) {
            if (matched_indices_2_in_1[k] < 0) {
                continue;
            }
            pairs[n][0] = k;
            pairs[n][1] = (unsigned int)matched_indices_2_in_1[k];
            ++n;
        }
    }
    free(matched2);
    free(matched_indices_2_in_1);
    *pairs_out = pairs;
    return num_matches;
}
