/* SPDX-License-Identifier: BSD-2-Clause */
/* See sv_init.h (BSD-2, AIST 2019 + stella-cv 2022). */
#include "sv_init.h"
#include "sv_solve_homography.h"
#include "sv_solve_fundamental.h"
#include "sv_linalg.h"
#include <math.h>
#include <stdlib.h>
#include <string.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#define SV_COS_PARALLAX_THR 0.99996192306f

static int cmp_float(const void* a, const void* b) {
    float fa = *(const float*)a, fb = *(const float*)b;
    return (fa > fb) - (fa < fb);
}

/* initialize::base::triangulate(): returns num_valid_pts, fills
 * triangulated_pts/is_triangulated (size num_kp_ref) and *num_tri/*parallax. */
static float g_par_frac; /* stella_vio: > 0: the parallax is taken at this fraction of the valid points instead of the 50th */
static unsigned int sv_base_triangulate(
    const double rot[9], const double trans[3],
    const unsigned char* is_inlier, /* size num_matches, indexed like ref_idx/cur_idx arrays */
    const unsigned int* midx_ref, const unsigned int* midx_cur, unsigned int num_matches,
    const sv_keypoint* ref_keypts, unsigned int num_kp_ref, const double* ref_bearings,
    const sv_keypoint* cur_keypts, const double* cur_bearings,
    const sv_camera_perspective* ref_cam, const sv_camera_perspective* cur_cam,
    float reproj_err_thr,
    double* triangulated_pts, unsigned char* is_triangulated,
    unsigned int* num_triangulated_pts, float* parallax_cos_out) {
    float reproj_err_thr_sq = reproj_err_thr * reproj_err_thr;
    double cur_cam_center[3];
    double neg_trans[3];
    unsigned int num_valid_pts = 0, num_tri = 0;
    float* cos_parallaxes = (float*)malloc(sizeof(float) * (num_matches ? num_matches : 1));
    unsigned int n_cp = 0;
    unsigned int i;

    memset(is_triangulated, 0, num_kp_ref);

    /* rot_ref_to_cur.transpose() used inline in the real source
     * (`-rot_ref_to_cur.transpose() * trans_ref_to_cur`): ALL rows
     * left-associative (see sv_mat3_mulv_lhs_transposed). */
    neg_trans[0] = -trans[0]; neg_trans[1] = -trans[1]; neg_trans[2] = -trans[2];
    sv_mat3_mulv_lhs_transposed(rot, neg_trans, cur_cam_center);

    for (i = 0; i < num_matches; ++i) {
        double pos_c_in_ref[3];
        double ref_normal[3], cur_normal[3];
        double ref_norm_d, cur_norm_d;
        float ref_norm, cur_norm;
        float cos_parallax;
        int parallax_is_small;
        unsigned int ref_idx, cur_idx;

        if (!is_inlier[i]) {
            continue;
        }
        ref_idx = midx_ref[i];
        cur_idx = midx_cur[i];

        sv_triangulate_bearings(&ref_bearings[ref_idx * 3], &cur_bearings[cur_idx * 3], rot, trans, pos_c_in_ref);

        if (!isfinite(pos_c_in_ref[0]) || !isfinite(pos_c_in_ref[1]) || !isfinite(pos_c_in_ref[2])) {
            continue;
        }

        ref_normal[0] = pos_c_in_ref[0]; ref_normal[1] = pos_c_in_ref[1]; ref_normal[2] = pos_c_in_ref[2];
        ref_norm_d = sv_vec3_norm(ref_normal);
        ref_norm = (float)ref_norm_d;
        cur_normal[0] = pos_c_in_ref[0] - cur_cam_center[0];
        cur_normal[1] = pos_c_in_ref[1] - cur_cam_center[1];
        cur_normal[2] = pos_c_in_ref[2] - cur_cam_center[2];
        cur_norm_d = sv_vec3_norm(cur_normal);
        cur_norm = (float)cur_norm_d;

        {
            double dotv = sv_vec3_dot(ref_normal, cur_normal);
            double denom = (double)(ref_norm * cur_norm);
            cos_parallax = (float)(dotv / denom);
        }
        parallax_is_small = SV_COS_PARALLAX_THR < cos_parallax;

        if (!parallax_is_small && pos_c_in_ref[2] <= 0.0) {
            continue;
        }
        {
            double pos_c_in_cur[3];
            sv_mat3_mulv(rot, pos_c_in_ref, pos_c_in_cur);
            pos_c_in_cur[0] += trans[0]; pos_c_in_cur[1] += trans[1]; pos_c_in_cur[2] += trans[2];
            if (!parallax_is_small && pos_c_in_cur[2] <= 0.0) {
                continue;
            }
        }

        {
            double reproj_ref[2];
            int is_valid_ref;
            double identity[9] = {1, 0, 0, 0, 1, 0, 0, 0, 1};
            double zero3[3] = {0, 0, 0};
            float ref_reproj_err_sq;

            is_valid_ref = sv_camera_reproject_to_image(ref_cam, identity, zero3, pos_c_in_ref, reproj_ref);
            if (!parallax_is_small && !is_valid_ref) {
                continue;
            }
            {
                double dx = reproj_ref[0] - (double)ref_keypts[ref_idx].x;
                double dy = reproj_ref[1] - (double)ref_keypts[ref_idx].y;
                ref_reproj_err_sq = (float)(dx * dx + dy * dy);
            }
            if (reproj_err_thr_sq < ref_reproj_err_sq) {
                continue;
            }
        }
        {
            double reproj_cur[2];
            int is_valid_cur;
            float cur_reproj_err_sq;

            is_valid_cur = sv_camera_reproject_to_image(cur_cam, rot, trans, pos_c_in_ref, reproj_cur);
            if (!parallax_is_small && !is_valid_cur) {
                continue;
            }
            {
                double dx = reproj_cur[0] - (double)cur_keypts[cur_idx].x;
                double dy = reproj_cur[1] - (double)cur_keypts[cur_idx].y;
                cur_reproj_err_sq = (float)(dx * dx + dy * dy);
            }
            if (reproj_err_thr_sq < cur_reproj_err_sq) {
                continue;
            }
        }

        ++num_valid_pts;
        cos_parallaxes[n_cp++] = cos_parallax;

        if (!parallax_is_small) {
            triangulated_pts[ref_idx * 3 + 0] = pos_c_in_ref[0];
            triangulated_pts[ref_idx * 3 + 1] = pos_c_in_ref[1];
            triangulated_pts[ref_idx * 3 + 2] = pos_c_in_ref[2];
            is_triangulated[ref_idx] = 1;
            ++num_tri;
        }
    }

    if (n_cp > 0) {
        unsigned int idx;
        qsort(cos_parallaxes, n_cp, sizeof(float), cmp_float);
        idx = n_cp - 1 < 50 ? n_cp - 1 : 50;
        if (g_par_frac > 0.0f) {
            idx = (unsigned int)(g_par_frac * (float)(n_cp - 1));
        }
        *parallax_cos_out = cos_parallaxes[idx];
    }
    else {
        *parallax_cos_out = 1.0f;
    }

    free(cos_parallaxes);
    *num_triangulated_pts = num_tri;
    return num_valid_pts;
}

static void init_attempt(
    uint32_t seed,
    const sv_keypoint* ref_keypts, unsigned int num_kp_ref, const double* ref_bearings,
    const sv_keypoint* cur_keypts, unsigned int num_kp_cur, const double* cur_bearings,
    const int* ref_matched_2_in_1,
    const sv_camera_perspective* ref_cam, const sv_camera_perspective* cur_cam,
    const double ref_cam_matrix[9], const double cur_cam_matrix[9],
    const sv_init_params* params,
    sv_init_attempt_result* out) {
    unsigned int i, num_matches = 0;
    unsigned int* midx_ref;
    unsigned int* midx_cur;
    sv_match_pair* mpairs;

    (void)num_kp_cur;
    g_par_frac = params->par_frac;

    memcpy(out->matched_2_in_1, ref_matched_2_in_1, sizeof(int) * num_kp_ref);
    for (i = 0; i < num_kp_ref; ++i) {
        if (ref_matched_2_in_1[i] >= 0) {
            num_matches++;
        }
    }
    out->num_matches = num_matches;
    out->h_valid = out->f_valid = 0;
    out->model_chosen = SV_INIT_MODEL_NONE;
    out->num_hyps = 0;

    if (num_matches < params->min_num_valid_pts) {
        out->verdict = SV_INIT_RESET_TOO_FEW_MATCHES;
        return;
    }

    midx_ref = (unsigned int*)malloc(sizeof(unsigned int) * num_matches);
    midx_cur = (unsigned int*)malloc(sizeof(unsigned int) * num_matches);
    mpairs = (sv_match_pair*)malloc(sizeof(sv_match_pair) * num_matches);
    {
        unsigned int k = 0;
        for (i = 0; i < num_kp_ref; ++i) {
            if (ref_matched_2_in_1[i] >= 0) {
                midx_ref[k] = i;
                midx_cur[k] = (unsigned int)ref_matched_2_in_1[i];
                mpairs[k].idx1 = (int)i;
                mpairs[k].idx2 = ref_matched_2_in_1[i];
                ++k;
            }
        }
    }

    {
        sv_mt19937 eh, ef;
        sv_homography_ransac_result hres;
        sv_fundamental_ransac_result fres;
        hres.is_inlier = out->inlier_h;
        fres.is_inlier = out->inlier_f;

        /* perspective::initialize() calls find_via_ransac(num_ransac_iters_, false)
         * -- recompute=false, see initialize/perspective.cc. */
        sv_mt19937_seed(&eh, seed);
        sv_solve_homography_find_via_ransac(ref_keypts, num_kp_ref, cur_keypts, num_kp_cur, mpairs, num_matches,
                                             1.0f, params->num_ransac_iters, 0, &eh, &hres);
        sv_mt19937_seed(&ef, seed);
        sv_solve_fundamental_find_via_ransac(ref_keypts, num_kp_ref, cur_keypts, num_kp_cur, mpairs, num_matches,
                                              1.0f, params->num_ransac_iters, 0, &ef, &fres);

        out->h_valid = hres.solution_valid;
        out->f_valid = fres.solution_valid;
        out->cost_h = hres.best_cost;
        out->cost_f = fres.best_cost;
        memcpy(out->best_H21, hres.best_H21, sizeof(out->best_H21));
        memcpy(out->best_F21, fres.best_F21, sizeof(out->best_F21));
        out->rel_cost_h = out->cost_h / (out->cost_h + out->cost_f);
    }

    {
        int pose_found = 0;
        double rots[8][9], transes[8][3], normals[8][3];
        unsigned int num_hyp = 0;
        const unsigned char* is_inlier_for_tri = NULL;

        if (0.5f > out->rel_cost_h && out->h_valid) {
            if (sv_solve_homography_decompose(out->best_H21, ref_cam_matrix, cur_cam_matrix, rots, transes, normals)) {
                out->model_chosen = SV_INIT_MODEL_H;
                num_hyp = 8;
                is_inlier_for_tri = out->inlier_h;
            }
            else {
                out->verdict = SV_INIT_FAIL_NO_VALID_MODEL;
                free(midx_ref); free(midx_cur); free(mpairs);
                return;
            }
        }
        else if (out->f_valid) {
            double rots4[4][9], transes4[4][3];
            sv_solve_fundamental_decompose(out->best_F21, ref_cam_matrix, cur_cam_matrix, rots4, transes4);
            memcpy(rots, rots4, sizeof(rots4));
            memcpy(transes, transes4, sizeof(transes4));
            out->model_chosen = SV_INIT_MODEL_F;
            num_hyp = 4;
            is_inlier_for_tri = out->inlier_f;
        }
        else {
            out->verdict = SV_INIT_FAIL_NO_VALID_MODEL;
            free(midx_ref); free(midx_cur); free(mpairs);
            return;
        }

        out->num_hyps = num_hyp;
        {
            double** hyp_tri = (double**)malloc(sizeof(double*) * num_hyp);
            unsigned char** hyp_istri = (unsigned char**)malloc(sizeof(unsigned char*) * num_hyp);
            unsigned int* nums_valid = (unsigned int*)malloc(sizeof(unsigned int) * num_hyp);
            unsigned int h;
            unsigned int max_idx = 0;

            for (h = 0; h < num_hyp; ++h) {
                hyp_tri[h] = (double*)calloc(num_kp_ref * 3, sizeof(double));
                hyp_istri[h] = (unsigned char*)calloc(num_kp_ref, 1);
                memcpy(out->hyps[h].rot, rots[h], sizeof(rots[h]));
                memcpy(out->hyps[h].trans, transes[h], sizeof(transes[h]));
                nums_valid[h] = sv_base_triangulate(rots[h], transes[h], is_inlier_for_tri,
                                                     midx_ref, midx_cur, num_matches,
                                                     ref_keypts, num_kp_ref, ref_bearings,
                                                     cur_keypts, cur_bearings,
                                                     ref_cam, cur_cam, params->reproj_err_thr,
                                                     hyp_tri[h], hyp_istri[h],
                                                     &out->hyps[h].num_triangulated_pts,
                                                     &out->hyps[h].parallax_cos);
                out->hyps[h].num_valid_pts = nums_valid[h];
                if (nums_valid[h] > nums_valid[max_idx]) {
                    max_idx = h;
                }
            }

            out->selected_hyp = max_idx;

            if (nums_valid[max_idx] < params->min_num_valid_pts) {
                pose_found = 0;
            }
            else {
                unsigned int num_similars = 0;
                for (h = 0; h < num_hyp; ++h) {
                    if (0.8 * (double)nums_valid[max_idx] < (double)nums_valid[h]) {
                        num_similars++;
                    }
                }
                if (num_similars > 1) {
                    pose_found = 0;
                }
                else if ((double)out->hyps[max_idx].parallax_cos > cos(params->parallax_deg_thr / 180.0 * M_PI)) {
                    pose_found = 0;
                }
                else if (out->hyps[max_idx].num_triangulated_pts < params->min_num_triangulated_pts) {
                    pose_found = 0;
                }
                else {
                    pose_found = 1;
                    memcpy(out->rot_ref_to_cur, rots[max_idx], sizeof(rots[max_idx]));
                    memcpy(out->trans_ref_to_cur, transes[max_idx], sizeof(transes[max_idx]));
                    memcpy(out->triangulated_pts, hyp_tri[max_idx], sizeof(double) * num_kp_ref * 3);
                    memcpy(out->is_triangulated, hyp_istri[max_idx], num_kp_ref);
                }
            }

            for (h = 0; h < num_hyp; ++h) {
                free(hyp_tri[h]);
                free(hyp_istri[h]);
            }
            free(hyp_tri);
            free(hyp_istri);
            free(nums_valid);
        }

        out->verdict = pose_found ? SV_INIT_SUCCESS : SV_INIT_FAIL_POSE_NOT_FOUND;
    }

    free(midx_ref);
    free(midx_cur);
    free(mpairs);
    (void)cmp_float;
}

/* stella_vio: init RANSAC seed robustness (HANDOVER "fr2_xyz gap": the default stream, seed 5489, is an outlier for some pairs).
 * Runs the whole attempt from n_seeds fixed seeds and keeps the SUCCESS result with the most valid points of its selected hypothesis
 * (ties: the earlier seed, so n_seeds = 1 is the exact port); if no seed succeeds the first seed's result (verdict) is returned. Deterministic. */
static const uint32_t seed_list[8] = {5489u, 1u, 2u, 7u, 4u, 5u, 6u, 8u};

void sv_init_try_monocular(
    const sv_keypoint* ref_keypts, unsigned int num_kp_ref, const double* ref_bearings,
    const sv_keypoint* cur_keypts, unsigned int num_kp_cur, const double* cur_bearings,
    const int* ref_matched_2_in_1,
    const sv_camera_perspective* ref_cam, const sv_camera_perspective* cur_cam,
    const double ref_cam_matrix[9], const double cur_cam_matrix[9],
    const sv_init_params* params,
    sv_init_attempt_result* out) {
    unsigned int k, n_seeds = params->num_seeds < 1 ? 1 : (params->num_seeds > 8 ? 8 : params->num_seeds);
    unsigned int nm = 0, i, best_score = 0;
    sv_init_attempt_result tmp;
    int have_best = 0;
    init_attempt(seed_list[0], ref_keypts, num_kp_ref, ref_bearings, cur_keypts, num_kp_cur, cur_bearings, ref_matched_2_in_1,
                 ref_cam, cur_cam, ref_cam_matrix, cur_cam_matrix, params, out);
    if (n_seeds == 1) {
        return;
    }
    for (i = 0; i < num_kp_ref; ++i) {
        nm += ref_matched_2_in_1[i] >= 0;
    }
    if (out->verdict == SV_INIT_SUCCESS) {
        best_score = out->hyps[out->selected_hyp].num_valid_pts;
        have_best = 1;
    }
    tmp = *out;
    tmp.matched_2_in_1 = (int*)malloc(sizeof(int) * (num_kp_ref ? num_kp_ref : 1));
    tmp.inlier_h = (unsigned char*)malloc(nm ? nm : 1);
    tmp.inlier_f = (unsigned char*)malloc(nm ? nm : 1);
    tmp.triangulated_pts = (double*)malloc(sizeof(double) * 3 * (num_kp_ref ? num_kp_ref : 1));
    tmp.is_triangulated = (unsigned char*)malloc(num_kp_ref ? num_kp_ref : 1);
    for (k = 1; k < n_seeds; ++k) {
        memset(tmp.is_triangulated, 0, num_kp_ref ? num_kp_ref : 1);
        init_attempt(seed_list[k], ref_keypts, num_kp_ref, ref_bearings, cur_keypts, num_kp_cur, cur_bearings, ref_matched_2_in_1,
                     ref_cam, cur_cam, ref_cam_matrix, cur_cam_matrix, params, &tmp);
        if (tmp.verdict == SV_INIT_SUCCESS && (!have_best || tmp.hyps[tmp.selected_hyp].num_valid_pts > best_score)) {
            int* m2 = out->matched_2_in_1;
            unsigned char *ih = out->inlier_h, *jf = out->inlier_f, *it = out->is_triangulated;
            double* tp = out->triangulated_pts;
            best_score = tmp.hyps[tmp.selected_hyp].num_valid_pts;
            have_best = 1;
            *out = tmp;
            out->matched_2_in_1 = m2; out->inlier_h = ih; out->inlier_f = jf; out->is_triangulated = it; out->triangulated_pts = tp;
            memcpy(m2, tmp.matched_2_in_1, sizeof(int) * num_kp_ref);
            memcpy(ih, tmp.inlier_h, nm);
            memcpy(jf, tmp.inlier_f, nm);
            memcpy(it, tmp.is_triangulated, num_kp_ref);
            memcpy(tp, tmp.triangulated_pts, sizeof(double) * 3 * num_kp_ref);
        }
    }
    free(tmp.matched_2_in_1); free(tmp.inlier_h); free(tmp.inlier_f); free(tmp.triangulated_pts); free(tmp.is_triangulated);
}
