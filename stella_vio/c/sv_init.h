/* SPDX-License-Identifier: BSD-2-Clause */
#ifndef SV_INIT_H
#define SV_INIT_H

#include "sv_types.h"
#include "sv_frame.h"
#include "sv_rng.h"
#include "sv_triangulate.h"

/* Port of stella_vslam's module::initializer::try_initialize_for_monocular()
 * (area matching, margin=100) + initialize::perspective::initialize()
 * (H/F RANSAC via two independent-engine solvers -- order doesn't affect
 * output, only interleaving WITHIN each engine's own call sequence does,
 * and each stella_vslam solve::{homography,fundamental}_solver owns an
 * independent std::mt19937, so running them sequentially here is
 * bit-exact) + initialize::base::find_most_plausible_pose()/triangulate()
 * (the 2x2-closed-form overload, no SVD -- see sv_triangulate.h). Does NOT
 * cover module::initializer::create_map_for_monocular() (global BA / map
 * scaling), which is out of module-3 scope. BSD-2 (AIST 2019 + stella-cv
 * 2022, see sv_rng.h).
 */

typedef struct sv_init_params {
    unsigned int num_ransac_iters; /* default 100 */
    unsigned int min_num_valid_pts; /* default 50 */
    unsigned int min_num_triangulated_pts; /* default 50 */
    float parallax_deg_thr; /* default 1.0 */
    float reproj_err_thr; /* default 4.0 */
    unsigned int num_seeds; /* stella_vio: RANSAC seeds tried, best kept (1 = exact port: seed 5489) */
    float par_frac; /* stella_vio: 0 (default) = parallax of the 50th point; > 0 = of the point at this fraction of the valid points */
} sv_init_params;

typedef enum sv_init_verdict {
    SV_INIT_RESET_TOO_FEW_MATCHES, /* num_matches < min_num_valid_pts: caller resets ref frame */
    SV_INIT_FAIL_NO_VALID_MODEL, /* neither H nor F solution valid */
    SV_INIT_FAIL_POSE_NOT_FOUND, /* find_most_plausible_pose rejected (any of its checks) */
    SV_INIT_SUCCESS
} sv_init_verdict;

typedef enum sv_init_model { SV_INIT_MODEL_NONE, SV_INIT_MODEL_H, SV_INIT_MODEL_F } sv_init_model;

typedef struct sv_init_hypothesis {
    double rot[9];
    double trans[3];
    unsigned int num_valid_pts;
    unsigned int num_triangulated_pts;
    float parallax_cos;
} sv_init_hypothesis;

typedef struct sv_init_attempt_result {
    unsigned int num_matches;
    int* matched_2_in_1; /* caller-allocated, num_kp1 entries (== ref frame's) */

    int h_valid, f_valid;
    float cost_h, cost_f;
    float rel_cost_h;
    double best_H21[9];
    double best_F21[9];
    unsigned char* inlier_h; /* caller-allocated, num_matches entries (indexed like ref_cur_matches_, i.e. i-th valid match) */
    unsigned char* inlier_f;

    sv_init_model model_chosen;
    unsigned int num_hyps; /* 8 (H) or 4 (F), 0 if model_chosen==NONE */
    sv_init_hypothesis hyps[8];
    unsigned int selected_hyp; /* index into hyps[], valid iff verdict==SUCCESS */

    sv_init_verdict verdict;
    /* on SUCCESS: */
    double rot_ref_to_cur[9];
    double trans_ref_to_cur[3];
    double* triangulated_pts; /* caller-allocated, num_kp1 entries * 3 doubles, row-major (idx*3+{0,1,2}) */
    unsigned char* is_triangulated; /* caller-allocated, num_kp1 entries */
} sv_init_attempt_result;

/* ref_matched_2_in_1: length num_kp_ref, from a prior sv_match_in_consistent_area
 * call (caller runs the matcher; see header comment -- this keeps the
 * matcher's prev_matched_pts state management with the caller, matching
 * module::initializer's own ref-frame-reset-on-failure state machine). */
void sv_init_try_monocular(
    const sv_keypoint* ref_keypts, unsigned int num_kp_ref, const double* ref_bearings /* num_kp_ref*3 */,
    const sv_keypoint* cur_keypts, unsigned int num_kp_cur, const double* cur_bearings /* num_kp_cur*3 */,
    const int* ref_matched_2_in_1,
    const sv_camera_perspective* ref_cam, const sv_camera_perspective* cur_cam,
    const double ref_cam_matrix[9], const double cur_cam_matrix[9],
    const sv_init_params* params,
    sv_init_attempt_result* out);

#endif /* SV_INIT_H */
