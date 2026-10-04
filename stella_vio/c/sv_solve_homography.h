/* SPDX-License-Identifier: BSD-2-Clause */
#ifndef SV_SOLVE_HOMOGRAPHY_H
#define SV_SOLVE_HOMOGRAPHY_H

#include "sv_rng.h"
#include "sv_types.h"

/* Port of stella_vslam's solve/homography_solver.{h,cc} -- BSD-2
 * (AIST 2019 + stella-cv 2022, see sv_rng.h); compute_H_21/decompose use
 * Eigen::JacobiSVD via sv_eigen_svd.h (MPL-2.0, not reproduced here).
 * All 3x3/Vec3 matrices are column-major doubles (sv_linalg.h convention).
 */

typedef struct sv_match_pair {
    int idx1, idx2; /* into keypts_1[]/keypts_2[] */
} sv_match_pair;

/* compute_H_21: keypts are normalized cv::Point2f (float) coords, n>=4.
 * Returns 0 if degenerate (svd.rank() < 8), 1 and fills H21 otherwise. */
int sv_solve_compute_H21(const float* x1, const float* y1,
                          const float* x2, const float* y2, int n,
                          double H21[9]);

/* homography_solver::decompose: 8 (R,t,normal) hypotheses. Returns 0 if
 * the rank condition fails (d1/d2<1.0001 || d2/d3<1.0001). rots/transes/
 * normals: caller-allocated, 8 entries each (9/3/3 doubles per entry,
 * column-major). */
int sv_solve_homography_decompose(const double H21[9], const double cam1[9], const double cam2[9],
                                   double rots[8][9], double transes[8][3], double normals[8][3]);

typedef struct sv_homography_ransac_result {
    int solution_valid;
    float best_cost;
    double best_H21[9];
    unsigned char* is_inlier; /* caller-allocated, num_matches entries */
} sv_homography_ransac_result;

/* find_via_ransac, orchestrating create_random_array (sv_rng) +
 * compute_H_21 + check_inliers exactly as homography_solver::find_via_ransac
 * (min_set_size=4, chi_sq=5.991, sigma passed by caller). undist_1/2: full
 * (unnormalized) keypoint arrays; matches: ref_cur_matches_ pairs, length
 * num_matches. engine: fresh default-seeded sv_mt19937 (use_fixed_seed=true
 * path), matching a fresh homography_solver's random_engine_. */
void sv_solve_homography_find_via_ransac(
    const sv_keypoint* undist_1, unsigned int num_kp1,
    const sv_keypoint* undist_2, unsigned int num_kp2,
    const sv_match_pair* matches, unsigned int num_matches,
    float sigma, unsigned int max_num_iter, int recompute,
    sv_mt19937* engine,
    sv_homography_ransac_result* out);

#endif /* SV_SOLVE_HOMOGRAPHY_H */
