/* SPDX-License-Identifier: BSD-2-Clause */
#ifndef SV_SOLVE_FUNDAMENTAL_H
#define SV_SOLVE_FUNDAMENTAL_H

#include "sv_rng.h"
#include "sv_types.h"
#include "sv_solve_homography.h" /* sv_match_pair */

/* Port of stella_vslam's solve/fundamental_solver.{h,cc} -- BSD-2
 * (AIST 2019 + stella-cv 2022, see sv_rng.h); SVD via sv_eigen_svd.h
 * (MPL-2.0). Column-major 3x3 (sv_linalg.h convention). */

void sv_solve_compute_F21(const float* x1, const float* y1,
                           const float* x2, const float* y2, int n,
                           double F21[9]);

/* fundamental_solver::decompose: F -> E -> essential_solver::decompose (4
 * hypotheses). Always returns 1 (matches stella's signature/behavior). */
int sv_solve_fundamental_decompose(const double F21[9], const double cam1[9], const double cam2[9],
                                    double rots[4][9], double transes[4][3]);

typedef struct sv_fundamental_ransac_result {
    int solution_valid;
    float best_cost;
    double best_F21[9];
    unsigned char* is_inlier;
} sv_fundamental_ransac_result;

void sv_solve_fundamental_find_via_ransac(
    const sv_keypoint* undist_1, unsigned int num_kp1,
    const sv_keypoint* undist_2, unsigned int num_kp2,
    const sv_match_pair* matches, unsigned int num_matches,
    float sigma, unsigned int max_num_iter, int recompute,
    sv_mt19937* engine,
    sv_fundamental_ransac_result* out);

#endif /* SV_SOLVE_FUNDAMENTAL_H */
