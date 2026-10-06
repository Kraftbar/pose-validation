/* SPDX-License-Identifier: Apache-2.0 */
/*
 * RD-VIO pure-C port: find_pnp_matrix_parsac_imu (rdvio_geometry/include/rdvio/geometry/pnp.h) and its IMU_Parsac
 * (rdvio_util/include/rdvio/util/imu_parsac.h), the PARSAC of module M3 with:
 *  - a prior pose (the IMU-predicted camera pose): the points within 2x the threshold of it form the prior inlier set; fewer
 *    than 20 of them, or under 15 %, and the solve stops at once with an all-ones mask (no confidence update);
 *  - hypotheses are scored only if at least 6 of their inliers are prior inliers, and that overlap count is the tie-break
 *    and the inlier count of the result;
 *  - the per-bin confidence is weighted by 1 - dynamic_prob^(0.1 * mean track length of the bin);
 *  - no accepted hypothesis: an all-ones mask, no confidence update.
 * Quirks kept: the weighted draw returns a VALID-BIN index that is used as a DATA index (as in Parsac); the adaptive
 * iteration bound uses inlier_ratio^5 for this 6-point model.
 * The minimal solver (solve_pnp_6pt: OpenCV EPnP + Rodrigues in float) is a callback until module M7b ports it.
 * Derived from RD-VIO (Apache-2.0). C99.
 */
#ifndef RD_IMU_PARSAC_H
#define RD_IMU_PARSAC_H
#include <stddef.h>
#include "rd_ransac.h"

/* solve_pnp_6pt: X[i] 3D points, x[i] normalized image points -> T (4x4, column-major) */
typedef void (*rd_pnp6_fn)(void* ctx, const double X[6][3], const double x[6][2], double T[16]);

/* find_pnp_matrix_parsac_imu(Xs, xs, lens, R, t, dynamic_prob, scale, mask, threshold, confidence, max_iteration, seed).
 * st: the function's static binConfidences. Xs n x 3, xs n x 2, R column-major. Fills mask[n] and T; returns the inlier
 * count (the prior-overlap count), or (size_t)-1 if a point lies outside the (-scale, scale)^2 grid (undefined in the C++). */
size_t rd_find_pnp_matrix_parsac_imu(rd_parsac_state* st, size_t n, const double* Xs, const double* xs, const size_t* lens,
                                     const double R[9], const double t[3], double dynamic_prob, double scale, char* mask,
                                     double threshold, double confidence, size_t max_iteration, int seed,
                                     rd_pnp6_fn solve, void* ctx, double T[16]);
#endif
