/* SPDX-License-Identifier: BSD-3-Clause */
/* Own C99 code following the ideas of PoseLib (Viktor Larsson, BSD-3-Clause, see LICENSES/poselib-BSD-3-Clause.txt):
 * LO-RANSAC with MSAC scoring (5pt relative pose, P3P absolute pose), non-linear refinement with a Cauchy loss
 * (Sampson error for the relative pose, reprojection for the absolute pose). Matrices are column-major 3x3 (m[col*3+row]),
 * poses map x -> R x + t. Opt-in from sv_init.c (init_refine, init_lo) and sv_relocalizer.c / sv_system.c (pnp_lo). */
#ifndef SV_POSELIB_H
#define SV_POSELIB_H
#include "sv_rng.h"
#include "sv_pnp.h"
#ifdef __cplusplus
extern "C" {
#endif

/* Sampson distance (normalized-plane units) of the pair of unit bearings under the essential matrix E21 (x2^T E x1 = 0); huge if a bearing is behind/at the plane. */
double sv_sampson(const double E[9], const double b1[3], const double b2[3]);
/* E = [t]x R. */
void sv_essential_from_pose(const double R[9], const double t[3], double E[9]);

/* Gauss-Newton / LM refinement of (R, |t| kept, direction of t) on the n pairs of unit bearings with mask[i] != 0 (mask NULL = all), Cauchy loss with
 * scale `scale` (normalized-plane units), `iters` LM iterations. Returns the number of pairs used (0 = nothing done). */
unsigned sv_relpose_refine(const double* b1, const double* b2, unsigned n, const unsigned char* mask, double scale, unsigned iters,
                           double R[9], double t[3]);
/* Cheirality: index (0..3) of the sv_solve_essential_decompose() hypothesis with most inlier points in front of both cameras. */
unsigned sv_relpose_pick(const double E[9], const double* b1, const double* b2, unsigned n, const unsigned char* mask, double R[9], double t[3]);
/* LO-RANSAC with the 5-point solver, MSAC scoring at threshold `thr` (Sampson distance, normalized-plane units), local optimisation = refine + re-score
 * on the inliers of every new best model. Output: essential matrix E (of the refined pose), inlier mask (n bytes), number of inliers (0 = no model).
 * Deterministic for a given rng state. Returns 0 on success (also when no model is found), -1 on allocation failure. */
int sv_relpose_lo_ransac(const double* b1, const double* b2, unsigned n, double thr, unsigned max_iters, unsigned min_iters, sv_mt19937* rng,
                         double E[9], unsigned char* mask, unsigned* num_inliers);

/* Three-point absolute pose: bearings b (3 unit vectors, consecutive xyz), world points P (3, consecutive xyz) -> up to 4 poses with
 * R P_i + t along b_i (positive depths). Grunert distances (quartic in the depth ratio) + rigid alignment. Returns the number of poses. */
int sv_p3p(const double b[9], const double P[9], double R[4][9], double t[4][3]);
/* LM refinement of a pose on m correspondences (unit bearings, world points, per-point scale sigma in normalized-plane units), Cauchy loss. */
void sv_pnp_refine(const double* b, const double* p, const double* sigma, unsigned m, unsigned iters, double R[9], double t[3]);
/* Drop-in for sv_pnp_ransac (same cosine thresholds, same cost, same validity rule count > min_inliers): P3P minimal solver, MSAC cost, LO on every
 * new best (refine on inliers, re-score, up to 3 rounds), adaptive iteration count up to max_iters (min 20). */
int sv_pnp_lo_ransac(const double* b, const double* p, const int* octaves, unsigned n, const float* scales, unsigned levels, unsigned min_inliers,
                     unsigned max_iters, sv_mt19937* rng, sv_pnp_result* result, unsigned char* mask);

#ifdef __cplusplus
}
#endif
#endif
