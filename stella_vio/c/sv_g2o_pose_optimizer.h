/* SPDX-License-Identifier: BSD-2-Clause */
#ifndef SV_G2O_POSE_OPTIMIZER_H
#define SV_G2O_POSE_OPTIMIZER_H

#include "sv_g2o_edge.h"
#include "sv_g2o_se3.h"

/* stella_vslam::optimize::pose_optimizer_g2o::optimize (the 3-arg private
 * overload) driving g2o::OptimizationAlgorithmLevenberg /
 * g2o::SparseOptimizer::optimize over a single (always non-fixed) 6-dof
 * shot_vertex with monocular perspective unary reprojection edges --
 * BSD (g2o BSD-2 notice; stella-vslam BSD-2, AIST 2019 / stella-cv 2022):
 *   external/candidates/stella_vslam/src/stella_vslam/optimize/
 *     pose_optimizer_g2o.cc, terminate_action.cc
 *   external/candidates/g2o/g2o/core/optimization_algorithm_levenberg.cpp
 *     (the do-while trial loop: lambda init = tau*maxDiag(H) with
 *     tau=1e-5, ni doubling, rho/scale accept-reject,
 *     _maxTrialsAfterFailure=10)
 *   external/candidates/g2o/g2o/core/block_solver.hpp (setLambda: additive
 *     diag += lambda, not multiplicative)
 *
 * Scope note: this is the single-vertex, unary-edge case only, so it needs
 * NO Schur complement over marginalized landmarks and no multi-block AMD
 * fill-reduction ordering (g2o's BlockSolver has exactly one pose block).
 * Confirmed bit-exact against 2652 real pose_optimizer_g2o::optimize()
 * calls captured from a real g2o/stella_vslam build on fr1_xyz + fr1_desk
 * (`check_sv_g2o_pose.c`, 0/1570 + 0/1082) -- see sv_eigen_llt.h/
 * sv_eigen_amd.h for the actual solver (SimplicialLLT, not LDLT) and the
 * AMD-is-identity-for-this-leaf measurement, and HANDOVER.md's module-4b
 * "bit-exact closure" entry for the full bug list this took to close.
 * Local/global BA (multi-vertex, Schur complement, real multi-block AMD
 * permutation) is the separate module 4b part 2: sv_g2o_ba.h /
 * sv_bundle_adjuster.h (this leaf's dense 6x6 identity-AMD shortcut is
 * unchanged).
 */

typedef struct sv_pose_optimizer_params {
    unsigned int num_trials_robust; /* stella default: 4 for frame tracking */
    unsigned int num_trials;        /* stella default: 6 (5 for keyframe reloc) */
    unsigned int num_each_iter;     /* g2o iterations per trial; stella: 10 */
} sv_pose_optimizer_params;

/* Runs stella's robust-then-plain outlier-reclassification trial loop.
 * `edges` (length num_edges) must already be filtered to valid, non-erased
 * landmark observations, each with pos_w/obs/inv_sigma_sq/fx../huber_delta
 * filled in and level=0, use_robust_kernel = (params->num_trials_robust>0).
 * On return: `pose` holds the optimized estimate, `edges[i].level` holds
 * the final inlier(0)/outlier(1) classification (mirrors
 * outlier_flags.at(idx_) in pose_optimizer_g2o.cc), and the return value
 * is num_edges - num_bad_obs (0 if num_edges < 5, in which case `pose` is
 * left unmodified, matching stella's `if (num_init_obs < 5) return 0;`
 * which skips optimization entirely). */
unsigned int sv_pose_optimizer_optimize(sv_se3* pose, sv_pose_opt_edge* edges, int num_edges,
                                         const sv_pose_optimizer_params* params);

#endif /* SV_G2O_POSE_OPTIMIZER_H */
