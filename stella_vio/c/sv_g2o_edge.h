/* SPDX-License-Identifier: BSD-2-Clause */
#ifndef SV_G2O_EDGE_H
#define SV_G2O_EDGE_H

#include "sv_g2o_se3.h"

/* stella_vslam's mono_perspective_pose_opt_edge / mono_perspective_reproj_edge
 * (g2o::BaseUnaryEdge<2,...>/BaseBinaryEdge<2,...>) plus g2o's
 * RobustKernelHuber and BaseFixedSizedEdge::constructQuadraticForm --
 * BSD (g2o BSD-2 notice; stella-vslam BSD-2, AIST 2019 / stella-cv 2022):
 *   external/candidates/stella_vslam/src/stella_vslam/optimize/internal/
 *     se3/perspective_pose_opt_edge.h (mono_perspective_pose_opt_edge)
 *     se3/perspective_reproj_edge.h (mono_perspective_reproj_edge)
 *   external/candidates/g2o/g2o/core/robust_kernel_impl.cpp
 *     (RobustKernelHuber::robustify)
 *   external/candidates/g2o/g2o/core/base_fixed_sized_edge.hpp
 *     (constructQuadraticForm: H = rho1*Omega, b = -rho1*Omega*error,
 *     second-order rho2 term dropped -- matches g2o's own approximation)
 * Only monocular/perspective, isotropic information (Identity*inv_sigma_sq,
 * as pose_opt_edge_wrapper/reproj_edge_wrapper always construct it for the
 * perspective camera model) is ported -- fisheye/equirectangular/stereo
 * share the same formulas in stella_vslam modulo cam_project, not ported.
 */

typedef struct sv_pose_opt_edge {
    double pos_w[3];       /* fixed landmark position (world) */
    double obs[2];         /* measurement (undistorted keypoint px) */
    double inv_sigma_sq;   /* isotropic information = inv_sigma_sq * I2 */
    double fx, fy, cx, cy; /* perspective intrinsics */
    double huber_delta;    /* sqrt_chi_sq (2D: sqrt(5.99146)) */
    int use_robust_kernel; /* 0 once num_trials_robust_ rounds are done */
    int level;             /* 0 = inlier/active, 1 = outlier/excluded */
} sv_pose_opt_edge;

/* mono_perspective_pose_opt_edge::cam_project. */
void sv_pose_opt_edge_project(const sv_pose_opt_edge* e, const double pos_c[3], double out[2]);

/* computeError(): error = obs - project(pose.map(pos_w)). */
void sv_pose_opt_edge_error(const sv_pose_opt_edge* e, const sv_se3* pose, double error[2]);

/* chi2() = error^T * information * error (information = inv_sigma_sq*I2,
 * so this is inv_sigma_sq * (ex*ex + ey*ey), NOT robust-weighted --
 * matches g2o::OptimizableGraph::Edge::chi2()). */
double sv_pose_opt_edge_chi2(const sv_pose_opt_edge* e, const double error[2]);

/* linearizeOplus(): 2x6 Jacobian w.r.t. the pose's se(3) tangent update,
 * row-major out[row*6+col]. */
void sv_pose_opt_edge_jacobian(const sv_pose_opt_edge* e, const sv_se3* pose, double jac[2][6]);

/* depth_is_positive(). */
int sv_pose_opt_edge_depth_positive(const sv_pose_opt_edge* e, const sv_se3* pose);

/* RobustKernelHuber::robustify(chi2, rho). rho[1] is g2o's Gauss-Newton
 * downweight; rho[2] (2nd order term) is measured but never applied by
 * BaseFixedSizedEdge::constructQuadraticForm (see its commented-out
 * 2*rho[2]*... term), so it is not returned here. */
void sv_huber_robustify(double delta, double chi2, double rho[3]);

/* constructQuadraticForm contribution of this edge to the (single, 6x6)
 * pose normal equations: H += weight*inv_sigma_sq*J^T*J,
 * b += -weight*inv_sigma_sq*J^T*error, where weight = e->use_robust_kernel
 * ? rho[1] : 1 (rho computed internally from chi2()). Only accumulates
 * when e->level == 0 (g2o's SparseOptimizer::initializeOptimization only
 * activates level-0 edges). H is row-major upper-triangle-and-lower packed
 * as a full 6x6 (out row-major out[r*6+c]), b is a 6-vector. */
void sv_pose_opt_edge_accumulate(const sv_pose_opt_edge* e, const sv_se3* pose,
                                  double H[6][6], double b[6]);

#endif /* SV_G2O_EDGE_H */
