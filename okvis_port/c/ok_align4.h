/* SPDX-License-Identifier: BSD-3-Clause AND MPL-2.0 */
/*
 * OKVIS2-X Align4DoF_Ceres (ViGraph.cpp:35-110, commit 38043e4): the 4-DoF T_GW refinement of checkForGpsInit with
 * robust_gps_init. Three leaves:
 *   - FourDoFResidual evaluated as ceres::AutoDiffCostFunction<FourDoFResidual, 3, 7> executes it: the quaternion-vector
 *     product of Eigen::Quaternion<ceres::Jet<double,7>> (QuaternionBase::_transformVector: uv = vec x v; uv += uv;
 *     v + w * uv + vec x uv), Jet +, -, * as ceres/jet.h (f.a*g.v + f.v*g.a, no FMA), residuals / Jacobian (row-major 3x7);
 *   - Eigen 3.4.0 HouseholderQR<ColMajor MatrixXd>::solve (ceres EigenDenseQR, DENSE_QR): ok_eigen_hqr_solve;
 *   - the Ceres problem itself (one PoseManifold4d block, N CauchyLoss(3) blocks, LEVENBERG_MARQUARDT, DENSE_QR, 100 iterations)
 *     run through the ported minimizer of ok_solve.c (ok_sv_solve, linear_solver_type OK_SV_DENSE_QR).
 * Derived from OKVIS2-X (BSD-3-Clause, see ok_gps.h for the notices), Ceres Solver 2.2.0 (BSD-3-Clause, Copyright 2023 Google Inc.)
 * and Eigen 3.4.0 (MPL-2.0, Copyright (C) Gael Guennebaud, Benoit Jacob and the Eigen authors).
 * C99. Verified by okvis_port/reference_tools/okvis_align4_test.cc against the real classes (tolerance 0).
 */
#ifndef OK_ALIGN4_H
#define OK_ALIGN4_H
#include "ok_kin.h"
#include "ok_solve.h"

typedef struct ok_align4_term { double pG[3], pW[3]; } ok_align4_term;

/* FourDoFResidual: x = params[7] (r, q x y z w), res[3] = pG - (q * pW + r). jac (row-major 3x7) may be NULL: then the
 * double instantiation (T = double) is evaluated, else the Jet instantiation (residuals = the .a parts). */
void ok_align4_residual(const ok_align4_term* t, const double x[7], double res[3], double* jac);

/* HouseholderQR<MatrixXd>(Map<ColMajor>(lhs, rows, cols)).solve(rhs) -> x (cols values); lhs is destroyed (the QR is computed
 * in a copy, as Eigen does, so only the arithmetic matters). rows >= cols, cols <= 8. */
void ok_eigen_hqr_solve(int rows, int cols, const double* lhs, const double* rhs, double* x);

/* Align4DoF_Ceres(points_G, points_W, T_GW_init, T_GW_refined): flat arrays [x y z]*n. hooks (may be NULL) observe the
 * minimizer (ok_sv_hooks). Returns the ceres TerminationType; T_out = PoseParameterBlock::estimate() (cached). */
int ok_align4dof_ceres(int n, const double* points_G, const double* points_W, const ok_tf* T_init, ok_tf* T_out,
                       const ok_sv_hooks* hooks);

#endif
